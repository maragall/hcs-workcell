"""
Solid bodies the planner routes around, from layout.json `volumes` (the manufacturer CAD).

The pose the controller reports is the wrist axis. The fingers are 205 mm ahead of it, so the claw, not the wrist,
is what reaches the hotel. `Volumes.check_pose` tests the claw's footprint (body and fingers, in the tool frame,
turned by the yaw) and the two links against every obstacle at the heights they occupy; `check_leg` samples a straight
move, yaw and z interpolated; `search` finds a path in x, y at constant height around the obstacles.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, List

from procedures.safety import Safety, SafetyViolation
from routes_opt import fk

SAMPLE_MM = 8.0          # footprint grid and leg sampling
GRID_MM = 20.0           # search grid
LINK_STEP_MM = 12.0


class Volumes:
    def __init__(self, layout: dict, safety: Safety):
        v = layout.get("volumes") or {}
        self.margin = float(v.get("margin_mm", 10.0))
        self.obstacles: Dict[str, dict] = v.get("obstacles", {})
        claw = v.get("claw") or {}
        self.z, self.links_z = claw.get("z", [-25.0, 60.0]), claw.get("links_z", [-25.0, 130.0])
        self.footprint = [(r["x"], r["y"]) for r in (claw.get("body"), claw.get("fingers")) if r]
        self.safety = safety
        self.box = safety.box
        self._ok: Dict[tuple, bool] = {}

    @property
    def active(self) -> bool:
        return bool(self.obstacles and self.footprint)

    # ---- one pose
    def _inside(self, name: str, x: float, y: float, z_lo: float, z_hi: float) -> bool:
        b, m = self.obstacles[name], self.margin
        return (b["x"][0] - m <= x <= b["x"][1] + m and b["y"][0] - m <= y <= b["y"][1] + m
                and z_hi >= b["z"][0] - m and z_lo <= b["z"][1] + m)

    def _claw_samples(self, x: float, y: float, yaw: float):
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        for (x0, x1), (y0, y1) in self.footprint:
            nx, ny = max(1, int((x1 - x0) / SAMPLE_MM)), max(1, int((y1 - y0) / SAMPLE_MM))
            for i in range(nx + 1):
                for j in range(ny + 1):
                    u, w = x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny
                    yield x + u * c - w * s, y + u * s + w * c

    def check_pose(self, p: List[float], label: str = "pose") -> None:
        x, y, z, yaw = p[0], p[1], p[2], p[5]
        for px, py in self._claw_samples(x, y, yaw):
            for name in self.obstacles:
                if self._inside(name, px, py, z + self.z[0], z + self.z[1]):
                    raise SafetyViolation(f"{label}: the claw at x={px:.0f} y={py:.0f} would be inside the {name} "
                                          f"(wrist x={x:.0f} y={y:.0f} z={z:.0f}, yaw {yaw:.0f})")
        for branch in self.safety._branches():
            try:
                elbow = fk.elbow_xy(fk.ik(p, elbow=branch))
            except ValueError as e:
                raise SafetyViolation(f"{label}: {e}")
            base = (fk.MODEL["x0"], fk.MODEL["y0"])
            for part, a, b in (("upper arm", base, elbow), ("forearm", elbow, (x, y))):
                n = max(1, int(math.dist(a, b) / LINK_STEP_MM))
                for k in range(n + 1):
                    qx, qy = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
                    for name, o in self.obstacles.items():
                        if o.get("links", True) and self._inside(name, qx, qy, z + self.links_z[0], z + self.links_z[1]):
                            raise SafetyViolation(f"{label}: the {part} at x={qx:.0f} y={qy:.0f} would be inside the {name}")

    def check_leg(self, a: List[float], b: List[float], label: str = "leg") -> None:
        n = max(1, int(max(math.dist(a[:2], b[:2]), abs(b[2] - a[2]), abs(b[5] - a[5]) * 1.5) / SAMPLE_MM))
        for k in range(n + 1):
            t = k / n
            self.check_pose([a[i] + t * (b[i] - a[i]) for i in range(6)], f"{label} ({k}/{n})")

    # ---- search
    def _leg_ok(self, a: List[float], b: List[float]) -> bool:
        key = (round(a[0]), round(a[1]), round(b[0]), round(b[1]), round(a[2]), round(a[5]))
        if key not in self._ok:
            try:
                self.safety.check_segment(a, b, "search")
                self.check_leg(a, b)
                self._ok[key] = True
            except SafetyViolation:
                self._ok[key] = False
        return self._ok[key]

    def search(self, here: List[float], target: List[float]) -> List[List[float]]:
        """Waypoints from `here` to `target`: turn the claw in place to a yaw that clears, travel at the higher of the
        two heights around the obstacles, turn to the target's yaw over it. Tries the current yaw, the target's and the
        four quarter turns; the shortest path wins. Raises SafetyViolation when none exists."""
        best, why = None, None
        for yaw in dict.fromkeys([here[5], target[5], 0.0, 90.0, -90.0, 180.0]):
            try:
                path = self._search_at(here, target, yaw)
            except SafetyViolation as e:
                why = e
                continue
            if path[0] != list(here):
                path = [list(here)] + path                # the first leg turns the claw in place
            length = sum(math.dist(a[:2], b[:2]) + abs(b[5] - a[5]) for a, b in zip(path, path[1:]))
            if best is None or length < best[0]:
                best = (length, path)
        if best is None:
            raise why or SafetyViolation("no route")
        return best[1]

    def _search_at(self, here: List[float], target: List[float], yaw: float) -> List[List[float]]:
        z = max(here[2], target[2])
        rot = [here[3], here[4], yaw]
        start, goal = [here[0], here[1], z] + rot, [target[0], target[1], z] + rot
        if start != list(here):
            try:
                self.safety.check_segment(here, start, "turn")
                self.check_leg(here, start, "turn")
            except SafetyViolation as e:
                raise SafetyViolation(f"cannot turn the claw to yaw {yaw:.0f} here: {e}")
        gx = [self.box["x"][0] + i * GRID_MM for i in range(int((self.box["x"][1] - self.box["x"][0]) / GRID_MM) + 1)]
        gy = [self.box["y"][0] + j * GRID_MM for j in range(int((self.box["y"][1] - self.box["y"][0]) / GRID_MM) + 1)]
        nodes = {("s",): start, ("g",): goal}
        for i, x in enumerate(gx):
            for j, y in enumerate(gy):
                nodes[(i, j)] = [x, y, z] + rot

        def neighbours(k):
            if k == ("s",) or k == ("g",):
                p = nodes[k]
                return [m for m, q in nodes.items() if m[0] not in ("s", "g") and math.dist(p[:2], q[:2]) <= 2.2 * GRID_MM] + \
                       ([("g",)] if k == ("s",) and math.dist(start[:2], goal[:2]) <= 3 * GRID_MM else [])
            i, j = k
            out = [(i + di, j + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di or dj) and (i + di, j + dj) in nodes]
            if math.dist(nodes[k][:2], goal[:2]) <= 2.2 * GRID_MM:
                out.append(("g",))
            return out

        dist, prev, heap = {("s",): 0.0}, {}, [(0.0, 0, ("s",), 0.0)]
        count = 0
        while heap:
            _, _, k, d = heapq.heappop(heap)
            if k == ("g",):
                break
            if d > dist.get(k, 1e18):
                continue
            for m in neighbours(k):
                nd = d + math.dist(nodes[k][:2], nodes[m][:2])
                if nd < dist.get(m, 1e18) and self._leg_ok(nodes[k], nodes[m]):
                    dist[m], prev[m] = nd, k
                    count += 1
                    heapq.heappush(heap, (nd + math.dist(nodes[m][:2], goal[:2]) * 0.5, count, m, nd))
        if ("g",) not in prev and start[:2] != goal[:2]:
            raise SafetyViolation("no route around the hotel and the pillar from here: the claw (205 mm ahead of the wrist) "
                                  "cannot pass; move the arm by hand to a taught position first")
        try:                                              # the last leg: over the target to the target (lower, turn)
            self.safety.check_segment(goal, target, "last leg")
            self.check_leg(goal, target, "last leg")
        except SafetyViolation as e:
            raise SafetyViolation(f"yaw {yaw:.0f}: {e}")
        path, k = [], ("g",)
        while k != ("s",):
            path.append(nodes[k])
            k = prev[k]
        path.append(start)
        path.reverse()
        out = [path[0]]                                   # straighten: skip nodes that a straight leg clears
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._leg_ok(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out
