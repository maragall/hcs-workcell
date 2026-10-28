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
from typing import Dict, List, Optional

from procedures.safety import Safety, SafetyViolation
from routes_opt import fk

SAMPLE_MM = 8.0          # footprint grid and leg sampling
GRID_MM = 20.0           # search grid
LINK_STEP_MM = 12.0
RELAX_MM = 40.0          # around a start pose that already stands inside the margin
RELAX_SHRINK_MM = 5.0


class Volumes:
    def __init__(self, layout: dict, safety: Safety, holding: Optional[str] = None):
        v = layout.get("volumes") or {}
        self.margin = float(v.get("margin_mm", 10.0))
        self.obstacles: Dict[str, dict] = v.get("obstacles", {})
        claw = v.get("claw") or {}
        self.z, self.links_z = claw.get("z", [-25.0, 60.0]), claw.get("links_z", [-25.0, 130.0])
        self.footprint = [(r["x"], r["y"], False) for r in (claw.get("body"), claw.get("fingers")) if r]
        held = (v.get("held") or {}).get(holding or "")
        if held:                                    # the plate in the claw is part of what moves
            self.footprint.append((held["x"], held["y"], True))
        self.front_allow = float((v.get("held") or {}).get("hotel_front_allowance_mm", 0.0))
        self.axis_keepout = float(v.get("axis_keepout_mm", 0.0))
        self.safety = safety
        self.box = safety.box
        self.relax: Optional[List[float]] = None       # where the arm is now, if it already stands within the margin
        self._ok: Dict[tuple, bool] = {}

    @property
    def active(self) -> bool:
        return bool(self.obstacles and self.footprint)

    # ---- one pose
    def _inside(self, name: str, x: float, y: float, z_lo: float, z_hi: float, m: Optional[float] = None,
                front: float = 0.0) -> bool:
        b = self.obstacles[name]
        m = self.margin if m is None else m
        return (b["x"][0] - m + front <= x <= b["x"][1] + m and b["y"][0] - m <= y <= b["y"][1] + m
                and z_hi >= b["z"][0] - m and z_lo <= b["z"][1] + m)

    def _claw_samples(self, x: float, y: float, yaw: float):
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        for (x0, x1), (y0, y1), held in self.footprint:
            nx, ny = max(1, int((x1 - x0) / SAMPLE_MM)), max(1, int((y1 - y0) / SAMPLE_MM))
            for i in range(nx + 1):
                for j in range(ny + 1):
                    u, w = x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny
                    yield x + u * c - w * s, y + u * s + w * c, held

    def settle(self, here: List[float]) -> None:
        """The arm may already stand where the model overlaps an obstacle by a hair (folded at home beside the pillar:
        the claw's bounding box overlaps the pillar's corner by a millimetre; the arm is physically there). It must be
        able to leave: within RELAX_MM of this pose obstacles shrink by RELAX_SHRINK_MM. A deeper overlap still raises."""
        try:
            self.check_pose(here, "the arm now", near_axis_ok=True)
        except SafetyViolation:
            self.relax = list(here[:3])
            self.check_pose(here, "the arm now", near_axis_ok=True)           # raises when the overlap is deeper than the shrink

    def withdraw(self, here: List[float]) -> Optional[List[float]]:
        """The arm stands inside an obstacle (it touched the hotel and stopped): the first spot, straight back along the
        claw's axis in steps of 10 mm, where the claw and links are clear with the full margin. None if there is none."""
        c, s = math.cos(math.radians(here[5])), math.sin(math.radians(here[5]))
        for mm in range(10, 101, 10):
            p = [here[0] - mm * c, here[1] - mm * s] + list(here[2:])
            try:
                self.safety.check_target(p, "withdraw")
                saved, self.relax = self.relax, None
                try:
                    self.check_pose(p, "withdraw")
                finally:
                    self.relax = saved
                return p
            except SafetyViolation:
                continue
        return None

    def check_pose(self, p: List[float], label: str = "pose", near_axis_ok: bool = False) -> None:
        if self.axis_keepout and not near_axis_ok:
            r = math.hypot(p[0] - fk.MODEL["x0"], p[1] - fk.MODEL["y0"])
            if r < self.axis_keepout:
                raise SafetyViolation(f"{label}: the wrist would be {r:.0f} mm from the arm's own axis; straight moves that close "
                                      f"are a singularity the controller refuses (under {self.axis_keepout:.0f} mm)")
        m = -RELAX_SHRINK_MM if self.relax is not None and math.dist(p[:3], self.relax) <= RELAX_MM else None
        x, y, z, yaw = p[0], p[1], p[2], p[5]
        for px, py, held in self._claw_samples(x, y, yaw):
            for name in self.obstacles:
                front = self.front_allow if held and name == "hotel" else 0.0
                if self._inside(name, px, py, z + self.z[0], z + self.z[1], m, front):
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
                        if o.get("links", True) and self._inside(name, qx, qy, z + self.links_z[0], z + self.links_z[1], m):
                            raise SafetyViolation(f"{label}: the {part} at x={qx:.0f} y={qy:.0f} would be inside the {name}")

    def check_joint_leg(self, a: List[float], b: List[float], label: str = "joint leg", start_actual: bool = False) -> None:
        """A joint move (the arm unfolding from beside its own axis): the joints are interpolated, every sample's claw
        and links are checked. Both elbow branches when the branch is unknown."""
        actual = list(self.safety.joints()[:4]) if (start_actual and self.safety.joints) else None
        for branch in ((fk.elbow_sign(actual),) if actual else self.safety._branches()):
            ja = actual or self.safety._ik(a, branch, label)
            jb = self.safety._ik(b, branch, label, ref=ja)
            n = max(1, int(max(abs(jb[i] - ja[i]) for i in (1, 2, 3)) / 2.0))
            for k in range(n + 1):
                p = fk.fk([ja[i] + k / n * (jb[i] - ja[i]) for i in range(4)])
                self.check_pose([p[0], p[1], p[2], 0.0, 0.0, p[5]], f"{label} ({k}/{n})", near_axis_ok=True)

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
        """Waypoints from `here` to `target`: travel at the higher of the two heights around the obstacles, turning the
        claw wherever a turn in place is clear (the yaws tried: the current one, the target's and the quarter turns).
        Dijkstra over (grid point, yaw) with an x, y guide; then the last leg, over the target to the target. Raises
        SafetyViolation when no path exists."""
        z = max(here[2], target[2])
        yaws = list(dict.fromkeys([here[5], target[5], 0.0, 90.0, -90.0, 180.0]))
        gx = [self.box["x"][0] + i * GRID_MM for i in range(int((self.box["x"][1] - self.box["x"][0]) / GRID_MM) + 1)]
        gy = [self.box["y"][0] + j * GRID_MM for j in range(int((self.box["y"][1] - self.box["y"][0]) / GRID_MM) + 1)]
        xy = {("s",): (here[0], here[1]), ("g",): (target[0], target[1])}
        for i, x in enumerate(gx):
            for j, y in enumerate(gy):
                xy[(i, j)] = (x, y)

        def pose(k, yi):
            return [xy[k][0], xy[k][1], z, here[3], here[4], yaws[yi]]

        def moves(k):
            if k in (("s",), ("g",)):
                near = [m for m in xy if m[0] not in ("s", "g") and math.dist(xy[k], xy[m]) <= 2.2 * GRID_MM]
                return near + ([("g",)] if k == ("s",) else [])        # the straight line is always tried
            i, j = k
            out = [(i + di, j + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di or dj) and (i + di, j + dj) in xy]
            if math.dist(xy[k], xy[("g",)]) <= 2.2 * GRID_MM:
                out.append(("g",))
            return out

        start, goal = (("s",), 0), (("g",), 1)
        dist, prev, heap, count = {start: 0.0}, {}, [(0.0, 0, start, 0.0)], 0
        while heap:
            _, _, st, d = heapq.heappop(heap)
            if st == goal:
                break
            if d > dist.get(st, 1e18):
                continue
            k, yi = st
            options = [((m, yi), math.dist(xy[k], xy[m])) for m in moves(k)]
            options += [((k, yj), abs(yaws[yj] - yaws[yi]) * 0.5) for yj in range(len(yaws)) if yj != yi]
            for nxt, cost in options:
                nd = d + cost
                if nd < dist.get(nxt, 1e18) and self._leg_ok(pose(*st), pose(*nxt)):
                    dist[nxt], prev[nxt] = nd, st
                    count += 1
                    guess = math.dist(xy[nxt[0]], xy[("g",)]) * 0.5 + (0.0 if nxt[1] == 1 else 5.0)
                    heapq.heappush(heap, (nd + guess, count, nxt, nd))
        if goal not in prev and start != goal:
            raise SafetyViolation("no route around the hotel and the pillar from here: the claw (205 mm ahead of the wrist) "
                                  "cannot pass; move the arm by hand to a taught position first")
        chain, st = [], goal
        while st != start:
            chain.append(st)
            st = prev[st]
        chain.append(start)
        chain.reverse()
        path = [pose(*c) for c in chain]
        out = [path[0]]                                   # straighten runs at one yaw: skip nodes a straight leg clears
        i = 0
        while i < len(path) - 1:
            j = i + 1
            while j + 1 < len(path) and path[j + 1][5] == path[i][5] and self._leg_ok(path[i], path[j + 1]):
                j += 1
            out.append(path[j])
            i = j
        try:                                              # the last leg: over the target to the target (lower, turn)
            self.safety.check_segment(out[-1], target, "last leg")
            self.check_leg(out[-1], target, "last leg")
        except SafetyViolation as e:
            raise SafetyViolation(f"last leg: {e}")
        return out
