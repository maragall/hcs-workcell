"""
Safety layer: speed profiles, z-floors per region, sanity box, keep-out regions, and the vision sanity guard.
All numbers come from layout.json; nothing here is hard-coded to this cell.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional

from routes_opt import fk

LINK_SAMPLE_MM = 10.0        # how finely the two arm links are sampled against a keep-out
JOINT_SAMPLE_DEG = 2.0       # how finely a joint move is sampled against a keep-out


class SafetyViolation(Exception):
    pass


@dataclass
class SpeedProfile:
    travel: float       # fraction 0.01..1.0 for joint travel between regions
    approach: float     # near a fixture, before touching
    contact: float      # last mm before grasp / release
    jaw_step: float     # gripper closing step
    jaw_pause: float    # pause between closing steps


class Safety:
    def __init__(self, layout: dict, mode: str, joints: Optional[Callable[[], List[float]]] = None):
        self.layout = layout
        if mode not in layout["speeds"]:
            raise SafetyViolation(f"unknown speed mode {mode}; have {list(layout['speeds'])}")
        self.profile = SpeedProfile(**layout["speeds"][mode])
        self.mode = mode
        box = layout["workspace_box"]
        self.box = {k: tuple(v) for k, v in box.items()}
        self.regions = layout["regions"]         # name -> {box:{x:[],y:[]}, z_floor: float} or {box, keep_out: true}
        self.vision_jump_mm = layout["vision"]["max_correction_mm"]
        self.joints = joints                     # the arm's joints now; picks the elbow branch for the keep-out check

    # ---- region & floor -------------------------------------------------------
    def region_of(self, p: List[float]) -> str:
        for name, r in self.regions.items():
            if r.get("keep_out") or not r.get("box"):
                continue
            bx, by = r["box"]["x"], r["box"]["y"]
            if bx[0] <= p[0] <= bx[1] and by[0] <= p[1] <= by[1]:
                return name
        return "free"

    def z_floor(self, p: List[float]) -> float:
        r = self.region_of(p)
        return self.regions.get(r, {}).get("z_floor", self.box["z"][0])

    def check_target(self, p: List[float], label: str, allow_floor_regions=()):
        for axis, i in (("x", 0), ("y", 1), ("z", 2)):
            lo, hi = self.box[axis]
            if not (lo <= p[i] <= hi):
                raise SafetyViolation(f"{label}: {axis}={p[i]:.1f} outside workspace box {lo}..{hi}")
        region = self.region_of(p)
        floor = self.z_floor(p)
        if p[2] < floor and region not in allow_floor_regions:
            raise SafetyViolation(f"{label}: z={p[2]:.1f} below the {region} floor {floor:.1f}")
        self.check_keep_out(p, label)

    def check_segment(self, a: List[float], b: List[float], label: str, allow_floor_regions=()):
        """Linear moves are straight in Cartesian space; sample the segment against the floors."""
        n = max(2, int(max(abs(b[i] - a[i]) for i in range(3)) / 5.0))
        for k in range(n + 1):
            t = k / n
            p = [a[i] + t * (b[i] - a[i]) for i in range(3)]
            self.check_target(p + list(b[3:]), f"{label} (segment {k}/{n})", allow_floor_regions)
        self.check_no_swing(a, b, label)

    def check_joint_move(self, a: List[float], b: List[float], label: str):
        """Joint moves swing the elbow; they are not allowed to start or end inside a no-swing region,
        and the swing itself must stay out of every keep-out."""
        for p, where in ((a, "from"), (b, "to")):
            r = self.region_of(p)
            if self.regions.get(r, {}).get("no_swing"):
                raise SafetyViolation(f"{label}: joint move {where} inside the {r} zone; only vertical linear "
                                      "moves with fixed yaw are allowed there")
        if not self.keep_outs:
            return
        for branch in self._branches():
            ja = self._ik(a, branch, label)
            jb = self._ik(b, branch, label, ref=ja)
            self.check_joint_path(ja, jb, label)

    def check_no_swing(self, a: List[float], b: List[float], label: str):
        """Inside a no-swing region a linear move may only be vertical and may not change yaw."""
        for p in (a, b):
            r = self.region_of(p)
            if not self.regions.get(r, {}).get("no_swing"):
                continue
            lateral = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
            if lateral > self.regions[r].get("max_lateral_mm", 2.0):
                raise SafetyViolation(f"{label}: {lateral:.1f} mm lateral move inside the {r} zone")
            if len(a) > 5 and len(b) > 5 and abs(b[5] - a[5]) > self.regions[r].get("max_yaw_change_deg", 0.5):
                raise SafetyViolation(f"{label}: yaw change {abs(b[5] - a[5]):.1f} inside the {r} zone")
            return

    # ---- keep-out ---------------------------------------------------------------
    @property
    def keep_outs(self) -> dict:
        """Active keep-out boxes (a null box is not yet measured: inactive, preflight lists it)."""
        return {n: r["box"] for n, r in self.regions.items() if r.get("keep_out") and r.get("box")}

    def _branches(self):
        return (fk.elbow_sign(self.joints()),) if self.joints else (1, -1)     # branch unknown: both elbows must clear

    def _ik(self, p, branch, label, ref=None):
        try:
            refs = dict(j2_ref=ref[1], j3_ref=ref[2], j4_ref=ref[3]) if ref else {}
            return fk.ik(p, elbow=branch, **refs)
        except ValueError as e:
            raise SafetyViolation(f"{label}: {e}")

    def check_joint_path(self, ja: List[float], jb: List[float], label: str):
        """Joint-space interpolation from ja to jb: every sample's arm stays out of the keep-outs."""
        if not self.keep_outs:
            return
        n = max(1, int(max(abs(jb[i] - ja[i]) for i in (1, 2, 3)) / JOINT_SAMPLE_DEG))
        for k in range(n + 1):
            j = [ja[i] + k / n * (jb[i] - ja[i]) for i in range(4)]
            self.check_keep_out(fk.fk(j), f"{label} (joints {k}/{n})", joints=j)

    def check_keep_out(self, p: List[float], label: str, joints: Optional[List[float]] = None):
        """No part of the arm, base to elbow to tool point, inside a keep-out region. The elbow comes from the
        joints when known (waypoint replay), else from the FK model on the arm's current elbow branch."""
        boxes = self.keep_outs
        if not boxes:
            return
        configs = [joints] if joints is not None else [self._ik(p, b, label) for b in self._branches()]
        for j in configs:
            for part, q in self._arm_points(fk.elbow_xy(j), p):
                for name, box in boxes.items():
                    if all(box[ax][0] <= q[i] <= box[ax][1] for ax, i in (("x", 0), ("y", 1), ("z", 2)) if ax in box):
                        raise SafetyViolation(f"{label}: {part} at x={q[0]:.0f} y={q[1]:.0f} inside the {name} keep-out")

    @staticmethod
    def _arm_points(elbow, p):
        """Tool point, elbow, and the two links sampled every LINK_SAMPLE_MM, all at the pose's z."""
        base = (fk.MODEL["x0"], fk.MODEL["y0"])
        yield "tool point", (p[0], p[1], p[2])
        yield "elbow", (elbow[0], elbow[1], p[2])
        for part, a, b in (("upper arm", base, elbow), ("forearm", elbow, p[:2])):
            n = max(1, int(math.dist(a, b) / LINK_SAMPLE_MM))
            for k in range(1, n):
                t = k / n
                yield part, (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), p[2])
