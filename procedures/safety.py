"""
Safety layer: speed profiles, z-floors per region, sanity box, and the vision sanity guard.
All numbers come from layout.json; nothing here is hard-coded to this cell.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List


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
    def __init__(self, layout: dict, mode: str):
        self.layout = layout
        if mode not in layout["speeds"]:
            raise SafetyViolation(f"unknown speed mode {mode}; have {list(layout['speeds'])}")
        self.profile = SpeedProfile(**layout["speeds"][mode])
        self.mode = mode
        box = layout["workspace_box"]
        self.box = {k: tuple(v) for k, v in box.items()}
        self.regions = layout["regions"]         # name -> {box:{x:[],y:[]}, z_floor: float}
        self.vision_jump_mm = layout["vision"]["max_correction_mm"]

    # ---- region & floor -------------------------------------------------------
    def region_of(self, p: List[float]) -> str:
        for name, r in self.regions.items():
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

    def check_segment(self, a: List[float], b: List[float], label: str, allow_floor_regions=()):
        """Linear moves are straight in Cartesian space; sample the segment against the floors."""
        n = max(2, int(max(abs(b[i] - a[i]) for i in range(3)) / 5.0))
        for k in range(n + 1):
            t = k / n
            p = [a[i] + t * (b[i] - a[i]) for i in range(3)]
            self.check_target(p + list(b[3:]), f"{label} (segment {k}/{n})", allow_floor_regions)
        self.check_no_swing(a, b, label)

    def check_joint_move(self, a: List[float], b: List[float], label: str):
        """Joint moves swing the elbow; they are not allowed to start or end inside a no-swing region."""
        for p, where in ((a, "from"), (b, "to")):
            r = self.region_of(p)
            if self.regions.get(r, {}).get("no_swing"):
                raise SafetyViolation(f"{label}: joint move {where} inside the {r} zone; only vertical linear "
                                      "moves with fixed yaw are allowed there")

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

