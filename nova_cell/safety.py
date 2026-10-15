"""
Safety layer: speed profiles, z-floors per region, sanity box, and the vision sanity guard.
All numbers come from layout.json; nothing here is hard-coded to this cell.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


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

    # ---- vision -----------------------------------------------------------------
    def check_vision_correction(self, d: List[float], label: str):
        mag = (d[0] ** 2 + d[1] ** 2 + d[2] ** 2) ** 0.5
        if mag > self.vision_jump_mm:
            raise SafetyViolation(f"{label}: vision correction {mag:.1f} mm exceeds limit {self.vision_jump_mm} mm")
        return mag
