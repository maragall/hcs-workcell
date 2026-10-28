"""
Wrist poses computed from the manufacturer CAD (hardware.json `cad`) instead of taught by hand.

The CAD gives where every plate sits on the platform. The pose the controller takes is the wrist axis, so a pose is
the plate's position turned into the robot frame and moved back along the claw. Two things are not in the CAD and
are fitted once from the poses already taught: the small rotation and shift between the CAD's axes and the robot's
(`fit`), and, per grip, how far the wrist stands behind the plate centre. Nothing here moves the arm.

    python cli.py cad-poses             how far each CAD pose is from the taught one
    fixtures.hotel.pose_source = "cad"  Hotel.hotel_slot_pose answers with the CAD pose (default "taught")
"""
from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional

from procedures import positions


def load_hardware(path: Optional[str] = None) -> dict:
    path = path or os.path.join(os.path.dirname(positions.PATH), "hardware.json")
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _rot(theta_deg: float, x: float, y: float):
    c, s = math.cos(math.radians(theta_deg)), math.sin(math.radians(theta_deg))
    return c * x - s * y, s * x + c * y


class CadPoses:
    def __init__(self, layout: dict, hardware: Optional[dict] = None):
        self.cad = (hardware or load_hardware())["cad"]
        h = self.cad["hotel"]
        self.cx = (h["x_r"][0] + h["x_r"][1]) / 2          # the plate centre across the hotel's depth
        self.cols = dict(h["columns_y_r"])
        self.pitch = float(h["row_pitch_mm"])
        self.taught = layout["fixtures"]["hotel"]["taught"]
        self.fit = self.fit_from_taught()

    def _taught_rows(self):
        for slot, pose in self.taught.items():
            if slot[:1] in self.cols and slot[1:].isdigit():
                yield slot, pose

    def fit_from_taught(self) -> Dict[str, float]:
        """Rotation and shift of the CAD's x, y into the robot's, and the wrist height of row 1, from the hotel poses
        taught by hand (least squares, 2D Procrustes). The residuals are what `compare` prints."""
        pts = [(self.cx, self.cols[s[0]], p[0], p[1], p[2] - self.pitch * (int(s[1:]) - 1)) for s, p in self._taught_rows()]
        n = len(pts)
        mx = sum(p[0] for p in pts) / n
        my = sum(p[1] for p in pts) / n
        wx = sum(p[2] for p in pts) / n
        wy = sum(p[3] for p in pts) / n
        a = sum((p[0] - mx) * (p[2] - wx) + (p[1] - my) * (p[3] - wy) for p in pts)
        b = sum((p[0] - mx) * (p[3] - wy) - (p[1] - my) * (p[2] - wx) for p in pts)
        theta = math.degrees(math.atan2(b, a)) if abs(a) + abs(b) > 1e-9 else 0.0
        rx, ry = _rot(theta, mx, my)
        return {"theta_deg": theta, "x": wx - rx, "y": wy - ry, "z_row1": sum(p[4] for p in pts) / n}

    def hotel_pose(self, slot: str, yaw: Optional[float] = None) -> List[float]:
        """[x, y, z, rx, ry, rz] of the wrist for a hotel slot, claw square to the hotel (yaw = the frame's rotation)."""
        col, row = slot[:1], int(slot[1:])
        f = self.fit
        x, y = _rot(f["theta_deg"], self.cx, self.cols[col])
        return [x + f["x"], y + f["y"], f["z_row1"] + self.pitch * (row - 1), 0.0, 0.0, f["theta_deg"] if yaw is None else yaw]

    def compare(self) -> List[dict]:
        """Per taught hotel slot: the CAD pose and the difference (taught minus CAD), mm and degrees."""
        out = []
        for slot, p in self._taught_rows():
            c = self.hotel_pose(slot)
            out.append({"slot": slot, "taught": [round(v, 2) for v in p[:3]] + [round(p[5], 2)],
                        "cad": [round(v, 2) for v in c[:3]] + [round(c[5], 2)],
                        "diff": [round(p[0] - c[0], 2), round(p[1] - c[1], 2), round(p[2] - c[2], 2), round(p[5] - c[5], 2)]})
        return out

    def rms_mm(self) -> float:
        d = [r["diff"][:3] for r in self.compare()]
        return math.sqrt(sum(v * v for row in d for v in row) / (3 * len(d)))
