"""
Forward / inverse kinematics of the Bioyond Nova S4 SCARA, fitted offline from logged data (see fit_fk.py).

JOINT ORDER AS REPORTED BY THE SDK (GetCurJointAsync / MoveAbsJointAsync), verified against 5,000 logged
joint/pose pairs (fit_fk.py):

    j1  column height            mm      z   = z_scale * j1 + z_offset           (z_scale = 1.000)
    j2  shoulder                 deg
    j3  elbow                    deg     j3 = -180 is the fully extended arm; the taught routes keep j3 in (-180, 0)
    j4  wrist yaw                deg     rz  = j2 + j3 + j4 (+ yaw_offset, fitted 0.00)

    x = x0 + l1*cos(j2 + a1) + l2*cos(j2 + j3 + a2) + lt*cos(rz + at)
    y = y0 + l1*sin(j2 + a1) + l2*sin(j2 + j3 + a2) + lt*sin(rz + at)

Fitted values: l1 = l2 = 200.0 mm, a1 = 0, a2 = 180 deg (hence the minus sign in the folded form below),
base at (x0, y0) = (126.0, 0.0) in the robot frame, tool offset lt = 0 (the reported pose IS the wrist axis).

The brief described the arm as "j1 shoulder, j2 elbow, j3 column, j4 wrist"; the data says otherwise (the first
joint tracks z in mm 1:1, the 2nd+3rd+4th sum to the reported yaw). Everything here uses the data's order.

Pose convention: [x, y, z, rx, ry, rz], mm and degrees, rx = ry = 0, rz = gripper yaw in (-180, 180].

Elbow configuration: `elbow=+1` is the branch used by every taught route (j3 in (-180, 0), i.e. e = j3 + 180 > 0);
`elbow=-1` is the mirror branch (j3 < -180), which is where the microscope seat taught on 2026-10-19 sits
(j3 = -187.2, 7 deg past full extension; the microscope is 0.8 mm inside the 400 mm reach).
"""
from __future__ import annotations

import json
import math
import os
from typing import List, Optional, Sequence

_HERE = os.path.dirname(os.path.abspath(__file__))

# defaults = values written to fk_model.json by fit_fk.py (kept here so the module works without the json)
MODEL = {
    "x0": 126.024, "y0": 0.034,
    "l1": 199.977, "a1_deg": 0.002,
    "l2": 199.975, "a2_deg": -179.997,
    "lt": 0.0, "at_deg": 0.0,
    "z_scale": 1.0, "z_offset": 38.598,
    "yaw_offset_deg": 0.0,
}


def load_model(path: Optional[str] = None) -> dict:
    path = path or os.path.join(_HERE, "fk_model.json")
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        MODEL.update({k: d[k] for k in MODEL if k in d})
    return MODEL


load_model()


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def fk(joints: Sequence[float], m: dict = None) -> List[float]:
    """joints = [j1_mm, j2_deg, j3_deg, j4_deg, ...] -> pose [x, y, z, 0, 0, rz]."""
    m = m or MODEL
    j1, j2, j3, j4 = (float(v) for v in joints[:4])
    t1 = math.radians(j2 + m["a1_deg"])
    t12 = math.radians(j2 + j3 + m["a2_deg"])
    rz = j2 + j3 + j4 + m["yaw_offset_deg"]
    ty = math.radians(rz + m["at_deg"])
    x = m["x0"] + m["l1"] * math.cos(t1) + m["l2"] * math.cos(t12) + m["lt"] * math.cos(ty)
    y = m["y0"] + m["l1"] * math.sin(t1) + m["l2"] * math.sin(t12) + m["lt"] * math.sin(ty)
    z = m["z_scale"] * j1 + m["z_offset"]
    return [x, y, z, 0.0, 0.0, wrap180(rz)]


def fk_xy_from_angles(j2: float, j3: float, m: dict = None):
    m = m or MODEL
    t1 = math.radians(j2 + m["a1_deg"]); t12 = math.radians(j2 + j3 + m["a2_deg"])
    return (m["x0"] + m["l1"] * math.cos(t1) + m["l2"] * math.cos(t12),
            m["y0"] + m["l1"] * math.sin(t1) + m["l2"] * math.sin(t12))


def elbow_sign(joints: Sequence[float]) -> int:
    """+1 for the taught branch (j3 wrapped into (-180, 0)), -1 for the mirror branch (wrapped j3 in (0, 180))."""
    return 1 if -180.0 < wrap180(float(joints[2])) < 0.0 else -1


def ik(pose: Sequence[float], elbow: int = 1, j4_ref: Optional[float] = None, j2_ref: Optional[float] = None,
       j3_ref: Optional[float] = None, m: dict = None) -> List[float]:
    """pose [x, y, z, rx, ry, rz] -> [j1, j2, j3, j4].

    elbow +1: taught branch, j3 returned in (-180, 0); -1: mirror branch, j3 returned in (0, 180) unless j3_ref
    says otherwise (the SDK does not wrap: the seat taught on 10-19 reads j3 = -187.2, the discarded trace +15).
    j4_ref / j2_ref / j3_ref pick the 360-degree representative closest to a reference (the taught routes have
    j4 between -32 and 184). Raises ValueError when out of reach.
    The tool offset lt is 0 in the fitted model; a non-zero lt is handled by subtracting it along the yaw first.
    """
    m = m or MODEL
    x, y, z = float(pose[0]), float(pose[1]), float(pose[2])
    rz = float(pose[5])
    if m["lt"]:
        ty = math.radians(rz + m["at_deg"])
        x -= m["lt"] * math.cos(ty); y -= m["lt"] * math.sin(ty)
    dx, dy = x - m["x0"], y - m["y0"]
    r2 = dx * dx + dy * dy
    l1, l2 = m["l1"], m["l2"]
    c = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)          # cos of the angle between the links (0 = extended)
    if c > 1.0 + 1e-9 or c < -1.0 - 1e-9:
        raise ValueError(f"out of reach: r={math.sqrt(r2):.2f} mm (max {l1 + l2:.2f}, min {abs(l1 - l2):.2f})")
    c = max(-1.0, min(1.0, c))
    # link angle phi = (j2 + j3 + a2) - (j2 + a1) = j3 + a2 - a1 ; cos(phi) = c ; two solutions +-acos
    phi = math.acos(c)
    # which sign of phi corresponds to the taught branch? taught: j3 in (-180,0) with a2 ~ -180 -> phi = j3 + a2 - a1
    # in (-360, -180) == (0, 180) mod 360 -> positive phi.
    phi = phi if elbow > 0 else -phi
    # j2 + a1 = atan2(dy, dx) - atan2(l2 sin phi, l1 + l2 cos phi)
    base = math.atan2(dy, dx) - math.atan2(l2 * math.sin(phi), l1 + l2 * math.cos(phi))
    j2 = math.degrees(base) - m["a1_deg"]
    j3 = math.degrees(phi) - m["a2_deg"] + m["a1_deg"]
    j3 = wrap180(j3)                       # taught branch lands in (-180, 0), mirror branch in (0, 180)
    if j3_ref is not None:
        j3 = _nearest_rep(j3, j3_ref)
    j2 = wrap180(j2)
    if j2_ref is not None:
        j2 = _nearest_rep(j2, j2_ref)
    j4 = rz - m["yaw_offset_deg"] - j2 - j3
    j4 = _nearest_rep(j4, j4_ref if j4_ref is not None else 0.0)
    j1 = (z - m["z_offset"]) / m["z_scale"]
    return [j1, j2, j3, j4]


def _nearest_rep(a: float, ref: float) -> float:
    return a + 360.0 * round((ref - a) / 360.0)


def reach(m: dict = None) -> float:
    m = m or MODEL
    return m["l1"] + m["l2"]


if __name__ == "__main__":
    # self-check against the taught microscope seat and the first waypoint of route B
    seat_j = [226.596, 93.866, -187.167, 183.635]
    seat_p = [124.033, 399.213, 265.194, 0, 0, 90.334]
    p = fk(seat_j)
    print("seat  fk err mm", [round(p[i] - seat_p[i], 3) for i in range(3)], "yaw err", round(wrap180(p[5] - seat_p[5]), 3))
    print("seat  ik elbow -1", [round(v, 3) for v in ik(seat_p, elbow=-1, j4_ref=180, j3_ref=-180)], "taught", seat_j)
    print("seat  ik elbow +1", [round(v, 3) for v in ik(seat_p, elbow=+1, j4_ref=180)])
    wp = [296.384, 89.198, -177.6461, 178.5941]
    wpp = [123.383, 399.907, 334.983, 0, 0, 90.146]
    p = fk(wp)
    print("routeB wp0 fk err mm", [round(p[i] - wpp[i], 3) for i in range(3)], "yaw err", round(wrap180(p[5] - wpp[5]), 3))
    print("routeB wp0 ik", [round(v, 3) for v in ik(fk(wp), elbow=+1, j4_ref=180)], "actual", wp)
