"""
python3 -m calibration.validate [vendor_data]

Runs plate detection, the intrinsic fit and the hand-eye solve on the vendor's own captures and
prints our numbers next to the vendor's CamParam.dat and CamInTool.dat, with the acceptance
margins: Focus, Cx, Cy within 0.5 %, reprojection at or below the vendor's 0.162 px,
CamInTool translation within 1 mm and rotation within 0.5 deg of the vendor's.

The intrinsic fit uses the plan's 13 captures (vendor_data/CalibrationPlans/scan_camera.json has
13 offsets); intrinsic_014..017 are a later session with the plate moved to a table, and the
hand-eye solve rejects them on its own by their plate position.
"""
from __future__ import annotations

import json
import os
import sys

import cv2
import numpy as np

from calibration import halcon, handeye, intrinsics

VENDOR_REPROJECTION_PX = 0.162      # from the vendor tool's log
VENDOR_HANDEYE_ERROR_MM = 0.26


def pct(a, b):
    return 100.0 * (a - b) / b


def main(root: str = "vendor_data") -> int:
    cal = os.path.join(root, "CalibrationData")
    with open(os.path.join(root, "CalibrationPlans", "scan_camera.json")) as f:
        plan_n = len(json.load(f)["Offsets"])
    vcam = halcon.CamParam.read(os.path.join(cal, "intrinsic", "CamParam.dat"))
    vpose = halcon.Pose.read(os.path.join(cal, "HandEye", "CamInTool.dat"))

    cam = intrinsics.solve(os.path.join(cal, "intrinsic"), limit=plan_n)
    ok = True
    print(f"intrinsics ({cam.notes}; model {cam.model})")
    print(f"  {'':8s} {'ours':>18s} {'vendor':>18s} {'diff':>10s}")
    for name, o, v in (("Focus", cam.focus, vcam.focus), ("Sx", cam.sx, vcam.sx), ("Cx", cam.cx, vcam.cx),
                       ("Cy", cam.cy, vcam.cy)):
        d = pct(o, v)
        flag = "" if abs(d) <= 0.5 or name == "Sx" else "  FAIL"
        ok &= not flag
        print(f"  {name:8s} {o:18.10g} {v:18.10g} {d:+9.3f}%{flag}")
    for i, (o, v) in enumerate(zip(cam.poly, vcam.poly)):
        print(f"  Poly{i + 1:<4d} {o:18.6g} {v:18.6g}")
    flag = "" if cam.reprojection_px <= VENDOR_REPROJECTION_PX else "  FAIL"
    ok &= not flag
    print(f"  reproj   {cam.reprojection_px:18.3f} {VENDOR_REPROJECTION_PX:18.3f} px{flag}")

    res = handeye.solve(os.path.join(cal, "HandEye"), cam)
    T, Tv = res.matrix, vpose.matrix
    drot = float(np.degrees(np.linalg.norm(cv2.Rodrigues(T[:3, :3].T @ Tv[:3, :3])[0])))
    dt = (np.array(res.pose.t) - np.array(vpose.t)) * 1e3
    print(f"hand-eye ({res.images} captures used, dropped {list(res.dropped)}; unobservable {list(res.unobservable)})")
    print(f"  r (deg)  ours {np.round(res.pose.r, 4)}  vendor {np.round(vpose.r, 4)}  rotation diff {drot:.3f} deg"
          + ("" if drot <= 0.5 else "  FAIL"))
    print(f"  t (m)    ours {np.round(res.pose.t, 5)}  vendor {np.round(vpose.t, 5)}  diff mm {np.round(dt, 3)}"
          + ("" if np.abs(dt).max() <= 1.0 else "  FAIL"))
    print(f"  mean err {res.mean_error_m * 1e3:.3f} mm  vendor {VENDOR_HANDEYE_ERROR_MM} mm;  plate in base "
          f"{np.round(res.plate_in_base.t, 4)} m")
    ok &= drot <= 0.5 and np.abs(dt).max() <= 1.0
    print("ACCEPTED" if ok else "NOT ACCEPTED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:2]))
