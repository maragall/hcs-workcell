"""
python3 -m calibration.validate [vendor_data]

The vendor's captures show their old target, so the check is synthetic: the marker is rendered
through the vendor's CamParam.dat and CamInTool.dat (ground truth) along our plan around one of
their arm poses, with pixel noise; the corners are detected on those images, the joint solve runs
from the vendor's initial CamParam, and the result is set against the vendor's files.
Acceptance: Focus, Sx, Cx, Cy within 0.1 %; CamInTool within 0.05 deg and 0.1 mm.
"""
from __future__ import annotations

import os
import sys
import tempfile

import cv2
import numpy as np

from calibration import capture, halcon, plan, solve, synth

START_POSE = [436.653, 108.688, 300.861, 0.0, 0.0, -31.918]    # the vendor's first hand-eye capture pose
MARKER_M = 0.040
DISTANCE_M = 0.20
NOISE_PX = 0.3


class _Arm:
    def __init__(self, pose):
        self._pose = list(pose)

    def set_speed(self, s):
        pass

    def move_linear(self, p):
        self._pose = list(p)

    def pose(self):
        return list(self._pose)


def synthetic_folder(cam: halcon.CamParam, camintool: halcon.Pose, out_dir: str, start=START_POSE,
                     noise_px: float = NOISE_PX) -> synth.Scene:
    """Captures along the plan, the marker at base z = 0 (the vendor's gauge)."""
    T_bm = synth.facing(start, camintool, DISTANCE_M)
    start = list(start)
    start[2] -= T_bm[2, 3] * 1e3                   # lift the start so the marker sits at z = 0
    T_bm = synth.facing(start, camintool, DISTANCE_M)
    scene = synth.Scene(cam, camintool, T_bm, MARKER_M, noise_px)
    arm = _Arm(start)
    capture.run_plan(plan.around(start), "cam", arm, synth.FakeBox(scene, arm), out_dir)
    return scene


def compare(res: solve.Result, vcam: halcon.CamParam, vpose: halcon.Pose) -> bool:
    ok = True
    print(f"joint solve: {res.images} captures, skipped {list(res.skipped)}, {res.reprojection_px:.3f} px, "
          f"marker error {res.mean_error_m * 1e3:.3f} mm, unobservable {list(res.unobservable)}")
    print(f"  {'':8s} {'ours':>18s} {'vendor':>18s} {'diff':>10s}")
    for name in ("focus", "sx", "cx", "cy"):
        o, v = getattr(res.camparam, name), getattr(vcam, name)
        d = 100.0 * (o - v) / v
        flag = "" if abs(d) <= 0.1 else "  FAIL"
        ok &= not flag
        print(f"  {name.capitalize():8s} {o:18.10g} {v:18.10g} {d:+9.4f}%{flag}")
    for i, (o, v) in enumerate(zip(res.camparam.poly, vcam.poly)):
        print(f"  Poly{i + 1:<4d} {o:18.6g} {v:18.6g}")
    T, Tv = res.camintool.matrix, vpose.matrix
    drot = float(np.degrees(np.linalg.norm(cv2.Rodrigues(T[:3, :3].T @ Tv[:3, :3])[0])))
    dt = (np.array(res.camintool.t) - np.array(vpose.t)) * 1e3
    print(f"  r (deg)  ours {np.round(res.camintool.r, 4)}  vendor {np.round(vpose.r, 4)}  diff {drot:.4f} deg"
          + ("" if drot <= 0.05 else "  FAIL"))
    print(f"  t (m)    ours {np.round(res.camintool.t, 5)}  vendor {np.round(vpose.t, 5)}  diff mm {np.round(dt, 3)}"
          + ("" if np.abs(dt).max() <= 0.1 else "  FAIL"))
    ok &= drot <= 0.05 and np.abs(dt).max() <= 0.1
    print("ACCEPTED" if ok else "NOT ACCEPTED")
    return ok


def main(root: str = "vendor_data") -> int:
    cal = os.path.join(root, "CalibrationData")
    vcam = halcon.CamParam.read(os.path.join(cal, "intrinsic", "CamParam.dat"))
    vpose = halcon.Pose.read(os.path.join(cal, "HandEye", "CamInTool.dat"))
    start = halcon.CamParam.read(os.path.join(cal, "IniIntrinsic", "CamParam.dat"))
    with tempfile.TemporaryDirectory() as d:
        synthetic_folder(vcam, vpose, d)
        res = solve.run(d, start, MARKER_M)
    return 0 if compare(res, vcam, vpose) else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:2]))
