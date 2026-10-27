"""
The `calibration` package's interface, faked: what the setup calls on --sim. Pictures come from the frame
buffer; the numbers are well inside the vendor's limits so the setup can run through.

    calibration.plan.around(here, check) -> poses
    calibration.capture.run_plan(poses, camera, robot, box, out_dir, led, stop=) -> n
    calibration.halcon.CamParam.read(path)
    calibration.solve.run(folder, start, marker_size_m) -> Result   .reprojection_px .mean_error_m .images .write(dir)
    calibration.upload.credentials(layout); .push(host, camera, camparam_path, camintool_path, creds); .reboot(host, creds)
    calibration.verify.check(box, camera, robot, pose_a, pose_b, marker_id, led) -> delta_mm
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from types import SimpleNamespace


@dataclass
class Result:
    reprojection_px: float
    mean_error_m: float
    images: int

    def write(self, out_dir: str) -> None:
        os.makedirs(out_dir, exist_ok=True)
        for name in ("CamParam.dat", "CamInTool.dat"):
            with open(os.path.join(out_dir, name), "w", encoding="utf-8") as f:
                f.write(f"# fake {name}\n")


def _around(here, check=None):
    out = []
    for dx, dy, dz in ((-20, 0, 0), (20, 0, 0), (0, -20, 0), (0, 20, 0), (0, 0, 20), (20, 20, 20)):
        p = [here[0] + dx, here[1] + dy, here[2] + dz] + list(here[3:])
        try:
            if check:
                check(p)
        except Exception:
            continue
        out.append(p)
    return out + [list(here)]


def _run_plan(poses, camera, robot, box, out_dir, led=0, stop=None):
    """One picture per pose (or one where the arm is, when the user moves it by hand)."""
    os.makedirs(out_dir, exist_ok=True)
    have = len([f for f in os.listdir(out_dir) if f.endswith(".jpg")])
    for i, p in enumerate(poses if poses is not None else [robot.pose()], start=have + 1):
        if poses is not None:
            robot.move_linear(p)
        with open(os.path.join(out_dir, f"{i:03d}.jpg"), "wb") as f:
            f.write(box.frame(camera, 1280, 720, 85, None, led))
        with open(os.path.join(out_dir, f"{i:03d}.json"), "w", encoding="utf-8") as f:
            json.dump({"pose": robot.pose()}, f)
    return have + (len(poses) if poses is not None else 1)


def _run(folder, start, marker_size_m):
    n = len([f for f in os.listdir(folder) if f.endswith(".jpg")])
    return Result(0.31, 0.0006, n)


plan = SimpleNamespace(around=_around)
capture = SimpleNamespace(run_plan=_run_plan)
halcon = SimpleNamespace(CamParam=SimpleNamespace(read=lambda path: None))
solve = SimpleNamespace(run=_run)
upload = SimpleNamespace(credentials=lambda layout: {}, push=lambda host, camera, a, b, creds: None, reboot=lambda host, creds: None)
verify = SimpleNamespace(check=lambda box, camera, robot, pose_a, pose_b, marker_id=1, led=0: 0.4)
