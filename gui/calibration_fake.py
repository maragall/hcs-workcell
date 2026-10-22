"""
The `calibration` package's interface, faked: what the Cameras tab calls until the real package (branch
`calibration`) lands, and always on --sim. Pictures come from the frame buffer; the numbers are well inside the
vendor's limits so the tab can be clicked through.

    calibration.capture.run_plan(plan, camera, robot, box, out_dir)
    calibration.intrinsics.solve(folder) -> CamParam        .reprojection_px  .write(path)
    calibration.handeye.solve(folder, camparam) -> CamInTool .mean_error_m    .write(path)
    calibration.upload.push(box_host, camera, camparam_path, camintool_path); calibration.upload.reboot(box_host)
    calibration.verify.check(box, camera) -> delta_mm
    calibration.teach_free.add(box, mode, teach_pose, trigger_pose)
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass
from types import SimpleNamespace


@dataclass
class CamParam:
    reprojection_px: float

    def write(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f)


@dataclass
class CamInTool:
    mean_error_m: float

    def write(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f)


def _run_plan(plan, camera, robot, box, out_dir):
    """One picture per pose of the plan (or six where the arm is, when the user moves it by hand)."""
    os.makedirs(out_dir, exist_ok=True)
    poses = plan if plan is not None else [robot.pose()] * 6
    for i, p in enumerate(poses):
        if plan is not None:
            robot.move_linear(p)
        with open(os.path.join(out_dir, f"{i:03d}.jpg"), "wb") as f:
            f.write(box.frame(camera, 1280, 720, 85, None, 0))
        with open(os.path.join(out_dir, f"{i:03d}.json"), "w", encoding="utf-8") as f:
            json.dump({"pose": robot.pose(), "t": time.time()}, f)
    return len(poses)


capture = SimpleNamespace(run_plan=_run_plan)
intrinsics = SimpleNamespace(solve=lambda folder: CamParam(0.31))
handeye = SimpleNamespace(solve=lambda folder, camparam: CamInTool(0.0006))
upload = SimpleNamespace(push=lambda box_host, camera, camparam_path, camintool_path: None, reboot=lambda box_host: None)
verify = SimpleNamespace(check=lambda box, camera: 0.4)
teach_free = SimpleNamespace(add=lambda box, mode, teach_pose, trigger_pose: None)
