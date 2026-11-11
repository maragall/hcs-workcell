"""
Check the uploaded calibration with the box's own marker reading.

The marker tab is fixed in the cell. With a correct CamParam + CamInTool the box reports the
marker at the same place in the robot frame from any tool pose, so: read it from pose A, move
to pose B, read again; the distance between the two robot-frame positions is the error. A wrong
camera-in-tool makes it move with the arm, and the discrepancy grows with the move. The vendor's
tool prints the same figure as "Verification passed (ΔT = ... mm)".
"""
from __future__ import annotations

from typing import Sequence

import numpy as np


def check(box, camera: str, robot, pose_a: Sequence[float], pose_b: Sequence[float], marker_id: int = 1,
          led: int = 0) -> float:
    """Returns delta_mm. Both poses must see the marker; the box raises VisionError otherwise."""
    robot.move_linear(list(pose_a))
    a = box.aruco_pose(camera, marker_id, None, led)
    robot.move_linear(list(pose_b))
    b = box.aruco_pose(camera, marker_id, None, led)
    if a.frame != b.frame:
        raise RuntimeError(f"marker reported in different frames: {a.frame!r} and {b.frame!r}")
    return float(np.linalg.norm(np.array(a.target[:3]) - np.array(b.target[:3])))
