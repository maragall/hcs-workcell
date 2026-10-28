"""
The one capture plan: poses around wherever the arm is when the marker is in view.

Three heights, a 3x3 grid of x, y offsets at the first and the ring of eight at the other two,
the yaw cycling through four angles, every pose checked by the caller's `check` (the safe box),
the start pose last so the arm ends where it began. 26 poses at most. The z steps fix the focal
length and the principal point, the x, y spread puts the marker across the whole image for the
distortion, the yaws fix the camera's rotation on the hand.
"""
from __future__ import annotations

from typing import Callable, List, Optional, Sequence

STEP_MM = 20.0                       # x, y offsets: at 140 mm from the tag the picture holds the tag and about 35 mm of slack
RISE_MM = (0.0, 25.0, 50.0)          # the three heights, up from the start
YAW_DEG = (-8.0, -3.0, 3.0, 8.0)     # the camera sits 115 mm off the tool axis: a yaw swings the view 2 mm per degree
FOCAL_PX, HALF_VIEW_PX = 2632.0, 1232.0   # the down camera: focal length and half the picture's short side (vendor lens)
CAMERA_AHEAD_MM, TAG_HALF_DIAG_MM = 115.0, 28.3


def tag_in_view(pose: Sequence[float], here: Sequence[float], tag_xy: Sequence[float], dist_mm: float, slack: float = 0.85) -> bool:
    """Whether the whole 40 mm tag is in the picture at `pose`, with the camera looking straight down. `tag_xy` is where the tag is
    in the robot frame, `dist_mm` the camera's height above it when the wrist is at `here`."""
    import math
    yaw = math.radians(pose[5])
    look = (pose[0] + CAMERA_AHEAD_MM * math.cos(yaw), pose[1] + CAMERA_AHEAD_MM * math.sin(yaw))
    d = dist_mm + (pose[2] - here[2])
    return math.hypot(tag_xy[0] - look[0], tag_xy[1] - look[1]) + TAG_HALF_DIAG_MM <= slack * HALF_VIEW_PX / FOCAL_PX * d


def around(here: Sequence[float], check: Optional[Callable[[List[float]], None]] = None,
           step_mm: float = STEP_MM, rises=RISE_MM, yaws=YAW_DEG) -> List[List[float]]:
    """Poses (x y z rx ry rz) for the plan; `check(pose)` raises to drop one."""
    here = [float(v) for v in here]
    out, k = [], 0
    for h, dz in enumerate(rises):
        for dx in (-step_mm, 0.0, step_mm):
            for dy in (-step_mm, 0.0, step_mm):
                if h and dx == 0.0 == dy:
                    continue
                yaw = yaws[k % len(yaws)]
                k += 1
                p = [here[0] + dx, here[1] + dy, here[2] + dz, here[3], here[4], here[5] + yaw]
                try:
                    if check:
                        check(p)
                except Exception:
                    continue
                out.append(p)
    return out + [here]
