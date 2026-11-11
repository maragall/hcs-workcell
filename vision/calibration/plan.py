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

STEPS_MM = (20.0, 30.0, 40.0, 50.0)  # x, y offsets per height: the higher, the more of the table the picture holds around the tag
STEP_MM = STEPS_MM[0]
RISE_MM = (0.0, 35.0, 70.0, 105.0)   # the heights, up from the start (the tag 140 mm below the camera at the first)
YAW_DEG = (-15.0, -5.0, 5.0, 15.0)   # the camera sits 115 mm off the tool axis: a yaw swings the view 2 mm per degree; tag_in_view drops the ones that lose the tag
TAG_YAW_DEG = (0.0, 10.0, 20.0, 30.0)   # for around_tag: turning this way swings the wrist away from the hotel (the other way puts the claw edge past x 400)
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


def around_tag(here: Sequence[float], tag_xy: Sequence[float], dist_mm: float, rises=RISE_MM, yaws=TAG_YAW_DEG,
               ring_frac: float = 0.7) -> List[List[float]]:
    """Poses where the camera's look point stays on the tag: at every height the centre and a ring of eight offsets (radius a share of
    what the picture can hold at that height), each with a yaw from the cycle, and the wrist put wherever that look point needs it
    (the camera is CAMERA_AHEAD_MM ahead of the wrist, so a yaw swings the wrist, not the view). The start pose is last."""
    import math
    here = [float(v) for v in here]
    out, k = [], 0
    for h, dz in enumerate(rises):
        d = dist_mm + dz
        r = max(0.0, ring_frac * (0.85 * HALF_VIEW_PX / FOCAL_PX * d - TAG_HALF_DIAG_MM))
        offs = [(0.0, 0.0)] + [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in range(0, 360, 45)]
        for ox, oy in offs:
            yaw = here[5] + yaws[k % len(yaws)]
            k += 1
            lx, ly = tag_xy[0] + ox, tag_xy[1] + oy
            out.append([lx - CAMERA_AHEAD_MM * math.cos(math.radians(yaw)), ly - CAMERA_AHEAD_MM * math.sin(math.radians(yaw)),
                        here[2] + dz, here[3], here[4], yaw])
    return out + [here]


def around(here: Sequence[float], check: Optional[Callable[[List[float]], None]] = None,
           step_mm: float = STEP_MM, rises=RISE_MM, yaws=YAW_DEG) -> List[List[float]]:
    """Poses (x y z rx ry rz) for the plan; `check(pose)` raises to drop one."""
    here = [float(v) for v in here]
    out, k = [], 0
    for h, dz in enumerate(rises):
        step = STEPS_MM[h] if (step_mm == STEP_MM and h < len(STEPS_MM)) else step_mm
        for dx in (-step, 0.0, step):
            for dy in (-step, 0.0, step):
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
