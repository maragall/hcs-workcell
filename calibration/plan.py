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

STEP_MM = 40.0                       # x, y offsets; with the yaw swing this keeps the marker in view at 0.2 m
RISE_MM = (0.0, 25.0, 50.0)          # the three heights, up from the start
YAW_DEG = (-15.0, -5.0, 5.0, 15.0)   # the camera sits 100 mm off the tool axis: larger yaws swing the marker out


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
