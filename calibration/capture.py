"""
Drive the arm through the plan and keep a picture and the arm's pose at every stop.

Output: <out_dir>/NNN.png (the full-resolution frame) and NNN.json {"pose": [x y z rx ry rz]}
with the pose the arm reports after the move. `poses` None means the operator moves the arm by
hand: one picture is taken where it is, for every call.
"""
from __future__ import annotations

import json
import os
import time
from typing import Optional, Sequence

import cv2
import numpy as np

DEFAULT_SPEED = 0.1
SETTLE_S = 1.0                  # after a move the arm still creeps a few hundredths of a mm: 0.5 px at 19 px/mm (measured 2026-10-28)


def run_plan(poses: Optional[Sequence[Sequence[float]]], camera: str, robot, box, out_dir: str,
             led: int = 0, speed: float = DEFAULT_SPEED, stop=None) -> int:
    """Returns how many captures the folder holds. Raises on the first failed move."""
    os.makedirs(out_dir, exist_ok=True)
    have = len([f for f in os.listdir(out_dir) if f.endswith(".png")])
    if poses is None:
        _grab(camera, robot, box, out_dir, have + 1, led)
        return have + 1
    robot.set_speed(speed)
    for i, p in enumerate(poses, start=have + 1):
        if stop is not None and stop.is_set():
            return i - 1
        robot.move_linear([float(v) for v in p])
        time.sleep(SETTLE_S)
        _grab(camera, robot, box, out_dir, i, led)
    return have + len(poses)


def _grab(camera, robot, box, out_dir, i, led) -> str:
    frame = box.frame_full(camera, led)
    img = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"capture {i}: the box returned no decodable frame")
    path = os.path.join(out_dir, f"{i:03d}.png")
    cv2.imwrite(path, img)
    with open(os.path.join(out_dir, f"{i:03d}.json"), "w", encoding="utf-8") as f:
        json.dump({"pose": [float(v) for v in robot.pose()]}, f)
    return path
