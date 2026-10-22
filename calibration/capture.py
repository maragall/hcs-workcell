"""
Drive the arm through a vendor capture plan and save what the vendor's tool saves.

A plan (vendor_data/CalibrationPlans/<camera>.json) has InitialToolInBase (x y z rx ry rz,
mm and deg) and Offsets, one per capture. An offset composes in the frame of the initial
pose: its x, y are rotated by the initial rz and added, z and rz add. That is what the vendor's
HandEye poses show (2 mm rms against the plan, the joint-space reading is 26 mm off; the
residual is the plan file being re-saved from a slightly different start).

Output, like the vendor's folders: <kind>_NNN.png (full-resolution frame, NNN from 001) and for
hand-eye captures pose_NNN.txt with the arm's pose after the move, "x,y,z,rx,ry,rz" to 3
decimals, no newline.
"""
from __future__ import annotations

import json
import os
from typing import List, Sequence

import cv2
import numpy as np

DEFAULT_SPEED = 0.1


def load_plan(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        plan = json.load(f)
    if "InitialToolInBase" not in plan or "Offsets" not in plan:
        raise ValueError(f"{path}: not a capture plan")
    return plan


def compose(initial: Sequence[float], offset: Sequence[float]) -> List[float]:
    """Target pose for one offset of the plan (mm, deg)."""
    x0, y0, z0, rx0, ry0, rz0 = (float(v) for v in initial)
    dx, dy, dz, drx, dry, drz = (float(v) for v in offset)
    c, s = np.cos(np.radians(rz0)), np.sin(np.radians(rz0))
    rz = (rz0 + drz + 180.0) % 360.0 - 180.0
    return [x0 + c * dx - s * dy, y0 + s * dx + c * dy, z0 + dz, rx0 + drx, ry0 + dry, rz]


def pose_text(pose: Sequence[float]) -> str:
    return ",".join(f"{float(v):.3f}" for v in pose)


def run_plan(plan: dict, camera: str, robot, box, out_dir: str, kind: str = "handeye", led: int = 0) -> List[str]:
    """Move, grab, save, for every offset. Returns the image paths. Raises on the first failed move."""
    if kind not in ("intrinsic", "handeye"):
        raise ValueError(kind)
    os.makedirs(out_dir, exist_ok=True)
    robot.set_speed(float(plan.get("Speed", DEFAULT_SPEED)))
    saved = []
    for i, offset in enumerate(plan["Offsets"], start=1):
        robot.move_linear(compose(plan["InitialToolInBase"], offset))
        frame = box.frame_full(camera, led)
        img = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"capture {i}: the box returned no decodable frame")
        path = os.path.join(out_dir, f"{kind}_{i:03d}.png")
        cv2.imwrite(path, img)
        if kind == "handeye":
            with open(os.path.join(out_dir, f"pose_{i:03d}.txt"), "w", encoding="utf-8", newline="") as f:
                f.write(pose_text(robot.pose()))
        saved.append(path)
    return saved
