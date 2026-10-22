"""
Vendor manual section 5, teach-free positions, with the box's own API (devices/vision_box.py):
a teach position (where the gripper goes), a trigger position (where the camera looks at the
marker) and the relation between them. Same calls as cli.py record-pose / record-trigger.

mode is the vendor's pick method: "wide" = fingers on the long edges, "long" = on the short.
"""
from __future__ import annotations

from typing import Dict, Sequence

MODES = ("wide", "long")


def add(box, mode: str, teach_pose: Sequence[float], trigger_pose: Sequence[float], camera: str, name: str,
        workspace: str = "autoteach", marker_mm: float = 150, led: int = 0, marker: str = "aruco") -> Dict[str, str]:
    """Stores <name>-teach, <name>-trigger and <name>-rel, replacing same-named entries."""
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    teach, trigger, relation = f"{name}-teach", f"{name}-trigger", f"{name}-rel"
    existing = {(p["type"], p["name"]) for p in box.positions(workspace)}
    if ("teach", teach) in existing:
        box.delete_position(teach, "teach", workspace)
    if ("trigger", trigger) in existing:
        box.delete_position(trigger, "trigger", workspace)
    box.add_teach(teach, list(teach_pose), mode, workspace)
    box.add_trigger(trigger, list(trigger_pose), camera, marker, marker_mm, mode, workspace, led)
    box.add_relation(trigger, teach, relation, mode, workspace)
    return {"teach": teach, "trigger": trigger, "relation": relation}
