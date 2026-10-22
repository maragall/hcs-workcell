"""What must be taught, measured and photographed before the arm moves on its own."""
from __future__ import annotations

import os
from typing import Iterable, List


def _walk(node, path=""):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from _walk(v, f"{path}.{k}" if path else k)
    else:
        yield path, node


def preflight(layout: dict, ref_dir: str, scopes: Iterable[str] = ("microscope",)) -> List[str]:
    """Problems that stop a run: untaught poses, unmeasured widths, missing reference images, open _todo keys."""
    problems = [p for p, v in _walk(layout) if p.endswith("_todo")]
    F = layout["fixtures"]
    poses = [(s, F.get(s, {}).get("pose")) for s in scopes]
    poses += [("polar.pose_at_0", F["polar"]["pose_at_0"].get("pose")),
              ("polar.pose_at_90", F["polar"]["pose_at_90"].get("pose"))]
    problems += [f"pose not taught: {name}" for name, pose in poses if not pose]
    problems += [f"grip width not measured: {g}" for g, w in layout["gripper"]["closed"].items() if not w]
    problems += [f"open width not set: {g}" for g, w in layout["gripper"]["open"].items() if not w]
    hotel = F["hotel"]
    if hotel.get("headroom_mm") is not None and hotel["lift_mm"] >= hotel["headroom_mm"]:
        problems.append(f"hotel lift {hotel['lift_mm']} mm is not below the headroom {hotel['headroom_mm']} mm")
    if layout["polar"]["ext_axis"].get("zero_reading") is None:
        problems.append("polar zero reference not set")
    problems += [f"trajectory not taught: {name}" for name, t in layout["trajectories"].items() if not t]
    for s in scopes:
        box = layout["regions"].get(s, {}).get("box")
        if not box or box["x"][0] >= box["x"][1] or box["y"][0] >= box["y"][1]:
            problems.append(f"regions.{s} box not set")
    need = layout["vision"]["lower_check"]["min_refs"]
    gates = [(s, state) for s in scopes for state in ("plate_present", "empty")]
    gates += [("polar@0", "plate_present"), ("polar@0", "empty"), ("polar@90", "plate_present"), ("polar@90", "empty"),
              ("gripper", "short_edges"), ("gripper", "long_edges")]
    for gate, state in gates:
        d = os.path.join(ref_dir, gate, state)
        n = len(os.listdir(d)) if os.path.isdir(d) else 0
        if n < need:
            problems.append(f"reference images {gate}/{state}: {n}/{need}")
    return problems
