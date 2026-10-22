"""
The positions file: every pose the arm goes to, plus floors, speeds, limits, routes and plate types.
Data, not code. It is `layout.json` at the repo root (the architecture figure calls it /positions.yaml;
nothing in the code depends on the name).
"""
from __future__ import annotations

import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, "layout.json")
RUNS = os.path.join(ROOT, "runs")
REFS = os.path.join(ROOT, "reference_images")


def load(path: str = PATH) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save(layout: dict, path: str = PATH) -> None:
    """Atomic: the teach commands call this while the arm is live."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(layout, f, indent=2)
    os.replace(tmp, path)


def update_fixture(fixture_name: str, path: str = PATH, **keys) -> None:
    """Write `keys` into fixtures.<fixture_name> on disk, leaving everything else as the file has it.
    fixture_name may be dotted (polar.pose_at_0). Used when the cell learns a pose or a grasp width."""
    layout = load(path)
    node = layout["fixtures"]
    for part in fixture_name.split("."):
        node = node[part]
    node.update(keys)
    save(layout, path)
