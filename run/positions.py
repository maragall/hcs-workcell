"""
The positions file: every pose the arm goes to, plus floors, speeds, limits, routes and plate types.
Data, not code. It is `layout.json` at the repo root (the architecture figure calls it /positions.yaml;
nothing in the code depends on the name).
"""
from __future__ import annotations

import json
import os
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, "layout.json")
RUNS = os.path.join(ROOT, "runs")
REFS = os.path.join(ROOT, "reference_images")


def load(path: str = PATH) -> dict:
    for attempt in range(50):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except PermissionError:          # Windows: a writer is replacing the file this instant
            if attempt == 49:
                raise
            time.sleep(0.02)


_SAVE_LOCK = threading.Lock()


def save(layout: dict, path: str = PATH) -> None:
    """Atomic: the teach commands call this while the arm is live. One writer at a time, and on Windows the
    replace is retried while a reader (another thread's load) still has the file open."""
    with _SAVE_LOCK:
        tmp = f"{path}.{os.getpid()}.{threading.get_ident()}.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(layout, f, indent=2)
        for attempt in range(50):
            try:
                os.replace(tmp, path)
                return
            except PermissionError:
                if attempt == 49:
                    raise
                time.sleep(0.02)


def update_fixture(fixture_name: str, path: str = PATH, **keys) -> None:
    """Write `keys` into fixtures.<fixture_name> on disk, leaving everything else as the file has it.
    fixture_name may be a path (polar.pose_at_0). Used when the cell learns a pose or a grasp width."""
    layout = load(path)
    node = layout["fixtures"]
    for part in fixture_name.split("."):
        node = node[part]
    node.update(keys)
    save(layout, path)
