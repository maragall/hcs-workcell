"""
cephla-hub stub: the only two things the workcell ever asks of the hub.

The hub itself is a separate project (AI-docs/hcs-workcell/to-do/cephla-hub.md). This file is the
workcell's side of the contract, with no network and no Hugging Face dependency yet:

  serve(check, version)   -> local path of models/<check>/<version>.onnx, or None if not cached
  record(check, frame, verdict, step_succeeded) -> appends one row to a local outbox

Nothing here is required for the cell to run, and nothing imports it yet. A missing model means
"unchecked", never a crash.
"""
from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from typing import Optional

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(os.path.dirname(HERE), "hub_models")    # models/<check>/<version>.onnx, placed by hand for now
OUTBOX = os.path.join(os.path.dirname(HERE), "hub_outbox")    # <check>.jsonl, uploaded only if the customer opts in

LEARNING_CHECKS = ("seating", "grip", "occupancy")   # retrained from outcomes
FIXED_CHECKS = ("align",)                           # ArUco rule, never retrained


def serve(check: str, version: str, models_dir: str = MODELS) -> Optional[str]:
    """Return the local path of the pinned model for `check`, or None when it is not cached.

    Later: download models/arm/<check>/<version>.onnx from the hub into models_dir when missing.
    Never resolves "latest": the cell pins a version in its config.
    """
    if check in FIXED_CHECKS:
        return None
    path = os.path.join(models_dir, check, f"{version}.onnx")
    return path if os.path.isfile(path) else None


def record(check: str, frame: str, verdict: str, step_succeeded: Optional[bool],
           outbox_dir: str = OUTBOX) -> dict:
    """Append one row to the local outbox for `check` and return the row.

    frame: path of the saved photo. verdict: what the check said. step_succeeded: what then happened,
    None if not known yet. The outbox stays on this machine until an upload job, gated on the
    customer's opt-in, pushes it to data/arm/<check> on the hub.
    """
    row = {
        "check": check,
        "frame": frame,
        "verdict": verdict,
        "step_succeeded": step_succeeded,
        "time": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    os.makedirs(outbox_dir, exist_ok=True)
    with open(os.path.join(outbox_dir, f"{check}.jsonl"), "a", encoding="utf-8") as f:
        f.write(json.dumps(row) + "\n")
    return row
