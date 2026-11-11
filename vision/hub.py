"""
cephla-hub stub: the only two things the workcell ever asks of the hub.

The hub itself is a separate project (AI-docs/hcs-workcell/to-do/cephla-hub.md). This file is the
workcell's side of the contract, with no network and no Hugging Face dependency yet:

  serve(check, version)   -> local path of models/<check>/<version>.onnx, or None if not cached
  record(check, frame, verdict, step)  -> one row in the local outbox, at check time
  outcome(step, succeeded)             -> one row when that step ends; the trainer joins the two by step

Recording is opt-in: layout.json vision.hub.record. Nothing here is required for the cell to run; a missing
model means "unchecked", never a crash.
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


def _append(name: str, row: dict, outbox_dir: str) -> dict:
    row["time"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    os.makedirs(outbox_dir, exist_ok=True)
    with open(os.path.join(outbox_dir, f"{name}.jsonl"), "a", encoding="utf-8") as f:
        f.write(json.dumps(row) + "\n")
    return row


def record(check: str, frame: str, verdict: str, step: str, outbox_dir: str = OUTBOX) -> dict:
    """One row for `check`: the saved photo, what the check said, and the step it was part of.
    The outbox stays on this machine until an upload job, gated on the customer's opt-in, pushes it to the hub."""
    return _append(check, {"check": check, "frame": frame, "verdict": verdict, "step": step}, outbox_dir)


def outcome(step: str, succeeded: bool, outbox_dir: str = OUTBOX) -> dict:
    """What then happened to `step`: the label every check row of that step is trained against."""
    return _append("outcomes", {"step": step, "succeeded": succeeded}, outbox_dir)
