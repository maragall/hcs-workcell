"""
After a collision or a controller fault: back to the nest, then stop.

The pose the arm stops in after a collision is often touching what it hit. `recover_to_nest` clears the controller's errors,
enables the arm (locked, not loose) and takes it to the nest (routes.nest) with Motion.go_nest, whose planner first backs the
claw out along its own axis if the arm stands inside an obstacle. It never retries and never continues the run: the caller stops,
and a marker file keeps a supervisor from relaunching into the same collision (the errors it would have checked are cleared here).
"""
from __future__ import annotations

import os
import time
from typing import Callable

from devices.arm import MotionRefused
from procedures.corrector import StopSafe
from procedures.safety import SafetyViolation

WORDS = ("controller error", "collision", "possible collision", "servo", "controller lost enable", "not enabled")
MARKER = "COLLISION"                      # runs/COLLISION: written with the reason; a supervisor does not relaunch while it exists


def is_motion_fault(reason: str) -> bool:
    """The text of a safe stop that came from the controller, not from a vision check or a plate state."""
    text = str(reason).lower()
    return any(w in text for w in WORDS)


def _wait_still(robot, seconds: float = 20.0) -> None:
    end = time.time() + seconds
    while time.time() < end:
        try:
            if not robot.state().moving:
                return
        except Exception:
            return
        time.sleep(0.2)


def recover_to_nest(robot, motion, log: Callable[..., None] = lambda *a, **k: None) -> str:
    """One attempt. `motion` is a Cycle (or any Motion with go_nest and a ledger). Returns what happened, in words."""
    _wait_still(robot)
    try:
        if hasattr(robot, "clear_errors"):
            robot.clear_errors()
        left = [e for e in (robot.errors() or []) if e.get("errCode")]
        if left:
            return "the controller still reports " + "; ".join(f"{e.get('errCode')} {e.get('errMsg', '')}" for e in left) + ": arm left where it stopped"
        if not robot.state().enabled and not robot.enable():
            return "the arm could not be enabled: left where it stopped"
        if robot.state().drag:
            robot.drag(False)
    except Exception as e:
        return f"could not clear the fault ({e}): arm left where it stopped"
    grip = getattr(getattr(motion, "store", None), "s", None) and motion.store.s.grip
    if grip and grip != "none":
        return f"errors cleared, arm enabled; the claw holds a plate ({grip}), so it stays where it stopped: inspect it, take the plate by hand"
    try:
        motion.go_nest()
    except (SafetyViolation, StopSafe, MotionRefused) as e:
        log("recover", "nest", f"not reached: {e}")
        return f"errors cleared, arm enabled; could not reach the nest: {e}"
    log("recover", "nest", "reached")
    return "errors cleared, arm enabled and at the nest"


def mark(runs_dir: str, reason: str) -> str:
    """runs/COLLISION: the reason and when; the supervisor refuses to relaunch while it exists."""
    path = os.path.join(runs_dir, MARKER)
    with open(path, "a", encoding="utf-8") as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {reason}\n")
    return path
