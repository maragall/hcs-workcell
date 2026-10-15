"""
Error correction: the bounded set of recovery actions the cycle may take on its own.

Every action is non-destructive and limited in count.  Anything not in this table ends the cycle
in a safe state and raises an operator alert.
"""
from __future__ import annotations

import time
from dataclasses import dataclass


class StopSafe(Exception):
    """Cycle cannot continue without an operator. State on disk says where the plate is."""


@dataclass
class Budget:
    regrasp: int = 1
    reimage: int = 3
    polar_retry: int = 1
    enable_retry: int = 1


class Corrector:
    def __init__(self, layout: dict, vision, robot, safety, tele, supervised: bool):
        self.layout, self.vision, self.robot, self.safety, self.tele = layout, vision, robot, safety, tele
        self.supervised = supervised
        self.budget = Budget()
        self.led_levels = [layout["vision"]["led"], 30, 60]

    def alert(self, step: str, reason: str):
        self.tele.log("alert", step, reason)
        with open(self.tele.run_dir + "/ALERT.txt", "a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {step}: {reason}\n")

    def operator_or_stop(self, step: str, reason: str, question: str) -> bool:
        self.alert(step, reason)
        if not self.supervised:
            raise StopSafe(reason)
        return input(f"  >>> {question} [y/N]: ").strip().lower() == "y"

    def on_lower_blocked(self, fixture: str, attempt: int, verdict) -> int:
        """Returns the LED level to use for the next image, or raises StopSafe."""
        if attempt < self.budget.reimage:
            level = self.led_levels[min(attempt, len(self.led_levels) - 1)]
            self.tele.log("correct", "lower_check", f"retry {attempt + 1} with LED {level}", score=verdict.best)
            time.sleep(1.0)
            return level
        if self.operator_or_stop("lower_check", f"{fixture}: scene does not match safe references ({verdict.reason})",
                                 "You looked at the fixture and it is safe to lower?"):
            return self.led_levels[0]
        raise StopSafe(f"lower blocked at {fixture}")

    def on_grasp_failed(self, fixture: str, attempt: int, detail: str) -> bool:
        """True = retry allowed."""
        self.tele.log("correct", "grasp", f"{fixture}: {detail} (attempt {attempt})")
        if attempt <= self.budget.regrasp:
            return True
        raise StopSafe(f"grasp failed twice at {fixture}: {detail}")

    def on_vision_too_far(self, fixture: str, mag: float):
        raise StopSafe(f"{fixture}: vision correction {mag:.1f} mm exceeds the allowed jump; marker or fixture moved")

    def on_vision_error(self, fixture: str, err) -> bool:
        """True = proceed without correction (taught pose only); only when the operator accepts."""
        return self.operator_or_stop("vision", f"{fixture}: {err}", "Proceed with the taught pose, no vision correction?")

    def on_motion_refused(self, step: str, err):
        raise StopSafe(f"{step}: {err}")

    def on_disabled(self, step: str) -> bool:
        if self.budget.enable_retry <= 0:
            return False
        self.budget.enable_retry -= 1
        self.tele.log("correct", step, "controller disabled; re-enabling")
        return self.robot.enable()

    def on_polar_failed(self, err) -> bool:
        if self.budget.polar_retry <= 0:
            raise StopSafe(f"polar stage: {err}")
        self.budget.polar_retry -= 1
        self.tele.log("correct", "polar", f"retrying after {err}")
        return True

    def on_drift(self, warning) -> None:
        msg = f"{warning.feature}={warning.value:.3f} median {warning.median:.3f} z={warning.z:.1f}"
        if warning.severity == "warn":
            self.tele.log("predict", "drift", msg)
            return
        if not self.operator_or_stop("drift", msg, "Drift is severe. Continue anyway?"):
            raise StopSafe("drift stop: " + msg)

    def new_cycle(self):
        self.budget = Budget()
