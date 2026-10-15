"""
Error prediction (level 1 baseline, feeds level 2).

Two mechanisms:
  1. Pre-step checks: deterministic conditions that must hold before a step may start.
  2. Drift statistics: rolling per-step baselines of duration, jaw closing value, vision offset
     magnitude and lower-check score.  A measurement far from its own history raises a warning
     before it becomes a failure (e.g. the grasp value creeping up = plate slipping / jaw wearing;
     vision offsets growing = stage or marker moving; lower-check scores sinking = lighting or a
     camera knock).

The thresholds are intentionally simple (robust z-scores on medians).  Level 2 replaces this file
with trained models on the same features; the feature names are the contract.
"""
from __future__ import annotations

import json
import os
import statistics
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional


@dataclass
class Warning_:
    feature: str
    value: float
    median: float
    mad: float
    z: float
    severity: str      # "warn" | "stop"


class DriftMonitor:
    def __init__(self, path: str, window: int = 60, warn_z: float = 3.5, stop_z: float = 6.0, min_n: int = 8,
                 min_rel_scale: float = 0.02, min_abs_scale: float = 1e-3):
        self.path, self.window, self.warn_z, self.stop_z, self.min_n = path, window, warn_z, stop_z, min_n
        self.min_rel_scale, self.min_abs_scale = min_rel_scale, min_abs_scale
        self.hist: Dict[str, Deque[float]] = defaultdict(lambda: deque(maxlen=window))
        self._load()

    def _load(self):
        if os.path.exists(self.path):
            try:
                with open(self.path, "r", encoding="utf-8") as f:
                    d = json.load(f)
                for k, v in d.items():
                    self.hist[k].extend(v[-self.window:])
            except (OSError, ValueError):
                pass

    def save(self):
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump({k: list(v) for k, v in self.hist.items()}, f)

    def observe(self, feature: str, value: float) -> Optional[Warning_]:
        h = self.hist[feature]
        verdict = None
        if len(h) >= self.min_n:
            med = statistics.median(h)
            mad = statistics.median(abs(x - med) for x in h)
            scale = max(1.4826 * mad, self.min_rel_scale * abs(med), self.min_abs_scale)
            z = abs(value - med) / scale
            if z >= self.stop_z:
                verdict = Warning_(feature, value, med, mad, z, "stop")
            elif z >= self.warn_z:
                verdict = Warning_(feature, value, med, mad, z, "warn")
        h.append(value)
        return verdict

    def slope(self, feature: str, last_n: int = 20) -> float:
        """Simple trend: least-squares slope of the last n values (units per cycle)."""
        h = list(self.hist[feature])[-last_n:]
        n = len(h)
        if n < 5:
            return 0.0
        xs = range(n)
        mx, my = (n - 1) / 2, sum(h) / n
        num = sum((x - mx) * (y - my) for x, y in zip(xs, h))
        den = sum((x - mx) ** 2 for x in xs)
        return num / den if den else 0.0


class PreChecks:
    """Deterministic conditions evaluated before a step. Each returns a list of problems."""

    def __init__(self, layout: dict):
        self.layout = layout
        self.closed = layout["gripper"]["closed"]
        self.jaw_tol = layout["gripper"]["tolerance"]
        self.jaw_open = layout["gripper"]["open"]

    def before_grasp(self, jaw: float, robot_state) -> List[str]:
        p = []
        if jaw < self.jaw_open - 1.0:
            p.append(f"jaw not open before grasp ({jaw:.2f})")
        if robot_state.drag:
            p.append("drag mode on")
        if not robot_state.enabled:
            p.append("robot not enabled")
        return p

    def grasp_ok(self, jaw: float, grip: str) -> Optional[str]:
        want = self.closed[grip]
        if abs(jaw - want) <= self.jaw_tol:
            return None
        if jaw < want - self.jaw_tol:
            return f"jaw closed to {jaw:.2f}: nothing or too little between the fingers (plate missed / wrong edge)"
        return f"jaw stopped at {jaw:.2f}: obstruction or plate not seated"

    def before_release(self, jaw: float, grip: str) -> List[str]:
        want = self.closed[grip]
        return [] if abs(jaw - want) <= self.jaw_tol else [f"not holding a plate on {grip} ({jaw:.2f})"]
