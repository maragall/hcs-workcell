"""
Persisted cycle state.  Written BEFORE and AFTER every step, so a crash or power cycle can resume
without repeating a grasp or a release.  The state answers one question at restart:
"where is the plate, and what is the gripper holding?"
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional


@dataclass
class CellState:
    plate_at: str = "microscope"          # microscope | polar | hotel:<slot> | gripper
    polar_angle: int = 0                  # 0 or 90
    grip: str = "none"                    # none | short_edges | long_edges
    step: str = "idle"                    # last step started
    step_done: bool = True                # whether that step completed
    cycle: int = 0
    hotel: Dict[str, str] = field(default_factory=dict)     # slot -> plate id ("" if unknown)
    plate_id: str = ""                    # barcode if scanned
    faults: List[dict] = field(default_factory=list)
    last_safe_pose: Optional[List[float]] = None
    updated: float = 0.0


class StateStore:
    def __init__(self, path: str):
        self.path = path
        self.s = self.load()

    def load(self) -> CellState:
        if os.path.exists(self.path):
            with open(self.path, "r", encoding="utf-8") as f:
                return CellState(**json.load(f))
        return CellState()

    def save(self):
        self.s.updated = time.time()
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(asdict(self.s), f, indent=2)
        os.replace(tmp, self.path)

    def begin(self, step: str):
        self.s.step, self.s.step_done = step, False
        self.save()

    def done(self, **changes):
        for k, v in changes.items():
            setattr(self.s, k, v)
        self.s.step_done = True
        self.save()

    def fault(self, step: str, kind: str, detail: str):
        self.s.faults.append({"t": time.time(), "cycle": self.s.cycle, "step": step, "kind": kind, "detail": detail})
        self.s.faults = self.s.faults[-200:]
        self.save()
