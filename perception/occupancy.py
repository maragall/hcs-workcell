"""
Slot occupancy detector: is there a plate in this hotel slot? Front camera at the approach point.

Two classes of reference frames per slot, "empty" and "occupied", scored by the same normalised
cross-correlation as the lowering gate. The cell labels its own references: the approach before a
place sees the slot empty, the approach before a pick sees it occupied (procedures/gates.py _slot_view).

A detector is `ready` for a slot once both classes have `min_refs` frames and a leave-one-out pass over
those frames gets every one right. Until then its verdict is logged, not enforced.

References: reference_images/slot_<slot>/{empty,occupied}/*.jpg. Report: `python cli.py occupancy`.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from perception.lower_check import _prep, ncc

STATES = ("empty", "occupied")


@dataclass
class OccupancyVerdict:
    decision: str          # "empty" | "occupied" | "unknown"
    confidence: float      # score gap between the two classes, 0 when unknown
    score_empty: float
    score_occupied: float
    n_refs: int            # the smaller class
    reason: str


@dataclass
class SelfTest:
    n: int                 # frames tested
    wrong: int             # frames whose own class did not win
    min_gap: float         # smallest winning gap, negative when something was wrong


class Occupancy:
    def __init__(self, ref_root: str, min_refs: int = 5, min_match: float = 0.75):
        self.ref_root, self.min_refs, self.min_match = ref_root, min_refs, min_match
        self._cache: Dict[tuple, List[np.ndarray]] = {}
        self._tests: Dict[str, SelfTest] = {}          # per slot, for the references in _cache

    # ---------------------------------------------------------------- references
    def _dir(self, slot: str, state: str) -> str:
        return os.path.join(self.ref_root, f"slot_{slot}", state)

    def _files(self, slot: str, state: str) -> List[str]:
        d = self._dir(slot, state)
        if not os.path.isdir(d):
            return []
        return [os.path.join(d, n) for n in sorted(os.listdir(d)) if n.lower().endswith((".jpg", ".png"))]

    def _refs(self, slot: str, state: str) -> List[np.ndarray]:
        key = (slot, state)
        if key not in self._cache:
            refs = []
            for p in self._files(slot, state):
                with open(p, "rb") as f:
                    refs.append(_prep(f.read()))
            self._cache[key] = refs
        return self._cache[key]

    def count(self, slot: str, state: str) -> int:
        return len(self._files(slot, state))

    def newest_age_s(self, slot: str, state: str) -> float:
        files = self._files(slot, state)
        return time.time() - max(os.path.getmtime(p) for p in files) if files else float("inf")

    def add_reference(self, slot: str, state: str, img_bytes: bytes) -> str:
        d = self._dir(slot, state)
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"ref_{self.count(slot, state) + 1:03d}.jpg")
        with open(path, "wb") as f:
            f.write(img_bytes)
        self._cache.pop((slot, state), None)
        self._tests.pop(slot, None)
        return path

    def slots(self) -> List[str]:
        if not os.path.isdir(self.ref_root):
            return []
        return sorted(n[5:] for n in os.listdir(self.ref_root) if n.startswith("slot_"))

    # ------------------------------------------------------------------- verdict
    def _decide(self, cur: np.ndarray, empty: List[np.ndarray], occupied: List[np.ndarray]) -> OccupancyVerdict:
        se = max(ncc(cur, r) for r in empty)
        so = max(ncc(cur, r) for r in occupied)
        n = min(len(empty), len(occupied))
        if max(se, so) < self.min_match:
            return OccupancyVerdict("unknown", 0.0, se, so, n, f"matches neither class (best {max(se, so):.3f})")
        decision = "empty" if se > so else "occupied"
        return OccupancyVerdict(decision, abs(se - so), se, so, n, f"{decision} by {abs(se - so):.3f}")

    def verdict(self, slot: str, img_bytes: bytes) -> OccupancyVerdict:
        empty, occupied = self._refs(slot, "empty"), self._refs(slot, "occupied")
        if len(empty) < self.min_refs or len(occupied) < self.min_refs:
            n = min(len(empty), len(occupied))
            return OccupancyVerdict("unknown", 0.0, 0.0, 0.0, n, f"{n} references, need {self.min_refs} per class")
        return self._decide(_prep(img_bytes), empty, occupied)

    def self_test(self, slot: str) -> SelfTest:
        """Leave one out: every reference must be classified as its own label by the others. Computed once per
        reference set (ready() asks at every slot view; 24 + 24 references are 2000 correlations)."""
        if slot in self._tests:
            return self._tests[slot]
        empty, occupied = self._refs(slot, "empty"), self._refs(slot, "occupied")
        n = wrong = 0
        min_gap = float("inf")
        for label, own, other in (("empty", empty, occupied), ("occupied", occupied, empty)):
            for i, cur in enumerate(own):
                rest = own[:i] + own[i + 1:]
                if not rest or not other:
                    continue
                e, o = (rest, other) if label == "empty" else (other, rest)
                v = self._decide(cur, e, o)
                n += 1
                gap = (v.score_empty - v.score_occupied) * (1 if label == "empty" else -1)
                min_gap = min(min_gap, gap)
                if v.decision != label:
                    wrong += 1
        self._tests[slot] = SelfTest(n, wrong, min_gap if n else 0.0)
        return self._tests[slot]

    def ready(self, slot: str) -> bool:
        """Enough references of both kinds and a clean self-test: the verdict may block a move."""
        if min(self.count(slot, "empty"), self.count(slot, "occupied")) < self.min_refs:
            return False
        return self.self_test(slot).wrong == 0

    def report(self) -> List[dict]:
        """One row per slot: counts, self-test, ready. For `cli.py occupancy`."""
        rows = []
        for slot in self.slots():
            t = self.self_test(slot)
            rows.append({"slot": slot, "empty": self.count(slot, "empty"), "occupied": self.count(slot, "occupied"),
                         "tested": t.n, "wrong": t.wrong, "min_gap": round(t.min_gap, 3), "ready": self.ready(slot)})
        return rows
