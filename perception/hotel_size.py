"""
Hotel size, 15 or 30 slots, from one front-camera picture at the hotel view pose.

The same normalised cross-correlation as the lowering gate and the slot detector, against reference pictures
of each size: reference_images/hotel_size/<15|30>/*.jpg. A picture that matches neither size well, or both
equally, is "unknown": the caller asks the user once and the answer becomes a reference (`learn`), so the
cell recognises its own hotel from then on. No size is ever guessed.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from perception.lower_check import _prep, ncc

SIZES = (15, 30)


@dataclass
class SizeVerdict:
    size: Optional[int]        # 15 | 30 | None
    scores: Dict[int, float]   # best correlation per size (0 when the size has no reference)
    reason: str


class HotelSize:
    def __init__(self, ref_root: str, min_refs: int = 1, min_match: float = 0.75, min_gap: float = 0.05):
        self.root, self.min_refs, self.min_match, self.min_gap = os.path.join(ref_root, "hotel_size"), min_refs, min_match, min_gap

    def _files(self, size: int) -> List[str]:
        d = os.path.join(self.root, str(size))
        return [os.path.join(d, n) for n in sorted(os.listdir(d)) if n.lower().endswith((".jpg", ".png"))] if os.path.isdir(d) else []

    def count(self, size: int) -> int:
        return len(self._files(size))

    def learn(self, size: int, img: bytes) -> str:
        if size not in SIZES:
            raise ValueError(f"a hotel has {SIZES} slots, not {size}")
        d = os.path.join(self.root, str(size))
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"ref_{self.count(size) + 1:03d}.jpg")
        with open(path, "wb") as f:
            f.write(img)
        return path

    def verdict(self, img: bytes) -> SizeVerdict:
        cur = _prep(img)
        scores = {}
        for size in SIZES:
            files = self._files(size)
            refs = []
            for p in files:
                with open(p, "rb") as f:
                    refs.append(_prep(f.read()))
            scores[size] = max((float(ncc(cur, r)) for r in refs), default=0.0) if len(refs) >= self.min_refs else 0.0
        best = max(scores, key=scores.get)
        other = min(scores, key=scores.get)
        if scores[best] < self.min_match:
            return SizeVerdict(None, scores, f"matches no reference (best {scores[best]:.3f}, needs {self.min_match})")
        if scores[best] - scores[other] < self.min_gap:
            return SizeVerdict(None, scores, f"15 and 30 look alike ({scores[15]:.3f} / {scores[30]:.3f})")
        return SizeVerdict(best, scores, f"{best} slots by {scores[best] - scores[other]:.3f}")
