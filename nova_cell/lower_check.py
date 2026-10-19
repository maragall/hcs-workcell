"""
"Safe to lower" check by normalised cross-correlation against reference images.

Answer to the design question: this is NOT registration of the plate to the stage.  It is a
go / no-go gate.  Before the gripper descends at a fixture, the camera takes an image from the
hover pose and compares it with reference images that were recorded, at that same hover pose,
when lowering was known to be safe.  If the scene matches one of them well enough, lowering is
allowed.  Anything unexpected in the scene (a misplaced plate, a lid, a hand, a stage that did not
rotate, a cable) lowers the correlation and blocks the descent.

Tuned for NO false negatives (never says "safe" when it is not): the threshold is set from the
spread of the "safe" references themselves, and any doubt returns False.  False positives (refusing
to lower when it was in fact fine) are accepted and simply cost a retry or an operator look.

References live in reference_images/<fixture>/<state>/*.png, recorded with `run.py record-ref`.
"""
from __future__ import annotations

import io
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from PIL import Image

PATCH = (160, 120)          # work at low resolution: robust to noise, fast, and lighting-tolerant after normalisation


def _prep(img_bytes: bytes, roi: Optional[Tuple[float, float, float, float]] = None) -> np.ndarray:
    im = Image.open(io.BytesIO(img_bytes)).convert("L")
    if roi:  # fractional crop (x0, y0, x1, y1)
        w, h = im.size
        im = im.crop((int(roi[0] * w), int(roi[1] * h), int(roi[2] * w), int(roi[3] * h)))
    a = np.asarray(im.resize(PATCH, Image.BILINEAR), dtype=np.float32)
    a -= a.mean()
    s = a.std()
    return a / s if s > 1e-6 else a


def ncc(a: np.ndarray, b: np.ndarray) -> float:
    return float((a * b).mean())      # both are zero-mean, unit-std -> Pearson correlation in [-1, 1]


@dataclass
class LowerVerdict:
    safe: bool
    best: float
    threshold: float
    n_refs: int
    reason: str


class LowerCheck:
    def __init__(self, ref_root: str, min_refs: int = 3, margin: float = 0.08, floor: float = 0.55,
                 roi: Optional[Tuple[float, float, float, float]] = None):
        self.ref_root, self.min_refs, self.margin, self.floor, self.roi = ref_root, min_refs, margin, floor, roi
        self._cache = {}

    def _refs(self, fixture: str, state: str, roi=None) -> List[np.ndarray]:
        key = (fixture, state, roi)
        if key not in self._cache:
            d = os.path.join(self.ref_root, fixture, state)
            refs = []
            if os.path.isdir(d):
                for n in sorted(os.listdir(d)):
                    if n.lower().endswith((".png", ".jpg", ".jpeg")):
                        with open(os.path.join(d, n), "rb") as f:
                            refs.append(_prep(f.read(), roi or self.roi))
            self._cache[key] = refs
        return self._cache[key]

    def count(self, fixture: str, state: str) -> int:
        d = os.path.join(self.ref_root, fixture, state)
        return len([n for n in os.listdir(d) if n.lower().endswith((".png", ".jpg", ".jpeg"))]) if os.path.isdir(d) else 0

    def threshold(self, refs: List[np.ndarray]) -> float:
        """Lowest pairwise correlation among the safe references, minus a margin; never below `floor`."""
        if len(refs) < 2:
            return 0.95
        worst = min(ncc(refs[i], refs[j]) for i in range(len(refs)) for j in range(i + 1, len(refs)))
        return max(self.floor, worst - self.margin)

    def verdict(self, fixture: str, state: str, img_bytes: bytes, roi=None) -> LowerVerdict:
        refs = self._refs(fixture, state, roi)
        if len(refs) < self.min_refs:
            return LowerVerdict(False, 0.0, 1.0, len(refs),
                                f"only {len(refs)} reference images for {fixture}/{state}; need {self.min_refs}")
        cur = _prep(img_bytes, roi or self.roi)
        scores = [ncc(cur, r) for r in refs]
        best, thr = max(scores), self.threshold(refs)
        if np.isnan(best):
            return LowerVerdict(False, 0.0, thr, len(refs), "image has no contrast")
        return LowerVerdict(best >= thr, best, thr, len(refs),
                            "match" if best >= thr else f"best correlation {best:.3f} below {thr:.3f}")

    def add_reference(self, fixture: str, state: str, img_bytes: bytes) -> str:
        d = os.path.join(self.ref_root, fixture, state)
        os.makedirs(d, exist_ok=True)
        n = len([x for x in os.listdir(d) if x.lower().endswith(".png")]) + 1
        path = os.path.join(d, f"ref_{n:03d}.png")
        Image.open(io.BytesIO(img_bytes)).convert("RGB").save(path)
        for k in [k for k in self._cache if k[0] == fixture and k[1] == state]:
            self._cache.pop(k, None)
        return path


@dataclass
class TwoClassVerdict:
    decision: str          # "empty" | "occupied" | "unknown"
    score_empty: float
    score_occupied: float
    n_refs: int


class SlotCheck:
    """Two-class decision for a hotel slot from the front camera at the approach point."""

    def __init__(self, ref_root: str, min_refs: int = 5, margin: float = 0.0):
        self.ref_root, self.min_refs, self.margin = ref_root, min_refs, margin
        self._cache = {}

    def _refs(self, slot: str, state: str):
        key = (slot, state)
        if key not in self._cache:
            d = os.path.join(self.ref_root, f"slot_{slot}", state)
            refs = []
            if os.path.isdir(d):
                for n in sorted(os.listdir(d)):
                    if n.lower().endswith((".jpg", ".png")):
                        with open(os.path.join(d, n), "rb") as f:
                            refs.append(_prep(f.read()))
            self._cache[key] = refs
        return self._cache[key]

    def verdict(self, slot: str, img_bytes: bytes) -> TwoClassVerdict:
        e, o = self._refs(slot, "empty"), self._refs(slot, "occupied")
        if len(e) < self.min_refs or len(o) < self.min_refs:
            return TwoClassVerdict("unknown", 0.0, 0.0, min(len(e), len(o)))
        cur = _prep(img_bytes)
        se, so = max(ncc(cur, r) for r in e), max(ncc(cur, r) for r in o)
        decision = "empty" if se - so > self.margin else "occupied"
        return TwoClassVerdict(decision, float(se), float(so), min(len(e), len(o)))
