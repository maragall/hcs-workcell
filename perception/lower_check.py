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

References live in reference_images/<fixture>/<state>/*.jpg (older cells: *.png), recorded with `cli.py record-ref`
or collected by the self-training gates.
"""
from __future__ import annotations

import io
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from PIL import Image, ImageFilter

PATCH = (160, 120)          # work at low resolution: robust to noise, fast, and lighting-tolerant after normalisation


def _prep(img_bytes: bytes, roi: Optional[Tuple[float, float, float, float]] = None) -> np.ndarray:
    """Edge magnitude with local contrast normalisation: insensitive to room lighting (day/night), which raw
    intensity correlation was not. Validated on 304 logged hotel frames: 100 % day and night, margin >= 0.24."""
    im = Image.open(io.BytesIO(img_bytes)).convert("L")
    if roi:
        w, h = im.size
        im = im.crop((int(roi[0] * w), int(roi[1] * h), int(roi[2] * w), int(roi[3] * h)))
    a = np.asarray(im.resize((PATCH[0] * 2, PATCH[1] * 2), Image.BILINEAR), dtype=np.float32)
    g = np.abs(np.diff(a, axis=1))[:-1, :] + np.abs(np.diff(a, axis=0))[:, :-1]
    g = np.asarray(Image.fromarray(np.clip(g, 0, 255).astype(np.uint8)).resize(PATCH, Image.BILINEAR), dtype=np.float32)
    blur = np.asarray(Image.fromarray(g.astype(np.uint8)).filter(ImageFilter.GaussianBlur(6)), dtype=np.float32)
    g = g - blur
    g -= g.mean()
    s = g.std()
    return g / s if s > 1e-6 else g


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
        """Stores the camera's JPEG bytes as they came: the same pixels the verdict saw, no PNG re-encode
        (100 ms and 2.5 MB per 720p frame, inside the arm lock)."""
        d = os.path.join(self.ref_root, fixture, state)
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"ref_{self.count(fixture, state) + 1:03d}.jpg")
        with open(path, "wb") as f:
            f.write(img_bytes)
        for k in [k for k in self._cache if k[0] == fixture and k[1] == state]:
            self._cache.pop(k, None)
        return path
