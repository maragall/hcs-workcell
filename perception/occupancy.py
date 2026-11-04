"""Is there a plate in this seat? One picture in, `full` / `empty` / `unsure` out, by distance from the seat's EMPTY pictures.

The plate is never modelled (hundreds of types and colours, labs differ in lighting and background): each station keeps its own
empty references under reference_images/occupancy/<station>/empty_<stamp>_<camera>.jpg and the question is only "how far is
the live picture from the nearest empty one", measured against how far the empties are from each other. `full_*` pictures
are not used by the verdict; tools/occupancy_eval.py uses them to show the margin.

Level 1 (no learning). Grey, region of interest (ROI, fractions of the frame: the central 60 % for the down camera over a seat,
the lower third for the front camera at a slot mouth, per-station overrides in ROI), 320 px wide, Gaussian blur. Three differences between the live ROI and one empty reference, all 0 for identical pictures:
  ncc   1 - normalised cross-correlation (cv2.matchTemplate TM_CCOEFF_NORMED), the live centre searched over +-SHIFT px in the
        reference so a millimetre of arm repeatability does not count as a difference
  grad  1 - normalised cross-correlation of the Sobel gradient magnitude maps (structure; noise amplitude and lighting gain cancel)
  edge  |log(Canny edge density live / empty)|, thresholds from the median grey of each picture
score = ncc + edge, against the nearest empty reference (grad is reported, not scored: it measures sensor noise on speckled metal).
Values seen 2026-11-04: two empties of the module 28 minutes apart 0.11, a clear 96-well plate in a slot 0.5..1.1, on a seat 0.7..0.9.

Threshold. N = the empty noise: the largest leave-one-out empty-vs-empty score (with 4 or more empties also mean + 3 sd), never
below NOISE_FLOOR; with a single empty nothing can be measured and the conservative DEFAULT_NOISE is used (Station.calibrated
says which).
verdict: score <= LOW * N -> empty; score >= HIGH * N -> full; between -> unsure.

Level 2 (optional, `deep=True`): PatchCore-style, CPU. ResNet18 (onnx model zoo, resnet18-v1-7) patch features from the
stage-3 output (14 x 14 x 256 over the 224 px ROI), nearest-neighbour L2 distance from each live patch to all patches of all
empty references, score = 95th percentile of those distances. Needs onnxruntime and the model file (MODEL); absent -> None.

    from perception.occupancy import Station
    st = Station.load("microscope")           # camera chosen by camera_for(); empties read and calibrated
    v = st.verdict(cv2.imread(path))          # v.state, v.score, v.noise, v.nearest, v.features
"""
from __future__ import annotations

import glob
import os
from dataclasses import dataclass, field

import cv2
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REFS = os.path.join(HERE, "reference_images", "occupancy")
MODEL = os.environ.get("OCC_MODEL", os.path.join(HERE, "models", "resnet18-v1-7.onnx"))

WIDTH = 320            # px, the ROI is resized to this width
BLUR = 1.5             # Gaussian sigma after the resize
SHIFT = 8              # px of shift tolerance in the correlation search (at WIDTH)
DEFAULT_NOISE = 0.25   # score units, the empty noise assumed when only one empty exists (ncc + edge: two empties 28 min apart scored 0.11)
NOISE_FLOOR = 0.15     # the noise is never taken below this: two empties minutes apart (A1: 0.00) have not seen a day's light or the arm's drift
LOW, HIGH = 1.5, 2.5   # reported bands as multiples of the empty noise (tools/occupancy_eval.py); the verdict uses the fixed bands below
EMPTY_MAX, FULL_MIN = 0.30, 0.45   # verdict: distance to the nearest empty reference; between the two is unsure
ROI_DEFAULT = (0.2, 0.2, 0.8, 0.8)   # x0, y0, x1, y1 as fractions of the frame: the central 60 % (down camera over a seat)
ROI_SLOT = (0.22, 0.65, 0.80, 1.0)   # front camera at a slot's approach point: the plate fills the lower third between the walls
ROI = {}                             # per-station overrides, e.g. "A1": (0.25, 0.6, 0.75, 1.0)
CAMERA = {"microscope": "down", "polar": "down"}   # the seat seen from above; slots (anything else) use the front camera


def camera_for(station):
    return CAMERA.get(station, "front")


def roi_for(station):
    return ROI.get(station, ROI_DEFAULT if camera_for(station) == "down" else ROI_SLOT)


def prepare(image, roi=ROI_DEFAULT):
    """Grey ROI at WIDTH px, blurred. `image` is a BGR or grey frame."""
    g = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    x0, y0, x1, y1 = int(roi[0] * w), int(roi[1] * h), int(roi[2] * w), int(roi[3] * h)
    g = g[y0:y1, x0:x1]
    g = cv2.resize(g, (WIDTH, int(round(g.shape[0] * WIDTH / g.shape[1]))), interpolation=cv2.INTER_AREA)
    return cv2.GaussianBlur(g, (0, 0), BLUR)


def _grad(g):
    m = cv2.magnitude(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))
    return m / (m.mean() + 1e-6)


def _edges(g):
    m = float(np.median(g))
    return cv2.Canny(g, int(max(0, 0.66 * m)), int(min(255, 1.33 * m)))


def features(live, ref):
    """The three differences between two prepared pictures plus the shift (dx, dy) at which the live centre matched."""
    t = live[SHIFT:-SHIFT, SHIFT:-SHIFT]
    _, ncc, _, (x, y) = cv2.minMaxLoc(cv2.matchTemplate(ref, t, cv2.TM_CCOEFF_NORMED))
    r = ref[y:y + t.shape[0], x:x + t.shape[1]]
    # 1 - correlation of the gradient maps: structure, not noise amplitude. The mean absolute difference used first scored 0.9 between two
    # empty pictures of the module's speckled pad 25 minutes apart (2026-11-03 23:55, different exposure, same scene).
    grad = 1.0 - float(cv2.minMaxLoc(cv2.matchTemplate(_grad(r), _grad(t), cv2.TM_CCOEFF_NORMED))[1])
    edge = abs(float(np.log((_edges(t).mean() / 255 + 1e-4) / (_edges(r).mean() / 255 + 1e-4))))
    return {"ncc": 1.0 - float(ncc), "grad": grad, "edge": edge, "shift": (x - SHIFT, y - SHIFT)}


def score(live, ref):
    """ncc + edge. The gradient term is reported but not scored: on the module's speckled pad two empties 28 minutes apart scored 0.69 on
    it (noise texture under a different exposure) while ncc and edge stayed at 0.08 and 0.03 (2026-11-04)."""
    f = features(live, ref)
    return f["ncc"] + f["edge"], f


def pictures(station, camera, state):
    """Paths of the recorded pictures of one station, camera and state, oldest first."""
    return sorted(glob.glob(os.path.join(REFS, station, f"{state}_*_{camera}.jpg")))


@dataclass
class Verdict:
    state: str            # full / empty / unsure
    score: float
    noise: float
    nearest: str          # file name of the closest empty reference
    features: dict
    deep: float | None = None


@dataclass
class Station:
    name: str
    camera: str
    roi: tuple
    empties: list = field(default_factory=list)   # [(file name, prepared array)]
    noise: float = DEFAULT_NOISE
    calibrated: bool = False                       # False: noise is DEFAULT_NOISE because fewer than two empties exist
    loo: list = field(default_factory=list)        # leave-one-out empty-vs-empty scores
    _deep: object = None

    @classmethod
    def load(cls, name, camera=None, roi=None, paths=None):
        """Empties from the station folder (or `paths`), then calibrated."""
        camera = camera or camera_for(name)
        st = cls(name, camera, roi or roi_for(name))
        for p in (paths if paths is not None else pictures(name, camera, "empty")):
            im = cv2.imread(p)
            if im is not None:
                st.empties.append((os.path.basename(p), prepare(im, st.roi)))
        return st.calibrate()

    def calibrate(self):
        self.loo = [self.distance(a, skip=i)[0] for i, (_, a) in enumerate(self.empties)] if len(self.empties) > 1 else []
        if self.loo:
            self.noise = max(NOISE_FLOOR, max(self.loo))
            if len(self.loo) >= 4:
                self.noise = max(self.noise, float(np.mean(self.loo) + 3 * np.std(self.loo)))
            self.calibrated = True
        return self

    def distance(self, prepared, skip=None):
        """Score against the nearest empty reference: (score, name, features)."""
        best = None
        for i, (n, ref) in enumerate(self.empties):
            if i == skip:
                continue
            s, f = score(prepared, ref)
            if best is None or s < best[0]:
                best = (s, n, f)
        if best is None:
            raise ValueError(f"{self.name}: no empty reference for the {self.camera} camera under {REFS}")
        return best

    def verdict(self, image, deep=False):
        """Fixed bands on the distance to the NEAREST empty reference (EMPTY_MAX / FULL_MIN), not multiples of the station's spread: once
        day and night empties sit in one folder the spread is the lighting gap (1 to 6.6 on 2026-11-04 01:34) and bands scaled by it swallow a
        plate (B3 at 3.9 read empty). Measured: same-light empties stay under 0.2 of their nearest, a plate scores 0.5 and up."""
        s, n, f = self.distance(prepare(image, self.roi))
        v = Verdict("empty" if s <= EMPTY_MAX else "full" if s >= FULL_MIN else "unsure", s, self.noise, n, f)
        if deep:
            v.deep = self.deep_score(image)
        return v

    # ---- level 2: PatchCore-style nearest patch distance on ResNet18 features ----
    def deep_score(self, image):
        """95th percentile over live patches of the L2 distance to the nearest patch of any empty reference; None without the model."""
        emb = _embedder()
        if emb is None:
            return None
        if self._deep is None:                                              # the memory bank: every patch of every empty
            raws = [cv2.imread(os.path.join(REFS, self.name, n)) for n, _ in self.empties]
            self._deep = np.concatenate([emb(_roi_crop(r, self.roi)) for r in raws if r is not None])
        live = emb(_roi_crop(image, self.roi))
        d = np.sqrt(np.maximum((live ** 2).sum(1)[:, None] + (self._deep ** 2).sum(1)[None] - 2 * live @ self._deep.T, 0))
        return float(np.percentile(d.min(1), 95))


def _roi_crop(image, roi):
    h, w = image.shape[:2]
    return image[int(roi[1] * h):int(roi[3] * h), int(roi[0] * w):int(roi[2] * w)]


_EMB = []


def _embedder():
    """ResNet18 stage-3 patch features (N x 256) of a BGR crop, or None when onnxruntime or the model file is missing."""
    if _EMB:
        return _EMB[0]
    try:
        import onnxruntime as ort
        from scipy.ndimage import uniform_filter
        sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
    except Exception:
        _EMB.append(None)
        return None
    mean, std = np.array([0.485, 0.456, 0.406], np.float32), np.array([0.229, 0.224, 0.225], np.float32)
    out = [o.name for o in sess.get_outputs()]
    layer = next((o for o in out if "stage3" in o), out[0])

    def emb(bgr):
        x = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), (224, 224), interpolation=cv2.INTER_AREA).astype(np.float32) / 255
        x = ((x - mean) / std).transpose(2, 0, 1)[None]
        f = sess.run([layer], {sess.get_inputs()[0].name: x})[0][0]          # C x H x W
        f = uniform_filter(f, size=(1, 3, 3), mode="nearest")                   # PatchCore's local neighbourhood average
        return f.transpose(1, 2, 0).reshape(-1, f.shape[0]).astype(np.float32)
    _EMB.append(emb)
    return emb
