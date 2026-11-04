"""Is the plate seated flat? One FRONT-camera picture taken as the fingers open at a seat, `flat` / `misplaced` / `unsure` out.

What is measured. The plate fills the lower quarter of the frame; its FAR long edge (away from the fingers, the top of the plate
region) is a near-horizontal dark-to-bright transition: dark table skirt or instrument above, the lit plate rim below. In a
band of the frame (BAND rows, BANDS per station, full width minus MARGIN on each side) the first strong downward dark-to-bright
transition of every STEP-th column is taken (Sobel y on a blurred grey picture, threshold relative to the strongest edge in the
band, sub-pixel peak); a RANSAC search keeps the longest line of points within RESID px (posts, hotel parts and well highlights
caught first in some columns fall out), then cv2.fitLine DIST_HUBER refits it and its inliers. Reported per picture (Edge):
  angle  degrees of the fitted line, positive = right side higher in the picture
  row    mean pixel row of the inlier points (the edge's height; a smaller row = higher in the picture = the plate sits higher)
  left, right   median row of the inlier points in the left and right thirds of the band (a plate on one lip rises on one side)
  n      inlier points; ok is False when fewer than MIN_POINTS remain (then the verdict is unsure)
Camera geometry does not cancel out: the flat edge itself is not level in the picture (lens and mounting), so every number is
only read as a difference from the station's flat reference(s): reference_images/seat/<station>/flat_*_front.jpg, the medians
of their angle, row, left and right. Verdict (Station.verdict), with dh = max(|row - row_flat|, |left - left_flat|,
|right - right_flat|) and da = |angle - angle_flat|:
  flat        dh <= FLAT_PX and da <= FLAT_DEG
  misplaced   dh >= BAD_PX or da >= BAD_DEG
  unsure      between, no flat reference for the station, or the edge not found
Numbers of 2026-11-04 (microscope seat, 1280 x 960, the far edge at row ~782, about 8.3 px per mm there): the flat edge repeats
within 1.2 px between pictures 20 minutes apart (arm re-posed each time); a plate with its far side up on the seat's lip reads
7..9 px higher (about 1 mm: the lip); cocked on a far corner 20..24 px higher on that side, angle 1.3..1.7 deg from flat.
A plate resting on the NEAR lip alone leaves the far edge where it was (5 of the 12 misplaced pictures): this check does not
see it, the near rim is below the frame. The flat references are per station AND per plate type: a lid adds 2.4 mm (20 px).
Polar (module): the clear plate stands on a white platform, its rim is a faint edge (1..2 grey/px) found on the left 60 % of
the band only (n ~110 of 256; `right` rides on the well tops there, so trust row and left); the held plate before release
reads 19 px above the seated one. A flat read on this station is weaker evidence than on the microscope seat.

    from perception.tilt import Station, far_edge, draw
    e = far_edge(cv2.imread(path))              # Edge, no reference needed
    st = Station.load("microscope")             # flat references read, st.calibrated says whether any exist
    v = st.verdict(cv2.imread(path))            # v.state, v.edge, v.dh, v.da, v.ref
    draw(img, e, out_png)                       # the points and the line on the picture, for a human to check
Table over all pictures and the flat / misplaced separation: python -X utf8 tools/tilt_eval.py
"""
from __future__ import annotations

import glob
import math
import os
from dataclasses import dataclass

import cv2
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REFS = os.path.join(HERE, "reference_images", "seat")

BAND = (0.72, 1.0)     # rows of the frame searched, fractions; the far edge is near 0.81 at the microscope seat
BANDS = {"polar": (0.80, 1.0)}   # per station: on the module the plate stands on a white platform whose own far edge (row 708) must be skipped
MARGIN = 0.10          # columns dropped on each side, fraction of the width
BLUR = 2.0             # Gaussian sigma on the grey picture
STEP = 4               # every STEP-th column gives one edge point
MIN_GRAD = 1.5         # grey levels per pixel: no transition weaker than this counts (noise after the blur is ~0.3)
REL = 0.35             # ... nor weaker than this fraction of the band's 99.5th-percentile gradient
RANSAC = 300           # line hypotheses tried on the points; the one with most points within RESID is the far edge
RESID = 3.0            # px from the fitted line: inliers of the final fit
MIN_POINTS = 30
FLAT_PX, BAD_PX = 3.0, 5.0      # height bands, px: 2.5x and 4x the 1.2 px repeatability; the lip reads 8 px, so 5 is over half a lip
FLAT_DEG, BAD_DEG = 0.2, 0.4    # angle bands, deg: 3.5 and 7 px across the 1024 px band (0.4 deg = one lip under one far corner)


@dataclass
class Edge:
    angle: float = float("nan")
    row: float = float("nan")
    left: float = float("nan")
    right: float = float("nan")
    n: int = 0
    ok: bool = False
    points: np.ndarray = None      # (N, 3) x, y, inlier flag; for draw()
    line: tuple = None             # vx, vy, x0, y0 of the fit, frame coordinates
    band: tuple = BAND


def far_edge(img, band=BAND, margin=MARGIN) -> Edge:
    """The far edge of the plate in one front picture (BGR or grey)."""
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(g, (0, 0), BLUR)
    H, W = g.shape
    y0, y1, x0, x1 = int(H * band[0]), int(H * band[1]), int(W * margin), int(W * (1 - margin))
    gy = cv2.Sobel(g[y0:y1, x0:x1].astype(np.float32), cv2.CV_32F, 0, 1, ksize=3) / 8.0
    thr = max(MIN_GRAD, REL * float(np.percentile(gy, 99.5)))
    pts = []
    for x in range(0, gy.shape[1], STEP):
        col = gy[:, x]
        hit = np.flatnonzero(col >= thr)
        if not len(hit):
            continue
        r = int(hit[0])
        while r + 1 < len(col) and col[r + 1] > col[r]:     # climb to the peak of this transition
            r += 1
        sub = 0.0
        if 0 < r < len(col) - 1:
            a, b, c = col[r - 1], col[r], col[r + 1]
            d = a - 2 * b + c
            sub = 0.5 * (a - c) / d if d != 0 else 0.0
        pts.append((x0 + x, y0 + r + sub, 1.0))
    e = Edge(points=np.array(pts, dtype=np.float32).reshape(-1, 3), band=band)
    if len(pts) < MIN_POINTS:
        return e
    P = e.points
    rng, keep = np.random.default_rng(0), np.zeros(len(P), bool)
    for _ in range(RANSAC):                                     # the longest consistent line through the points wins
        i, j = rng.choice(len(P), 2, replace=False)
        if abs(P[i, 0] - P[j, 0]) < (x1 - x0) / 4:
            continue
        slope = (P[j, 1] - P[i, 1]) / (P[j, 0] - P[i, 0])
        inl = np.abs(P[:, 1] - P[i, 1] - slope * (P[:, 0] - P[i, 0])) <= RESID
        if inl.sum() > keep.sum():
            keep = inl
    for _ in range(2):                                          # robust refit on the consensus, then on its own inliers
        if keep.sum() < MIN_POINTS:
            return e
        vx, vy, cx, cy = (float(v) for v in cv2.fitLine(P[keep, :2], cv2.DIST_HUBER, 0, 0.01, 0.01).ravel())
        if vx < 0:
            vx, vy = -vx, -vy
        keep = np.abs((P[:, 0] - cx) * vy - (P[:, 1] - cy) * vx) <= RESID
    P[:, 2] = keep
    xs, ys = P[keep, 0], P[keep, 1]
    third = (x1 - x0) / 3.0
    lo, hi = ys[xs < x0 + third], ys[xs >= x1 - third]
    e.angle = -math.degrees(math.atan2(vy, vx))
    e.row, e.n, e.ok, e.line = float(ys.mean()), int(keep.sum()), True, (vx, vy, cx, cy)
    e.left = float(np.median(lo)) if len(lo) else cy + (x0 + third / 2 - cx) * vy / vx
    e.right = float(np.median(hi)) if len(hi) else cy + (x1 - third / 2 - cx) * vy / vx
    return e


@dataclass
class Verdict:
    state: str
    edge: Edge
    dh: float = float("nan")   # px, the largest of the three height differences from the flat reference
    da: float = float("nan")   # deg, angle difference from the flat reference
    ref: dict = None


class Station:
    """The flat references of one station and the verdict against them."""

    def __init__(self, name, flats):
        self.name, self.band, self.flats = name, BANDS.get(name, BAND), [e for e in flats if e.ok]
        self.calibrated = bool(self.flats)
        self.ref = {k: float(np.median([getattr(e, k) for e in self.flats])) for k in ("angle", "row", "left", "right")} if self.flats else None

    @classmethod
    def load(cls, name, pattern="flat_*_front.jpg"):
        paths = sorted(glob.glob(os.path.join(REFS, name, pattern)))
        return cls(name, [far_edge(cv2.imread(p), BANDS.get(name, BAND)) for p in paths])

    def verdict(self, img) -> Verdict:
        e = far_edge(img, self.band)
        if not (e.ok and self.calibrated):
            return Verdict("unsure", e, ref=self.ref)
        r = self.ref
        dh = max(abs(e.row - r["row"]), abs(e.left - r["left"]), abs(e.right - r["right"]))
        da = abs(e.angle - r["angle"])
        state = "flat" if dh <= FLAT_PX and da <= FLAT_DEG else "misplaced" if dh >= BAD_PX or da >= BAD_DEG else "unsure"
        return Verdict(state, e, dh, da, r)


def draw(img, e: Edge, path, text=""):
    """The band, the edge points (green inliers, red rejected) and the fitted line on the picture, saved to path."""
    out = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    H, W = out.shape[:2]
    cv2.rectangle(out, (int(W * MARGIN), int(H * e.band[0])), (int(W * (1 - MARGIN)) - 1, int(H * e.band[1]) - 1), (255, 128, 0), 1)
    if e.points is not None:
        for x, y, k in e.points:
            cv2.circle(out, (int(round(x)), int(round(y))), 2, (0, 200, 0) if k else (0, 0, 255), -1)
    if e.line:
        vx, vy, cx, cy = e.line
        xa, xb = W * MARGIN, W * (1 - MARGIN)
        cv2.line(out, (int(xa), int(round(cy + (xa - cx) * vy / vx))), (int(xb), int(round(cy + (xb - cx) * vy / vx))), (0, 255, 255), 1)
    label = f"angle {e.angle:+.2f} deg  row {e.row:.1f}  left {e.left:.1f}  right {e.right:.1f}  n {e.n}  {text}" if e.ok else f"edge not found  {text}"
    cv2.rectangle(out, (0, 0), (W, 40), (0, 0, 0), -1)
    cv2.putText(out, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(path, out)
