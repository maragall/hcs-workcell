"""
The HALCON 7x7 dot calibration plate: description file, detection, ordering, orientation.

Plate frame (HALCON): origin at the plate centre, x right and y down as seen from the camera
when the plate is unrotated, z into the plate. The triangular corner mark is the chamfer on
the outer edge of the black frame at the (min x, min y) corner, i.e. top-left unrotated.

detect() returns the 49 dot centres in HALCON mark order (row by row from min y, min x) and
the matching plate points in metres, so the pair feeds solvePnP / the intrinsic fit directly.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
from scipy.ndimage import map_coordinates

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DESCR = os.path.join(HERE, "caltab_7x7.descr")      # the vendor's file: 7x7 marks at 5 mm
# The plate on the cell is HALCON's 3.125 mm dot plate: 6.25 mm pitch (vendor manual, and the
# "HC070-3.125 7x7" label in the captures). The vendor's own CamInTool only fits the captures at
# this pitch (0.3 mm plate scatter, 14 mm at the file's 5 mm), so the description is scaled to it.
PLATE_PITCH_M = 0.00625


class PlateNotFound(Exception):
    pass


@dataclass(frozen=True)
class PlateSpec:
    rows: int
    cols: int
    marks: np.ndarray            # (rows*cols, 2) x y in metres, HALCON order
    mark_radius: float           # metres
    frame_outer: Tuple[float, float, float, float]   # min x, max y, max x, min y ("i" line)
    frame_width: float           # "w" line
    triangle: Tuple[float, float, float, float]      # two corner points ("t" line)

    @property
    def pitch(self) -> float:
        return float(abs(self.marks[1, 0] - self.marks[0, 0]))

    def scaled(self, pitch_m: float) -> "PlateSpec":
        """The same plate drawn at another mark pitch (the physical plate may differ from the file)."""
        k = pitch_m / self.pitch
        return PlateSpec(self.rows, self.cols, self.marks * k, self.mark_radius * k,
                         tuple(v * k for v in self.frame_outer), self.frame_width * k,
                         tuple(v * k for v in self.triangle))

    @property
    def object_points(self) -> np.ndarray:
        """(N, 3) float64 plate points, z = 0."""
        return np.hstack([self.marks, np.zeros((len(self.marks), 1))])

    def triangle_corner(self) -> Tuple[float, float]:
        """The plate corner the chamfer cuts: the frame corner nearest the two triangle points."""
        x0, y0, x1, y1 = self.triangle
        minx, maxy, maxx, miny = self.frame_outer
        cx = minx if abs((x0 + x1) / 2 - minx) < abs((x0 + x1) / 2 - maxx) else maxx
        cy = miny if abs((y0 + y1) / 2 - miny) < abs((y0 + y1) / 2 - maxy) else maxy
        return cx, cy

    @classmethod
    def default(cls) -> "PlateSpec":
        return cls.from_descr().scaled(PLATE_PITCH_M)

    @classmethod
    def from_descr(cls, path: str = DEFAULT_DESCR) -> "PlateSpec":
        rows = cols = None
        marks, radii = [], []
        outer = tri = None
        width = None
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for raw in f:
                line = raw.split("#", 1)[0].strip()
                if not line:
                    continue
                key, *vals = line.split()
                nums = [float(v) for v in vals] if key in "rcziotw" and len(key) == 1 else None
                if key == "r":
                    rows = int(nums[0])
                elif key == "c":
                    cols = int(nums[0])
                elif key == "i":
                    outer = tuple(nums)
                elif key == "t":
                    tri = tuple(nums)
                elif key == "w":
                    width = nums[0]
                elif key in ("o", "z"):
                    pass
                else:
                    x, y, r = (float(v) for v in line.split()[:3])
                    marks.append((x, y))
                    radii.append(r)
        if rows is None or cols is None or len(marks) != rows * cols or outer is None or tri is None:
            raise ValueError(f"{path}: not a HALCON plate description")
        return cls(rows, cols, np.array(marks, float), float(np.mean(radii)), outer, width or 0.0, tri)


@dataclass
class Detection:
    image_points: np.ndarray      # (N, 2) float64, col row, subpixel
    object_points: np.ndarray     # (N, 3) float64, metres, plate frame
    homography: np.ndarray        # plate xy (m) -> image, for diagnostics


# ----------------------------------------------------------------------------- detection
def _white_masks(gray: np.ndarray):
    """Otsu first (flat lighting), then an adaptive threshold (shading, flare)."""
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    yield otsu
    block = max(51, (min(gray.shape) // 10) | 1)
    adaptive = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, block, 2)
    yield cv2.morphologyEx(adaptive, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def _candidate_interiors(gray: np.ndarray, n_marks: int):
    """White regions enclosing exactly n_marks dark holes of similar size: the inside of the frame."""
    seen = []
    for white in _white_masks(gray):
        contours, hier = cv2.findContours(white, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hier is None:
            continue
        hier = hier[0]
        for i, h in enumerate(hier):
            if h[3] != -1:                       # a hole, not an outer contour
                continue
            holes = []
            j = h[2]
            while j != -1:
                holes.append(contours[j])
                j = hier[j][0]
            areas = np.array([cv2.contourArea(c) for c in holes])
            if len(areas) < n_marks:
                continue
            a_n = np.sort(areas)[::-1][n_marks - 1]
            big = areas >= 0.5 * a_n
            if a_n < 9.0 or big.sum() != n_marks or areas.max() > 4.0 * a_n:
                continue
            seen.append([c for c, b in zip(holes, big) if b])
    return seen


def _refine_centre(gray: np.ndarray, centre: np.ndarray, radius: float, n_rays: int = 64) -> Optional[np.ndarray]:
    """Subpixel dot centre: the mid-level crossing along radial profiles, then an ellipse fit."""
    cx, cy = centre
    r_max = radius * 1.8 + 3
    steps = np.arange(0.0, r_max, 0.25)
    ang = np.linspace(0, 2 * np.pi, n_rays, endpoint=False)
    xs = cx + np.outer(np.cos(ang), steps)
    ys = cy + np.outer(np.sin(ang), steps)
    if xs.min() < 1 or ys.min() < 1 or xs.max() > gray.shape[1] - 2 or ys.max() > gray.shape[0] - 2:
        return None
    prof = map_coordinates(gray, [ys.ravel(), xs.ravel()], order=1).reshape(n_rays, -1)
    n_in, n_out = max(2, int(radius * 0.5 / 0.25)), max(2, int(0.3 * len(steps)))
    pts = []
    for k in range(n_rays):
        p = prof[k]
        mid = 0.5 * (np.median(p[:n_in]) + np.median(p[-n_out:]))    # per ray: a gradient across the dot
        above = np.nonzero(p >= mid)[0]
        if not len(above) or above[0] == 0:
            continue
        i = above[0]
        t = (mid - p[i - 1]) / max(p[i] - p[i - 1], 1e-9)
        r = steps[i - 1] + t * 0.25
        pts.append((cx + r * np.cos(ang[k]), cy + r * np.sin(ang[k])))
    if len(pts) < n_rays * 0.75:
        return None
    (ex, ey), _, _ = cv2.fitEllipse(np.array(pts, np.float32))
    return np.array([ex, ey], float)


def _order_grid(points: np.ndarray, rows: int, cols: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Index the dots on a rows x cols lattice through a homography from the four hull corners.
    Returns (lattice (i, j) per point, corner order used) or None."""
    hull = cv2.convexHull(points.astype(np.float32))
    peri = cv2.arcLength(hull, True)
    quad = cv2.approxPolyDP(hull, 0.04 * peri, True).reshape(-1, 2)
    if len(quad) != 4:
        return None
    unit = np.array([[0, 0], [cols - 1, 0], [cols - 1, rows - 1], [0, rows - 1]], np.float32)
    H = cv2.getPerspectiveTransform(unit, quad.astype(np.float32))
    ij = np.array([[j, i] for i in range(rows) for j in range(cols)], np.float32)
    pred = cv2.perspectiveTransform(ij.reshape(-1, 1, 2), H).reshape(-1, 2)
    d = np.linalg.norm(points[:, None, :] - pred[None, :, :], axis=2)
    assign = d.argmin(axis=1)
    if len(set(assign.tolist())) != rows * cols:
        return None
    pitch_px = np.median(np.linalg.norm(pred[1:cols] - pred[:cols - 1], axis=1))
    if d[np.arange(len(points)), assign].max() > 0.35 * pitch_px:
        return None
    lattice = ij[assign]
    return lattice, quad


def _orient(gray: np.ndarray, points: np.ndarray, lattice: np.ndarray, spec: PlateSpec):
    """Of the four rotations of the lattice, the one that puts the black triangle where the
    description says. Also fixes a mirrored lattice (the plate is opaque: never mirrored)."""
    rows, cols = spec.rows, spec.cols
    best = None
    for mirror in (False, True):
        lat = lattice.copy()
        if mirror:
            lat[:, 0] = cols - 1 - lat[:, 0]
        for k in range(4):
            lat_k = lat.copy()
            for _ in range(k):                           # rotate lattice 90 deg: (j, i) -> (cols-1-i, j)
                lat_k = np.stack([cols - 1 - lat_k[:, 1], lat_k[:, 0]], axis=1)
            obj = spec.marks[(lat_k[:, 1] * cols + lat_k[:, 0]).astype(int)]
            H, _ = cv2.findHomography(obj.astype(np.float32), points.astype(np.float32))
            if H is None or np.linalg.det(H[:2, :2]) <= 0:
                continue                                 # mirrored view of a front-facing plate
            samples = _corner_brightness(gray, H, spec)
            tx, ty = spec.triangle_corner()
            score = min(v for c, v in samples.items() if c != (tx, ty)) - samples[(tx, ty)]
            if best is None or score > best[0]:
                best = (score, lat_k, H)
    if best is None or best[0] <= 0:
        return None
    return best[1], best[2]


def _corner_brightness(gray, H, spec: PlateSpec):
    """Mean intensity of a small patch just inside the frame at each corner: on the black triangle
    at the marked corner, on white at the other three."""
    minx, maxy, maxx, miny = spec.frame_outer
    tx0, ty0, tx1, ty1 = spec.triangle
    leg = 0.5 * (abs(tx1 - tx0) + abs(ty1 - ty0)) - spec.frame_width   # chamfer leg from the inner corner
    inset = spec.frame_width + leg / 3.0                                 # the triangle's centroid
    out = {}
    for cx, cy in ((minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)):
        sx = cx + (inset if cx == minx else -inset)
        sy = cy + (inset if cy == miny else -inset)
        p = cv2.perspectiveTransform(np.array([[[sx, sy]]], np.float32), H).reshape(2)
        x, y = int(round(p[0])), int(round(p[1]))
        if not (1 <= x < gray.shape[1] - 1 and 1 <= y < gray.shape[0] - 1):
            out[(cx, cy)] = 255.0
            continue
        out[(cx, cy)] = float(gray[y - 1:y + 2, x - 1:x + 2].mean())
    return out


def detect(image, spec: Optional[PlateSpec] = None, refine: bool = True) -> Detection:
    """Find the plate. `image` is a path, bytes (encoded), or an array. Raises PlateNotFound."""
    spec = spec or PlateSpec.default()
    gray = _to_gray(image)
    n = spec.rows * spec.cols
    for holes in _candidate_interiors(gray, n):
        centres, radii = [], []
        for c in holes:
            m = cv2.moments(c)
            centres.append((m["m10"] / m["m00"], m["m01"] / m["m00"]))
            radii.append(np.sqrt(m["m00"] / np.pi))
        pts = np.array(centres, float)
        ordered = _order_grid(pts, spec.rows, spec.cols)
        if ordered is None:
            continue
        lattice, _ = ordered
        oriented = _orient(gray, pts, lattice, spec)
        if oriented is None:
            continue
        lattice, H = oriented
        if refine:
            pts = np.array([_refine_centre(gray, p, r) if _refine_centre(gray, p, r) is not None else p
                            for p, r in zip(pts, radii)])
        idx = (lattice[:, 1] * spec.cols + lattice[:, 0]).astype(int)
        order = np.argsort(idx)
        H, _ = cv2.findHomography(spec.marks[idx[order]].astype(np.float32), pts[order].astype(np.float32))
        return Detection(pts[order], spec.object_points[idx[order]], H)
    raise PlateNotFound(f"no {spec.rows}x{spec.cols} dot plate in the image")


def _to_gray(image) -> np.ndarray:
    if isinstance(image, (str, os.PathLike)):
        img = cv2.imread(os.fspath(image), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(image)
        return img
    if isinstance(image, (bytes, bytearray)):
        img = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise ValueError("not an image")
        return img
    arr = np.asarray(image)
    return cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY) if arr.ndim == 3 else arr
