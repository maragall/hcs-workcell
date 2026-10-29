"""
The calibration target: the 40 mm ArUco marker on the practice plate's tab (layout.json
vision._marker_note: DICT_ARUCO_ORIGINAL id 682, which the box addresses as id 1).

Marker frame (OpenCV's): origin at the centre, x right, y up, z out of the tab towards the
camera. Corners in detection order: top-left, top-right, bottom-right, bottom-left.
"""
from __future__ import annotations

import os
from typing import Optional

import cv2
import numpy as np

DICTIONARY = cv2.aruco.DICT_ARUCO_ORIGINAL
MARKER_ID = 682                    # the drawing's id; the box numbers it 1


class MarkerNotFound(Exception):
    pass


def object_points(size_m: float) -> np.ndarray:
    """(4, 3) corners in the marker frame, detection order."""
    h = size_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def _detector() -> "cv2.aruco.ArucoDetector":
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 7
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY), params)


# The tag on the practice plate (drawing aruco标定板: a 40 x 40 mm black frame; read from the real pictures 2026-10-28): 8 x 8 cells of
# 5 mm, a black border one cell wide around a 6 x 6 code, not a dictionary tag OpenCV knows. Interior cells from the top-left,
# 1 = white. The box reads it as id 1.
PLATE_TAG = np.array([[1, 1, 1, 0, 0, 0], [1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 0, 1], [0, 0, 0, 1, 1, 0], [0, 0, 1, 1, 0, 0], [0, 1, 1, 0, 0, 1]])
EDGE_PX = 12
TAG_CELLS, TAG_PX, MAX_CODE_MISMATCH = 8, 24, 3         # 8 x 8 cells with the border; warped 24 px a cell; a glare cell or two is allowed
CORRELATION_MIN = 0.6


def _order(quad: np.ndarray) -> np.ndarray:
    """Corners as top-left, top-right, bottom-right, bottom-left of the image."""
    s, d = quad.sum(1), np.diff(quad, axis=1).ravel()
    return np.array([quad[np.argmin(s)], quad[np.argmin(d)], quad[np.argmax(s)], quad[np.argmax(d)]], np.float32)


def _destripe(gray: np.ndarray) -> np.ndarray:
    """The picture with row-to-row brightness ripple taken out (a flickering light against a rolling shutter lays bright and dark
    bands across it, and a band over a corner of the tag eats the corner): each row is divided by its own gain, the ratio of its
    bright pixels' level to the smoothed level over neighbouring rows."""
    rows = np.percentile(gray, 75, axis=1).astype(np.float32)
    gain = rows / (cv2.GaussianBlur(rows.reshape(-1, 1), (0, 0), 25).ravel() + 1.0)
    out = gray.astype(np.float32) / np.clip(gain, 0.5, 2.0)[:, None]
    return np.clip(out, 0, 255).astype(np.uint8)


def _by_edge_lines(gray: np.ndarray, corners: np.ndarray, per_side: int = 60, reach_px: float = 0.03) -> np.ndarray:
    """The tag's corners from its four straight edges instead of one corner pixel each: on each side the edge is located to a
    fraction of a pixel along `per_side` lines across it (the black-to-white step), a robust line is fitted to those points,
    and neighbouring lines are intersected. Hundreds of edge pixels set each corner, where a corner detector uses a few and is
    pulled by blur and glare. Falls back to the given corners when a fit looks wrong (a corner moved more than 6 px, or fewer
    than half the lines found an edge)."""
    c = np.asarray(corners, float)
    side_len = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
    reach = max(6.0, reach_px * side_len)
    centre = c.mean(axis=0)
    img = gray.astype(np.float32)
    lines = []
    for i in range(4):
        a, b = c[i], c[(i + 1) % 4]
        d = (b - a) / (np.linalg.norm(b - a) + 1e-9)
        n = np.array([-d[1], d[0]])
        if np.dot(n, (a + b) / 2 - centre) < 0:
            n = -n                                              # outward
        pts = []
        for t in np.linspace(0.18, 0.82, per_side):
            p0 = a + t * (b - a)
            offs = np.arange(-reach, reach + 0.01, 0.5)
            xy = p0[None, :] + offs[:, None] * n[None, :]
            prof = cv2.remap(img, xy[:, 0:1].astype(np.float32), xy[:, 1:2].astype(np.float32), cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REPLICATE).ravel()
            prof = np.convolve(prof, np.ones(3) / 3, mode="same")
            g = np.gradient(prof)
            j = int(np.argmax(g[2:-2])) + 2                     # black inside to white outside: the steepest rise going outward
            if g[j] < 8.0 or j in (2, len(g) - 3):
                continue
            den = g[j - 1] - 2 * g[j] + g[j + 1]
            sub = 0.5 * (g[j - 1] - g[j + 1]) / den if abs(den) > 1e-6 else 0.0
            pts.append(p0 + (offs[j] + float(np.clip(sub, -1, 1)) * 0.5) * n)
        if len(pts) < per_side // 2:
            return c
        vx, vy, x0, y0 = cv2.fitLine(np.array(pts, np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        lines.append((np.array([x0, y0], float), np.array([vx, vy], float)))
    out = []
    for i in range(4):
        (p, u), (q, v) = lines[i - 1], lines[i]                 # corner i is where side i-1 meets side i
        m = np.array([[u[0], -v[0]], [u[1], -v[1]]])
        if abs(np.linalg.det(m)) < 1e-6:
            return c
        s = np.linalg.solve(m, q - p)
        out.append(p + s[0] * u)
    out = np.array(out)
    return out if np.abs(out - c).max() < 6.0 else c


def _cells(warped: np.ndarray) -> np.ndarray:
    """(6, 6) mean level of each code cell (the middle half of the cell), from the warped 8 x 8 square."""
    c = TAG_PX
    lo, hi = int(c * 0.25), int(c * 0.75)
    return np.array([[warped[(i + 1) * c + lo:(i + 1) * c + hi, (j + 1) * c + lo:(j + 1) * c + hi].mean() for j in range(6)]
                     for i in range(6)])


def _border_dark(warped: np.ndarray, mid: float) -> bool:
    c = TAG_PX
    ring = np.concatenate([warped[:c].ravel(), warped[-c:].ravel(), warped[:, :c].ravel(), warped[:, -c:].ravel()])
    return float((ring > mid).mean()) <= 0.15


def _flatten(warped: np.ndarray) -> np.ndarray:
    """The warped tag with the slowly varying light divided out (glare, a shadow edge), stretched to 0..255."""
    f = warped.astype(np.float32) / (cv2.GaussianBlur(warped.astype(np.float32), (0, 0), warped.shape[0] / 5.0) + 1.0)
    return cv2.normalize(f, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


def _four_corners(contour: np.ndarray) -> Optional[np.ndarray]:
    """The 4 corners of a dark blob: the outline's convex hull simplified until it has 4 points (glare on the black or a
    shadow edge leaves notches that make the plain outline 6 or 8 points; the hull closes them)."""
    hull = cv2.convexHull(contour)
    for eps in (0.02, 0.03, 0.045, 0.06, 0.08):
        quad = cv2.approxPolyDP(hull, eps * cv2.arcLength(hull, True), True)
        if len(quad) == 4:
            return quad
        if len(quad) < 4:
            return None
    return None


def _local_bits(warped: np.ndarray, means: np.ndarray) -> np.ndarray:
    """The cells with a threshold that follows the light across the tag (a shaded side dims the whites and the blacks together):
    the black level at each cell is read from the border on its row and column, the white level is the bright cells' level."""
    c, n = TAG_PX, warped.shape[0]
    mid = [slice((i + 1) * c + int(c * .25), (i + 1) * c + int(c * .75)) for i in range(6)]
    rows = np.array([(warped[m, 4:c - 4].mean() + warped[m, n - c + 4:n - 4].mean()) / 2 for m in mid])
    cols = np.array([(warped[4:c - 4, m].mean() + warped[n - c + 4:n - 4, m].mean()) / 2 for m in mid])
    black = (rows[:, None] + cols[None, :]) / 2
    white = float(np.percentile(means, 70))
    return (means > black + 0.35 * np.maximum(white - black, 1.0)).astype(int)


def _correlation_turn(warped: np.ndarray) -> Optional[int]:
    """Quarter turns k at which the warped square correlates with the tag (black border, PLATE_TAG inside) above CORRELATION_MIN:
    glare brightens black cells past any threshold, the pattern still correlates. None when no turn does."""
    img = warped.astype(np.float32)
    best, best_k = CORRELATION_MIN, None
    for k in range(4):
        code = np.rot90(PLATE_TAG, -k)
        tpl = np.zeros(img.shape, np.float32)
        for i in range(6):
            for j in range(6):
                if code[i, j]:
                    tpl[(i + 1) * TAG_PX:(i + 2) * TAG_PX, (j + 1) * TAG_PX:(j + 2) * TAG_PX] = 1.0
        score = float(cv2.matchTemplate(img, tpl, cv2.TM_CCOEFF_NORMED)[0, 0])
        if score > best:
            best, best_k = score, k
    return best_k


def detect_plate_tag(gray: np.ndarray) -> np.ndarray:
    """(4, 2) subpixel corners of the plate's own tag in the marker's order (so the corners mean the same thing in every
    picture, whatever the yaw): find dark convex quadrilaterals, warp each, accept the one whose border is dark and whose
    6 x 6 interior matches PLATE_TAG (a few cells off allowed, or correlating with it) in one of the four turns, and turn
    the corners by that."""
    gray = _destripe(gray)
    small = cv2.GaussianBlur(gray, (5, 5), 0)
    _, dark = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(dark, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # a second mask against the local light: glare that lifts a corner of the black above the global threshold still looks dark
    # next to the white tab around it, and the outline then follows the tag's real edge
    rel = small.astype(np.float32) / (cv2.GaussianBlur(small.astype(np.float32), (0, 0), gray.shape[0] / 8.0) + 1.0)
    _, dark2 = cv2.threshold(cv2.normalize(rel, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8), 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours = list(contours) + list(cv2.findContours(dark2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0])
    side = TAG_CELLS * TAG_PX
    target = np.array([[0, 0], [side, 0], [side, side], [0, side]], np.float32)
    h, w = gray.shape
    for c in sorted(contours, key=cv2.contourArea, reverse=True)[:12]:
        if cv2.contourArea(c) < 0.002 * gray.size:
            break
        quad = _four_corners(c)
        if quad is None:
            continue
        q = _order(quad.reshape(4, 2).astype(np.float32))
        if (q[:, 0] < EDGE_PX).any() or (q[:, 1] < EDGE_PX).any() or (q[:, 0] > w - 1 - EDGE_PX).any() or (q[:, 1] > h - 1 - EDGE_PX).any():
            continue                              # a tag cut by the picture's edge has the edge for a side: wrong corners
        raw = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(q, target), (side, side))
        flat = _flatten(raw)
        mid = float(cv2.threshold(flat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0])
        if not _border_dark(flat, mid):
            continue
        means = _cells(flat)
        k = None
        for bits in ((means > mid).astype(int), _local_bits(flat, means)):
            for turn in range(4):
                if int((np.rot90(bits, turn) != PLATE_TAG).sum()) <= MAX_CODE_MISMATCH:
                    k = turn
                    break
            if k is not None:
                break
        if k is None:
            k = _correlation_turn(raw)
        if k is None:
            continue
        q = np.roll(q, -k, axis=0)                # k quarter turns of the code: the first corner moves with it
        ref = q.reshape(4, 1, 2).copy()
        cv2.cornerSubPix(gray, ref, (7, 7), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        return _by_edge_lines(gray, ref.reshape(4, 2).astype(float))
    raise MarkerNotFound("no plate tag in the image")


def detect(image, marker_id: Optional[int] = MARKER_ID) -> np.ndarray:
    """(4, 2) subpixel corners (col, row) of the marker; path, encoded bytes or array in. The dictionary tag the drawing
    names first, then the plate's own tag."""
    gray = _to_gray(image)
    corners, ids, _ = _detector().detectMarkers(gray)
    if ids is None:
        return detect_plate_tag(gray)
    ids = ids.ravel().tolist()
    if marker_id is not None and marker_id in ids:
        return corners[ids.index(marker_id)].reshape(4, 2).astype(float)
    if marker_id is None and len(ids) == 1:
        return corners[0].reshape(4, 2).astype(float)
    raise MarkerNotFound(f"marker {marker_id} not in the image (seen {ids})")


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
