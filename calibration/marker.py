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


# The tag on the practice plate as the cell has it (seen 2026-10-27, read by the box as id 1): a black square with a 4 x 4
# code inside, not the dictionary tag the drawing names. Interior cells from the top-left, 1 = white.
PLATE_TAG = np.array([[1, 1, 0, 1], [1, 1, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0]])
EDGE_PX = 12
TAG_GRID, TAG_PX, MAX_CODE_MISMATCH = 6, 24, 2          # 6 x 6 cells with the border; warped 24 px a cell


def _order(quad: np.ndarray) -> np.ndarray:
    """Corners as top-left, top-right, bottom-right, bottom-left of the image."""
    s, d = quad.sum(1), np.diff(quad, axis=1).ravel()
    return np.array([quad[np.argmin(s)], quad[np.argmin(d)], quad[np.argmax(s)], quad[np.argmax(d)]], np.float32)


# The black border around the code is not always a whole cell wide (the practice plate's is about two thirds of one): the
# interior is read for each width and the first that is dark all round is used.
BORDER_CELLS = (1.0, 0.66, 0.8, 0.5)


def _flatten(warped: np.ndarray) -> np.ndarray:
    """The warped tag with the slowly varying light divided out (glare, a shadow edge), stretched to 0..255."""
    f = warped.astype(np.float32) / (cv2.GaussianBlur(warped.astype(np.float32), (0, 0), warped.shape[0] / 5.0) + 1.0)
    return cv2.normalize(f, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


def _inner_codes(warped: np.ndarray, mid: float):
    side = warped.shape[0]
    for r in BORDER_CELLS:
        cell = side / (4 + 2 * r)
        b = r * cell
        ring = int(max(2, b * 0.6))
        edge = np.concatenate([warped[:ring].ravel(), warped[-ring:].ravel(), warped[:, :ring].ravel(), warped[:, -ring:].ravel()])
        if (edge > mid).mean() > 0.15:                # the border is black
            continue
        bits = np.zeros((4, 4), int)
        for i in range(4):
            for j in range(4):
                y0, x0 = b + i * cell, b + j * cell
                patch = warped[int(y0 + cell * 0.3):int(y0 + cell * 0.7), int(x0 + cell * 0.3):int(x0 + cell * 0.7)]
                bits[i, j] = int(patch.mean() > mid)
        yield bits


def detect_plate_tag(gray: np.ndarray) -> np.ndarray:
    """(4, 2) subpixel corners of the plate's own tag in the marker's order (so the corners mean the same thing in every
    picture, whatever the yaw): find dark convex quadrilaterals, warp each, accept the one whose border is dark and whose
    4 x 4 interior matches PLATE_TAG in one of the four turns, and turn the corners by that."""
    small = cv2.GaussianBlur(gray, (5, 5), 0)
    _, dark = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(dark, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    side = TAG_GRID * TAG_PX
    target = np.array([[0, 0], [side, 0], [side, side], [0, side]], np.float32)
    for c in sorted(contours, key=cv2.contourArea, reverse=True)[:12]:
        if cv2.contourArea(c) < 0.002 * gray.size:
            break
        quad = cv2.approxPolyDP(c, 0.03 * cv2.arcLength(c, True), True)
        if len(quad) != 4 or not cv2.isContourConvex(quad):
            continue
        q = _order(quad.reshape(4, 2).astype(np.float32))
        h, w = gray.shape
        if (q[:, 0] < EDGE_PX).any() or (q[:, 1] < EDGE_PX).any() or (q[:, 0] > w - 1 - EDGE_PX).any() or (q[:, 1] > h - 1 - EDGE_PX).any():
            continue                              # a tag cut by the picture's edge has the edge for a side: wrong corners
        warped = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(q, target), (side, side))
        warped = _flatten(warped)                     # a glare patch lifts the black cells: divide the smooth light out
        mid = float(cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0])
        for inner in _inner_codes(warped, mid):
            for k in range(4):
                if int((np.rot90(inner, k) != PLATE_TAG).sum()) <= MAX_CODE_MISMATCH:
                    q = np.roll(q, -k, axis=0)        # k quarter turns of the code: the first corner moves with it
                    ref = q.reshape(4, 1, 2).copy()
                    cv2.cornerSubPix(gray, ref, (7, 7), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
                    return ref.reshape(4, 2).astype(float)
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
