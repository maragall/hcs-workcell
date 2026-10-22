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


def detect(image, marker_id: Optional[int] = MARKER_ID) -> np.ndarray:
    """(4, 2) subpixel corners (col, row) of the marker; path, encoded bytes or array in."""
    gray = _to_gray(image)
    corners, ids, _ = _detector().detectMarkers(gray)
    if ids is None:
        raise MarkerNotFound("no marker in the image")
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
