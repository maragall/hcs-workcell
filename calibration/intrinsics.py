"""
Intrinsic calibration: HALCON area_scan_polynomial (or division) from the plate captures.

solve(folder) detects the plate in every *.png, starts from a pinhole guess (cv2.calibrateCamera
on the normalized dots) and refines Focus, K1..K3, P1, P2, Sx, Cx, Cy plus one plate pose per
image with scipy least squares on the pixel residuals. Sy is the sensor's cell height and is
kept fixed, as HALCON does for area scan cameras (Focus and Sx carry the pixel aspect).
"""
from __future__ import annotations

import glob
import os
from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

from calibration import halcon
from calibration.plate import Detection, PlateNotFound, PlateSpec, detect

IMX219_CELL_M = 1.12e-6          # Sy of the box's cameras (Sony IMX219, 3280x2464), fixed in the fit


@dataclass
class Observations:
    detections: List[Detection]
    files: List[str]
    width: int
    height: int


def observe(folder: str, spec: Optional[PlateSpec] = None, pattern: str = "*.png",
            limit: Optional[int] = None) -> Observations:
    """Detect the plate in every image of a vendor-style folder (the first `limit` of them);
    images without a plate are skipped."""
    spec = spec or PlateSpec.default()
    dets, files, size = [], [], None
    for f in sorted(glob.glob(os.path.join(folder, pattern)))[:limit]:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        size = size or (img.shape[1], img.shape[0])
        try:
            dets.append(detect(img, spec))
            files.append(f)
        except PlateNotFound:
            continue
    if len(dets) < 3:
        raise PlateNotFound(f"{folder}: plate found in {len(dets)} images, need at least 3")
    return Observations(dets, files, *size)


# ---------------------------------------------------------------------- the fit
def _unpack(x, model: str, sy: float, width: int, height: int) -> Tuple[halcon.CamParam, np.ndarray]:
    if model == "area_scan_polynomial":
        focus, k1, k2, k3, p1, p2, sx, cx, cy = x[:9]
        cam = halcon.CamParam(focus, sx, sy, cx, cy, width, height, model, (k1, k2, k3, p1, p2))
        rest = x[9:]
    else:
        focus, kappa, sx, cx, cy = x[:5]
        cam = halcon.CamParam(focus, sx, sy, cx, cy, width, height, model, kappa=kappa)
        rest = x[5:]
    return cam, rest.reshape(-1, 6)


def _residuals(x, model, sy, width, height, obj_list, img_list):
    cam, poses = _unpack(x, model, sy, width, height)
    out = []
    for (rvec, tvec), obj, img in zip(poses.reshape(-1, 2, 3), obj_list, img_list):
        R, _ = cv2.Rodrigues(rvec)
        out.append(cam.project(obj @ R.T + tvec) - img)
    return np.concatenate(out).ravel()


def _pinhole_start(obs: Observations, sy: float) -> Tuple[halcon.CamParam, np.ndarray]:
    """cv2.calibrateCamera for the start values; HALCON's units from the fixed cell height."""
    obj = [d.object_points.astype(np.float32) for d in obs.detections]
    img = [d.image_points.astype(np.float32).reshape(-1, 1, 2) for d in obs.detections]
    flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
    _, K, _, rvecs, tvecs = cv2.calibrateCamera(obj, img, (obs.width, obs.height), None, None, flags=flags)
    focus = K[1, 1] * sy
    cam = halcon.CamParam(focus, focus / K[0, 0], sy, K[0, 2], K[1, 2], obs.width, obs.height)
    poses = np.array([np.concatenate([r.ravel(), t.ravel()]) for r, t in zip(rvecs, tvecs)])
    return cam, poses


def fit(obs: Observations, model: str = "area_scan_polynomial", sy: float = IMX219_CELL_M) -> halcon.CamParam:
    start, poses = _pinhole_start(obs, sy)
    if model == "area_scan_polynomial":
        x0 = [start.focus, 0, 0, 0, 0, 0, start.sx, start.cx, start.cy]
        scale = [1e-3, 1e4, 1e9, 1e14, 1e2, 1e2, 1e-6, 1e3, 1e3]
    elif model == "area_scan_division":
        x0 = [start.focus, 0, start.sx, start.cx, start.cy]
        scale = [1e-3, 1e4, 1e-6, 1e3, 1e3]
    else:
        raise ValueError(model)
    x0 = np.concatenate([x0, poses.ravel()])
    scale = np.concatenate([scale, np.tile([1, 1, 1, 0.1, 0.1, 0.1], len(poses))])
    obj = [d.object_points for d in obs.detections]
    img = [d.image_points for d in obs.detections]
    res = least_squares(_residuals, x0, args=(model, sy, obs.width, obs.height, obj, img),
                        x_scale=scale, method="trf", xtol=1e-12, ftol=1e-12, max_nfev=400)
    cam, _ = _unpack(res.x, model, sy, obs.width, obs.height)
    return cam.with_error(float(np.sqrt(np.mean(np.sum(res.fun.reshape(-1, 2) ** 2, axis=1)))))


def reprojection(cam: halcon.CamParam, det: Detection) -> Tuple[float, np.ndarray]:
    """RMS pixel error of one detection under `cam`, with the plate pose from solvePnP."""
    ok, rvec, tvec = cv2.solvePnP(det.object_points, cam.normalized(det.image_points), cam.K, None,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    R, _ = cv2.Rodrigues(rvec)
    d = cam.project(det.object_points @ R.T + tvec.ravel()) - det.image_points
    return float(np.sqrt((d ** 2).sum(axis=1).mean())), np.concatenate([rvec.ravel(), tvec.ravel()])


def solve(folder: str, spec: Optional[PlateSpec] = None, model: str = "area_scan_polynomial",
          outlier_ratio: float = 2.0, limit: Optional[int] = None) -> halcon.CamParam:
    """The vendor's intrinsic step: CamParam from a folder of plate captures (the first `limit`,
    i.e. the plan's). Captures whose own RMS is more than outlier_ratio times the median are
    dropped and the fit repeated; `notes` says which."""
    obs = observe(folder, spec, limit=limit)
    cam = fit(obs, model)
    per = np.array([reprojection(cam, d)[0] for d in obs.detections])
    keep = per <= outlier_ratio * np.median(per)
    if keep.all() or keep.sum() < 3:
        return replace(cam, notes=f"{len(obs.detections)} captures")
    kept = Observations([d for d, k in zip(obs.detections, keep) if k], [f for f, k in zip(obs.files, keep) if k],
                        obs.width, obs.height)
    dropped = ", ".join(f"{os.path.basename(f)} ({e:.2f} px)" for f, e, k in zip(obs.files, per, keep) if not k)
    return replace(fit(kept, model), notes=f"{keep.sum()} of {len(keep)} captures; dropped {dropped}")
