"""
One joint solve from the marker captures: camera model, camera-in-tool, marker-in-base.

Knowns per capture i: the arm's pose (base->tool T_bt_i, from NNN.json) and the marker's four
corners in the image (marker.detect). Unknowns: HALCON area_scan_polynomial parameters
(Focus, K1..K3, P1, P2, Sx, Cx, Cy; Sy is the sensor cell and stays), T_tc (camera in tool,
6) and T_bm (marker in base, 6). Residuals: project(T_tc^-1 T_bt_i^-1 T_bm corners) minus
the detected corners, in pixels, scipy least squares.

The arm's known motions make it well posed with one planar target: z steps give the focal
length and the principal point, the x, y spread the distortion, the yaws the hand-eye
rotation. A SCARA cannot fix the camera's offset along tool z (only its sum with the marker's
height), so the marker is pinned at base z = 0, as the vendor's CamInTool implies, and the
result names "tool z" as unobservable.

Start: the camera's previous CamParam (the box always has one), solvePnP for the marker
poses, the closed form of handeye_start for T_tc, the mean for T_bm.
"""
from __future__ import annotations

import glob
import json
import os
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

from calibration import halcon, marker

IMX219_CELL_M = 1.12e-6


@dataclass
class Captures:
    T_bt: List[np.ndarray]
    corners: List[np.ndarray]          # (4, 2) each
    files: List[str]
    width: int
    height: int
    skipped: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class Result:
    camparam: halcon.CamParam
    camintool: halcon.Pose
    marker_in_base: halcon.Pose
    reprojection_px: float
    mean_error_m: float              # mean over captures of the marker centre error on the tab, in m
    images: int
    unobservable: Tuple[str, ...] = ()
    skipped: Tuple[str, ...] = ()

    def write(self, out_dir: str) -> Tuple[str, str]:
        os.makedirs(out_dir, exist_ok=True)
        a, b = os.path.join(out_dir, "CamParam.dat"), os.path.join(out_dir, "CamInTool.dat")
        self.camparam.write(a)
        self.camintool.write(b)
        return a, b


# ------------------------------------------------------------------ poses
def tool_matrix(pose_mm_deg: Sequence[float]) -> np.ndarray:
    """Arm pose (x y z mm, rx ry rz deg) -> base->tool 4x4 in metres, R = Rz Ry Rx."""
    x, y, z, rx, ry, rz = (float(v) for v in pose_mm_deg)
    T = np.eye(4)
    T[:3, :3] = halcon.rot_z(np.radians(rz)) @ halcon.rot_y(np.radians(ry)) @ halcon.rot_x(np.radians(rx))
    T[:3, 3] = [x / 1e3, y / 1e3, z / 1e3]
    return T


def _inv(T):
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R.T, -R.T @ t
    return out


def _to_vec(T):
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


def _to_mat(v):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(v[:3], float))[0]
    T[:3, 3] = v[3:6]
    return T


def load_folder(folder: str, marker_id: Optional[int] = marker.MARKER_ID) -> Captures:
    """NNN.png + NNN.json ({"pose": [...]}) from capture.run_plan; captures without the marker are skipped."""
    T_bt, corners, files, skipped, size = [], [], [], [], None
    for f in sorted(glob.glob(os.path.join(folder, "*.png"))):
        meta = os.path.splitext(f)[0] + ".json"
        if not os.path.exists(meta):
            continue
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        size = size or (img.shape[1], img.shape[0])
        try:
            c = marker.detect(img, marker_id)
        except marker.MarkerNotFound:
            skipped.append(os.path.basename(f))
            continue
        with open(meta, "r", encoding="utf-8") as fh:
            T_bt.append(tool_matrix(json.load(fh)["pose"]))
        corners.append(c)
        files.append(f)
    if len(files) < 6:
        raise marker.MarkerNotFound(f"{folder}: marker seen in {len(files)} captures, need at least 6")
    return Captures(T_bt, corners, files, *size, skipped)


# ------------------------------------------------------------------ start values
def marker_pose(corners: np.ndarray, cam: halcon.CamParam, size_m: float) -> np.ndarray:
    """cam->marker 4x4 from one capture."""
    ok, rvec, tvec = cv2.solvePnP(marker.object_points(size_m), cam.normalized(corners), cam.K, None,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise marker.MarkerNotFound("solvePnP failed")
    return _to_mat(np.concatenate([rvec.ravel(), tvec.ravel()]))


def _motions(T_bt, T_cm):
    return [(_inv(T_bt[i]) @ T_bt[j], T_cm[i] @ _inv(T_cm[j])) for i in range(len(T_bt)) for j in range(i + 1, len(T_bt))]


def _translation(R, motions):
    C = np.vstack([A[:3, :3] - np.eye(3) for A, _ in motions])
    d = np.concatenate([R @ B[:3, 3] - A[:3, 3] for A, B in motions])
    t, *_ = np.linalg.lstsq(C, d, rcond=None)
    return t, float(np.linalg.norm(C @ t - d))


def _align(b, a):
    v = np.cross(b, a)
    s, c = np.linalg.norm(v), float(np.dot(b, a))
    if s < 1e-12:
        return np.eye(3) if c > 0 else cv2.Rodrigues(np.pi * np.cross(a, [1, 0, 0] if abs(a[0]) < 0.9 else [0, 1, 0]))[0]
    return cv2.Rodrigues(v / s * np.arctan2(s, c))[0]


def handeye_start(T_bt, T_cm) -> Tuple[np.ndarray, bool]:
    """AX = XB closed form, X = T_tc. Returns (X, single_axis). With one rotation axis (a SCARA) the
    axis fixes two angles and a sweep of the third on the translation equations the rest."""
    motions = _motions(T_bt, T_cm)
    alphas = np.array([cv2.Rodrigues(A[:3, :3])[0].ravel() for A, _ in motions])
    betas = np.array([cv2.Rodrigues(B[:3, :3])[0].ravel() for _, B in motions])
    sv = np.linalg.svd(alphas, compute_uv=False)
    single_axis = sv[1] <= 1e-3 * sv[0]
    if not single_axis:
        from scipy.linalg import sqrtm
        M = betas.T @ alphas
        R = np.real(np.linalg.inv(sqrtm(M.T @ M))) @ M.T
    else:
        big = np.linalg.norm(alphas, axis=1) > 0.02
        if not big.any():
            raise np.linalg.LinAlgError("no rotation between the captures")
        a = np.mean(alphas[big] / np.linalg.norm(alphas[big], axis=1, keepdims=True), axis=0)
        b = np.mean(betas[big] / np.linalg.norm(betas[big], axis=1, keepdims=True), axis=0)
        a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
        R0 = _align(b, a)
        R = min((cv2.Rodrigues(a * np.radians(th))[0] @ R0 for th in range(0, 360, 3)),
                key=lambda Rk: _translation(Rk, motions)[1])
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt if np.linalg.det(U @ Vt) > 0 else U @ np.diag([1, 1, -1]) @ Vt
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, _translation(R, motions)[0]
    return X, single_axis


# ------------------------------------------------------------------ the joint solve
def _unpack(x, sy, width, height, pin_z):
    focus, k1, k2, k3, p1, p2, sx, cx, cy = x[:9]
    cam = halcon.CamParam(focus, sx, sy, cx, cy, width, height, "area_scan_polynomial", (k1, k2, k3, p1, p2))
    T_tc = _to_mat(x[9:15])
    T_bm = _to_mat(np.concatenate([x[15:20], [0.0]]) if pin_z else x[15:21])
    return cam, T_tc, T_bm


def _residuals(x, caps: Captures, obj, sy, pin_z):
    cam, T_tc, T_bm = _unpack(x, sy, caps.width, caps.height, pin_z)
    T_ct = _inv(T_tc)
    out = []
    for T, c in zip(caps.T_bt, caps.corners):
        P = T_ct @ _inv(T) @ T_bm
        out.append(cam.project(obj @ P[:3, :3].T + P[:3, 3]) - c)
    return np.concatenate(out).ravel()


def run(folder: str, start: halcon.CamParam, marker_size_m: float, marker_id: Optional[int] = marker.MARKER_ID,
        sy: float = IMX219_CELL_M) -> Result:
    caps = load_folder(folder, marker_id)
    return fit(caps, start, marker_size_m, sy)


def fit(caps: Captures, start: halcon.CamParam, marker_size_m: float, sy: float = IMX219_CELL_M) -> Result:
    obj = marker.object_points(marker_size_m)
    T_cm = [marker_pose(c, start, marker_size_m) for c in caps.corners]
    X0, single_axis = handeye_start(caps.T_bt, T_cm)
    bm0 = np.mean([_to_vec(T @ X0 @ C) for T, C in zip(caps.T_bt, T_cm)], axis=0)
    if single_axis:
        X0[2, 3] -= bm0[5]                     # pin the marker at base z = 0: the hand carries the difference
        bm0 = bm0[:5]
    poly = start.poly if start.model == "area_scan_polynomial" else (0, 0, 0, 0, 0)
    x0 = np.concatenate([[start.focus, *poly, start.sx, start.cx, start.cy], _to_vec(X0), bm0])
    scale = np.concatenate([[1e-3, 1e4, 1e9, 1e14, 1e2, 1e2, 1e-6, 1e3, 1e3], [1, 1, 1, 0.1, 0.1, 0.1],
                            [1, 1, 1, 0.1, 0.1, 0.1][: len(bm0)]])
    res = least_squares(_residuals, x0, args=(caps, obj, sy, single_axis), x_scale=scale, method="trf",
                        xtol=1e-12, ftol=1e-12, max_nfev=300)
    cam, T_tc, T_bm = _unpack(res.x, sy, caps.width, caps.height, single_axis)
    px = float(np.sqrt(np.mean(np.sum(res.fun.reshape(-1, 2) ** 2, axis=1))))
    T_ct = _inv(T_tc)
    errs = [np.linalg.norm((T_ct @ _inv(T) @ T_bm)[:3, 3] - marker_pose(c, cam, marker_size_m)[:3, 3])
            for T, c in zip(caps.T_bt, caps.corners)]
    return Result(cam.with_error(px), halcon.Pose.from_matrix(T_tc), halcon.Pose.from_matrix(T_bm), px,
                  float(np.mean(errs)), len(caps.files), ("tool z",) if single_axis else (), tuple(caps.skipped))
