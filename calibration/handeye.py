"""
Eye-in-hand calibration: the camera's pose in the tool frame (CamInTool.dat).

Inputs are the vendor's HandEye folder: handeye_NNN.png with the plate fixed in the cell and
pose_NNN.txt, the arm's tool pose "x,y,z,rx,ry,rz" (mm, deg) at the capture.

    base->tool   T_bt = [Rz(rz) Ry(ry) Rx(rx) | t]      (the arm reports rz only on a SCARA)
    cam->plate   T_cp  from solvePnP with the fitted intrinsics
    unknowns     T_tc (camera in tool) and T_bp (plate in base), with T_bp = T_bt T_tc T_cp

Start: a closed form on the pairwise motions (closed_form). Refine: scipy least squares over
T_tc and T_bp on the per-image pose residual, translation in metres plus rotation in radians
times the camera-plate distance. mean_error_m is the mean over images of
|t_cp(observed) - t_cp(model)|; the vendor's log calls the same thing "mean error".

A SCARA only rotates about base z, so the camera's offset along tool z and the plate's height
are one unknown: the captures fix their sum, not each. The vendor's CamInTool puts the plate
at base z = 0 (validate.py shows it to 0.3 mm), so with a single rotation axis the plate's z
is pinned at 0 and the report names "tool z" as unobservable. Everything else is measured.
"""
from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.linalg import sqrtm
from scipy.optimize import least_squares

from calibration import halcon
from calibration.plate import Detection, PlateNotFound, PlateSpec, detect



@dataclass(frozen=True)
class CamInTool:
    pose: halcon.Pose
    mean_error_m: float
    plate_in_base: halcon.Pose
    images: int
    unobservable: Tuple[str, ...] = ()        # directions the captures cannot fix
    dropped: Tuple[str, ...] = ()             # captures rejected as a moved plate

    @property
    def matrix(self) -> np.ndarray:
        return self.pose.matrix

    def write(self, path: str, when: Optional[str] = None) -> None:
        self.pose.write(path, when)


# ------------------------------------------------------------------ helpers
def tool_matrix(pose_mm_deg: Sequence[float]) -> np.ndarray:
    """Arm pose (x y z mm, rx ry rz deg) -> base->tool 4x4 in metres."""
    x, y, z, rx, ry, rz = (float(v) for v in pose_mm_deg)
    T = np.eye(4)
    T[:3, :3] = halcon.rot_z(np.radians(rz)) @ halcon.rot_y(np.radians(ry)) @ halcon.rot_x(np.radians(rx))
    T[:3, 3] = [x / 1e3, y / 1e3, z / 1e3]
    return T


def read_pose_file(path: str) -> List[float]:
    with open(path, "r", encoding="utf-8") as f:
        vals = [float(v) for v in f.read().replace("\n", "").split(",") if v.strip()]
    if len(vals) != 6:
        raise ValueError(f"{path}: expected 6 values, got {len(vals)}")
    return vals


def plate_pose(det: Detection, cam: halcon.CamParam) -> np.ndarray:
    """cam->plate 4x4 from one detection."""
    ok, rvec, tvec = cv2.solvePnP(det.object_points, cam.normalized(det.image_points), cam.K, None,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise PlateNotFound("solvePnP failed")
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T


def _inv(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def _to_vec(T: np.ndarray) -> np.ndarray:
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


def _to_mat(v: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(v[:3], float))[0]
    T[:3, 3] = v[3:6]
    return T


# ------------------------------------------------------------------ closed form
def _motions(T_bt, T_cp):
    """All pairwise (A, B) with A = tool motion in the tool frame, B = camera motion."""
    out = []
    for i in range(len(T_bt)):
        for j in range(i + 1, len(T_bt)):
            out.append((_inv(T_bt[i]) @ T_bt[j], T_cp[i] @ _inv(T_cp[j])))
    return out


def _translation(R, motions):
    C = np.vstack([A[:3, :3] - np.eye(3) for A, _ in motions])
    d = np.concatenate([R @ B[:3, 3] - A[:3, 3] for A, B in motions])
    t, *_ = np.linalg.lstsq(C, d, rcond=None)
    return t, float(np.linalg.norm(C @ t - d))


def _align(b, a):
    """Rotation taking unit vector b onto unit vector a."""
    v = np.cross(b, a)
    s, c = np.linalg.norm(v), float(np.dot(b, a))
    if s < 1e-12:
        return np.eye(3) if c > 0 else cv2.Rodrigues(np.pi * _any_normal(a))[0]
    return cv2.Rodrigues(v / s * np.arctan2(s, c))[0]


def _any_normal(a):
    n = np.cross(a, [1, 0, 0] if abs(a[0]) < 0.9 else [0, 1, 0])
    return n / np.linalg.norm(n)


def closed_form(T_bt: List[np.ndarray], T_cp: List[np.ndarray]) -> Tuple[np.ndarray, bool]:
    """AX = XB start value, X = T_tc. Park and Martin when the arm rotated about two or more axes;
    with one axis (a SCARA) the axis fixes two angles and a sweep of the third on the translation
    equations fixes the rest (up to the component along that axis)."""
    motions = _motions(T_bt, T_cp)
    alphas = np.array([cv2.Rodrigues(A[:3, :3])[0].ravel() for A, _ in motions])
    betas = np.array([cv2.Rodrigues(B[:3, :3])[0].ravel() for _, B in motions])
    sv = np.linalg.svd(alphas, compute_uv=False)
    single_axis = sv[1] <= 1e-3 * sv[0]
    if not single_axis:
        M = betas.T @ alphas
        R = np.real(np.linalg.inv(sqrtm(M.T @ M))) @ M.T
    else:
        big = np.linalg.norm(alphas, axis=1) > 0.02        # pairs with a rotation to speak of
        if not big.any():
            raise np.linalg.LinAlgError("no rotation between the captures")
        a = np.mean(alphas[big] / np.linalg.norm(alphas[big], axis=1, keepdims=True), axis=0)
        b = np.mean(betas[big] / np.linalg.norm(betas[big], axis=1, keepdims=True), axis=0)
        if min(np.linalg.norm(a), np.linalg.norm(b)) < 1e-6:
            raise np.linalg.LinAlgError("rotation axes cancel")
        a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
        R0 = _align(b, a)
        R = min((cv2.Rodrigues(a * np.radians(th))[0] @ R0 for th in range(0, 360, 3)),
                key=lambda Rk: _translation(Rk, motions)[1])
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt if np.linalg.det(U @ Vt) > 0 else U @ np.diag([1, 1, -1]) @ Vt
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, _translation(R, motions)[0]
    return X, single_axis


# ------------------------------------------------------------------ refinement
def _unpack(x, pin_z: bool):
    T_tc = _to_mat(x[:6])
    T_bp = _to_mat(np.concatenate([x[6:11], [0.0]]) if pin_z else x[6:12])
    return T_tc, T_bp


def _residuals(x, T_bt, T_cp, pin_z, w_rot):
    """Per image: observed cam->plate against the model's, translation in m and rotation in rad
    scaled by the camera-plate distance so both count as metres on the plate."""
    T_tc, T_bp = _unpack(x, pin_z)
    T_ct = _inv(T_tc)
    out = []
    for T, C in zip(T_bt, T_cp):
        P = T_ct @ _inv(T) @ T_bp
        out.append(P[:3, 3] - C[:3, 3])
        out.append(w_rot * cv2.Rodrigues(C[:3, :3].T @ P[:3, :3])[0].ravel())
    return np.concatenate(out)


def refine(T_bt: List[np.ndarray], T_cp: List[np.ndarray], X0: np.ndarray, pin_z: bool):
    """Least squares over T_tc and T_bp (plate z in base held at 0 when pin_z). Returns both."""
    bp0 = np.mean([_to_vec(T @ X0 @ C) for T, C in zip(T_bt, T_cp)], axis=0)
    x0 = np.concatenate([_to_vec(X0), bp0[:5] if pin_z else bp0])
    w_rot = float(np.mean([np.linalg.norm(C[:3, 3]) for C in T_cp]))
    res = least_squares(_residuals, x0, args=(T_bt, T_cp, pin_z, w_rot), method="trf", xtol=1e-14, ftol=1e-14)
    return _unpack(res.x, pin_z)


def _errors(T_tc, T_bp, T_bt, T_cp) -> np.ndarray:
    T_ct = _inv(T_tc)
    return np.array([np.linalg.norm((T_ct @ _inv(T) @ T_bp)[:3, 3] - C[:3, 3]) for T, C in zip(T_bt, T_cp)])


# ------------------------------------------------------------------ entry points
def load_folder(folder: str, spec: Optional[PlateSpec] = None):
    """(T_bt list, detections, files) from a vendor-style HandEye folder."""
    spec = spec or PlateSpec.default()
    T_bt, dets, files = [], [], []
    for f in sorted(glob.glob(os.path.join(folder, "handeye_*.png"))):
        n = os.path.basename(f)[len("handeye_"):-len(".png")]
        pose_file = os.path.join(folder, f"pose_{n}.txt")
        if not os.path.exists(pose_file):
            continue
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        try:
            dets.append(detect(img, spec))
        except PlateNotFound:
            continue
        T_bt.append(tool_matrix(read_pose_file(pose_file)))
        files.append(f)
    if len(dets) < 3:
        raise PlateNotFound(f"{folder}: plate found in {len(dets)} captures, need at least 3")
    return T_bt, dets, files


def _ransac_inliers(T_bt, T_cp, threshold_m: float, seed: int = 0) -> np.ndarray:
    """Which captures agree on one plate position: closed forms on image triples, the triple with
    the most captures whose plate-in-base lies within threshold_m of its median wins."""
    n = len(T_bt)
    rng = np.random.default_rng(seed)
    triples = [tuple(rng.choice(n, 3, replace=False)) for _ in range(min(60, n * (n - 1) * (n - 2) // 6))]
    best = np.ones(n, bool)
    for tri in set(triples):
        try:
            X, _ = closed_form([T_bt[i] for i in tri], [T_cp[i] for i in tri])
        except (np.linalg.LinAlgError, ValueError):
            continue
        p = np.array([(T @ X @ C)[:3, 3] for T, C in zip(T_bt, T_cp)])
        inl = np.linalg.norm(p - np.median(p[list(tri)], axis=0), axis=1) <= threshold_m
        if inl.sum() > best.sum() or best.all():
            best = inl
    return best


def solve(folder: str, camparam: halcon.CamParam, spec: Optional[PlateSpec] = None,
          max_error_m: float = 2e-3) -> CamInTool:
    """The vendor's hand-eye step. Captures that do not agree with the others on where the plate
    is (nudged between shots, a second session) are left out: a RANSAC vote first, then any
    capture further than max(max_error_m, 5 x median) from the refined solution."""
    T_bt, dets, files = load_folder(folder, spec)
    T_cp = [plate_pose(d, camparam) for d in dets]
    keep = _ransac_inliers(T_bt, T_cp, max_error_m)
    while True:
        bt, cp = [T for T, k in zip(T_bt, keep) if k], [C for C, k in zip(T_cp, keep) if k]
        X0, single_axis = closed_form(bt, cp)
        T_tc, T_bp = refine(bt, cp, X0, pin_z=single_axis)
        errs = _errors(T_tc, T_bp, T_bt, T_cp)
        bad = keep & (errs > max(max_error_m, 5 * np.median(errs[keep])))
        if not bad.any() or keep.sum() - bad.sum() < 3:
            break
        keep &= ~bad
    return CamInTool(halcon.Pose.from_matrix(T_tc), float(errs[keep].mean()), halcon.Pose.from_matrix(T_bp),
                     int(keep.sum()), ("tool z",) if single_axis else (),
                     tuple(os.path.basename(f) for f, k in zip(files, keep) if not k))
