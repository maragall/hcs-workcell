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
from dataclasses import dataclass, field, replace
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

from calibration import halcon, marker

IMX219_CELL_M = 1.12e-6
LAST: dict = {}               # the last flat solve's per-picture residuals (diagnosis only)
EDGE_MARGIN_PX = 100          # captures with a tag corner closer than this to the picture's edge are left out of the solve


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
        h, w = img.shape
        if min(c[:, 0].min(), c[:, 1].min(), w - 1 - c[:, 0].max(), h - 1 - c[:, 1].max()) < EDGE_MARGIN_PX:
            skipped.append(os.path.basename(f))        # a corner near the picture's edge: the lens bends it most and a cut corner is mis-read
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
        sy: float = IMX219_CELL_M, marker_z_m: Optional[float] = None) -> Result:
    """With `marker_z_m` (the height of the marker above the base plane, known where the plate sits on a fixed pad) the
    marker is a flat target at that height and the solve is staged (fit_flat); without it, the general joint solve."""
    caps = load_folder(folder, marker_id)
    if marker_z_m is not None:
        return fit_flat(caps, start, marker_size_m, marker_z_m, sy)
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


# ------------------------------------------------------------------ the staged solve for a flat marker at a known height
STAGES = (("focus", "cx", "cy"), ("focus", "cx", "cy", "k1"), ("focus", "cx", "cy", "k1", "k2"),
          ("focus", "cx", "cy", "k1", "k2", "k3", "p1", "p2"))
FLAT_SCALE = {"r1": 1, "r2": 1, "r3": 1, "tx": .1, "ty": .1, "tz": .1, "mx": .1, "my": .1, "myaw": 1, "focus": .1, "cx": 100,
              "cy": 100, "k1": 1e4, "k2": 1e9, "k3": 1e14, "p1": 10.0, "p2": 10.0}
FLAT_NAMES = ("r1", "r2", "r3", "tx", "ty", "tz", "mx", "my", "myaw", "focus", "cx", "cy", "k1", "k2", "k3", "p1", "p2")
EXTRINSICS = FLAT_NAMES[:9]
# the focal length trades against the camera's height when the marker height is pinned and the plan has few heights, so every
# parameter stays near where it started: a lens is a few percent off, a principal point a few hundred pixels
FLAT_BOUND = {"r1": 10, "r2": 10, "r3": 10, "tx": 0.4, "ty": 0.4, "tz": 0.4, "mx": 1.0, "my": 1.0, "myaw": 20.0, "focus": 0.12,
              "cx": 300.0, "cy": 300.0, "k1": 3.0, "k2": 30.0, "k3": 30.0, "p1": 30.0, "p2": 30.0}
START_YAWS_DEG, START_CAM_Z_M, CAM_AHEAD_M = range(0, 360, 30), (-0.3, -0.15, 0.0, 0.1), 0.115
REFINE_STARTS = 3                 # the sweep is a few iterations each; only the best starts are run to the end
MAX_TAG_FRACTION = 0.35           # a tag wider than this share of the picture is too close: out of focus, at the lens's edge
OUTLIER_FACTOR = 3.0
ROBUST_PX = 4.0                   # residuals beyond this many pixels count less (a picture taken too close, a mis-read corner)


def distinct(caps: Captures, tol_mm: float = 0.5, tol_deg: float = 0.5) -> List[int]:
    """Indices of the captures at distinct arm poses (the plan can visit a pose twice; a repeat adds no information)."""
    keep: List[int] = []
    poses: List[np.ndarray] = []
    for i, T in enumerate(caps.T_bt):
        yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
        key = np.array([*(T[:3, 3] * 1e3), yaw])
        if any(np.abs(key[:3] - k[:3]).max() < tol_mm and abs((key[3] - k[3] + 180) % 360 - 180) < tol_deg for k in poses):
            continue
        poses.append(key)
        keep.append(i)
    return keep


def _flat_model(z, start: halcon.CamParam, marker_z: float):
    cam = replace(start, focus=start.focus * (1 + z[9]), cx=start.cx + z[10], cy=start.cy + z[11],
                  poly=(start.poly[0] + z[12] * 1e4, start.poly[1] + z[13] * 1e9, start.poly[2] + z[14] * 1e14,
                        start.poly[3] + z[15] * 10.0, start.poly[4] + z[16] * 10.0))
    T_bm = np.eye(4)
    T_bm[:3, :3] = halcon.rot_z(z[8])
    T_bm[:3, 3] = [z[6], z[7], marker_z]
    return cam, _to_mat(z[:6]), T_bm


def _flat_residuals(z, T_bt, corners, obj, start, marker_z):
    cam, T_tc, T_bm = _flat_model(z, start, marker_z)
    T_ct = _inv(T_tc)
    out = []
    for T, c in zip(T_bt, corners):
        P = T_ct @ _inv(T) @ T_bm
        out.append(cam.project(obj @ P[:3, :3].T + P[:3, 3]) - c)
    return np.concatenate(out).ravel()


def _fit_names(z0, names, args, max_nfev: int = 600, loss: str = "soft_l1"):
    idx = [FLAT_NAMES.index(n) for n in names]

    def f(v):
        z = z0.copy()
        z[idx] = v
        return _flat_residuals(z, *args)
    lo = np.array([-FLAT_BOUND[n] for n in names])
    hi = np.array([FLAT_BOUND[n] for n in names])
    r = least_squares(f, np.clip(z0[idx], lo + 1e-9, hi - 1e-9), x_scale=np.array([FLAT_SCALE[n] for n in names]),
                      bounds=(lo, hi), loss=loss, f_scale=ROBUST_PX, xtol=1e-13, ftol=1e-13, max_nfev=max_nfev)
    z = z0.copy()
    z[idx] = r.x
    return z, float(np.sqrt(np.mean(np.sum(r.fun.reshape(-1, 2) ** 2, axis=1)))), r.fun


def fit_flat(caps: Captures, start: halcon.CamParam, marker_size_m: float, marker_z_m: float,
             sy: float = IMX219_CELL_M) -> Result:
    """The marker lies flat at `marker_z_m` above the base plane (x, y and yaw unknown). Stage A: camera-in-tool and marker
    with the start lens, from a sweep of starting camera rotations (the closed-form hand-eye start is unreliable on a few
    poses of a SCARA); stage B: the focal length and principal point; stage C: K1, then K2. Pictures that fit three times
    worse than the median are dropped once. The camera's offset along tool z is real here because the marker height is
    pinned."""
    use = [i for i in distinct(caps)
           if np.linalg.norm(caps.corners[i][1] - caps.corners[i][0]) <= MAX_TAG_FRACTION * caps.width]
    if len(use) < 6:
        raise marker.MarkerNotFound(f"only {len(use)} usable pictures at distinct poses, need at least 6")
    T_bt, corners = [caps.T_bt[i] for i in use], [caps.corners[i] for i in use]
    obj = marker.object_points(marker_size_m)
    T_cm = [marker_pose(c, start, marker_size_m) for c in corners]
    starts = []
    for th in START_YAWS_DEG:
        for flip in (0, 1):
            R = halcon.rot_z(np.radians(th)) @ (halcon.rot_x(np.pi) if flip else np.eye(3))
            for cz in START_CAM_Z_M:
                X = np.eye(4)
                X[:3, :3], X[:3, 3] = R, [CAM_AHEAD_M, 0.0, cz]
                bm = np.mean([_to_vec(T @ X @ C) for T, C in zip(T_bt, T_cm)], axis=0)
                z0 = np.zeros(len(FLAT_NAMES))
                z0[:6], z0[6:9] = _to_vec(X), [bm[3], bm[4], bm[2]]
                starts.append(z0)
    args = (T_bt, corners, obj, start, marker_z_m)

    def robust_score(res):
        return float(np.median(np.sqrt(np.sum(res.reshape(len(corners), 4, 2) ** 2, axis=2).mean(axis=1))))   # one bad picture must not pick the basin
    rough = []
    for z0 in starts:
        try:
            z, _, res = _fit_names(z0, EXTRINSICS, args, max_nfev=40)
        except (np.linalg.LinAlgError, ValueError):
            continue
        rough.append((robust_score(res), len(rough), z))
    best = None
    for _, _, z in sorted(rough, key=lambda r: r[:2])[:REFINE_STARTS]:
        z, px, res = _fit_names(z, EXTRINSICS, args)
        score = robust_score(res)
        if best is None or score < best[3]:
            best = (z, px, res, score)
    if best is None:
        raise np.linalg.LinAlgError("no starting camera rotation gave a fit")
    z, px, res, _ = best
    per = np.sqrt(np.sum(res.reshape(len(corners), 4, 2) ** 2, axis=2).mean(axis=1))
    good = per <= max(OUTLIER_FACTOR * float(np.median(per)), 1.0)
    if good.sum() >= 6 and not good.all():
        T_bt, corners = [t for t, g in zip(T_bt, good) if g], [c for c, g in zip(corners, good) if g]
        use = [u for u, g in zip(use, good) if g]
    args = (T_bt, corners, obj, start, marker_z_m)
    z, px, res = _fit_names(z, EXTRINSICS, args, loss="linear")      # the basin is found and the outliers dropped: plain least squares
    for names in STAGES:
        z, px, res = _fit_names(z, EXTRINSICS + names, args, loss="linear")
    cam, T_tc, T_bm = _flat_model(z, start, marker_z_m)
    LAST.update(files=[os.path.basename(caps.files[i]) for i in use], z=z, res=res.reshape(len(use), 4, 2), names=FLAT_NAMES)   # for diagnosis
    errs = [np.linalg.norm((_inv(T_tc) @ _inv(T) @ T_bm)[:3, 3] - marker_pose(c, cam, marker_size_m)[:3, 3])
            for T, c in zip(T_bt, corners)]
    kept = {caps.files[i] for i in use}
    skipped = tuple(caps.skipped) + tuple(os.path.basename(f) for f in caps.files if f not in kept)
    return Result(cam.with_error(px), halcon.Pose.from_matrix(T_tc), halcon.Pose.from_matrix(T_bm), px, float(np.mean(errs)),
                  len(use), ("marker height",), skipped)
