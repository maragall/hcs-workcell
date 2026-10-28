"""
Register the CAD frame to the robot frame with no taught pose: from the arm's own observations of the module.

What the arm can observe by itself, with no camera calibration and no lens model:

  1. j1. The arm's kinematics put the shoulder axis at (x0, y0) in the robot frame (routes_opt/fk.py); the CAD puts it at
     (0, -102.1) in the platform's frame. One point in common.
  2. The module axis. The aruco servo (bringup/servo.py) finds the wrist pose V0 at which the plate tag looks as it did at the
     reference. The module turns the plate by a known angle about its axis; the servo finds the wrist pose V1 at which the tag
     looks the same again. The wrist must have made the plate's own rigid motion, a turn about the module axis, so the fixed
     point of the motion V0 -> V1 IS the module axis in the robot frame, whatever the camera's place on the hand, the lens and
     the tag's place on the plate. A second point in common, and the angle the module turned comes out of the same two poses
     (a check on the module's direction and on the arm's yaw scale).
  3. The height, from the tag's size in the picture at two wrist heights: size is 1 / depth, so two sizes and two heights give
     the height of the tag plane without the focal length (it needs how far the camera sits above the fingers' mid height,
     hardware.json cad.camera.above_tcp_mm). Without it, k stays at the assumed value and the result says so.

Two points in common fix the planar rigid transform (the CAD is metal, so no scale). The arm's frame is its own: whatever
rotation the platform stands at, whatever its base offset, comes out.

    robot (x, y) = R(theta) . m(c) + t,   m(c) = (z_c, x_c),   robot z = y_c + k
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from routes_opt import fk

K_ASSUMED = -57.4                 # the wrist reading is at the fingers' mid height; checked against the taught seats to ~1 mm
Pose = Tuple[float, float, float]  # wrist x, y mm, yaw deg


def _rot(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def m(x_c: float, z_c: float) -> np.ndarray:
    """The CAD's plane (x across the platform, z along it) as the robot's x, y before rotation and shift."""
    return np.array([z_c, x_c], float)


class RegistrationError(Exception):
    pass


@dataclass(frozen=True)
class Registration:
    theta_deg: float
    tx: float
    ty: float
    k: float
    k_source: str
    axis_robot: Tuple[float, float]
    checks: Dict[str, float] = field(default_factory=dict)
    warnings: Tuple[str, ...] = ()

    def xy(self, x_c: float, z_c: float) -> Tuple[float, float]:
        p = _rot(self.theta_deg) @ m(x_c, z_c) + np.array([self.tx, self.ty])
        return float(p[0]), float(p[1])

    def z(self, y_c: float) -> float:
        return y_c + self.k


# ------------------------------------------------------------------------------------------------ the observations
def rigid_centre(v0: Pose, v1: Pose, min_turn_deg: float = 5.0) -> Tuple[Tuple[float, float], float]:
    """The fixed point and angle of the planar motion that takes wrist pose v0 to v1 (x, y, yaw). The camera is rigid on the
    wrist, so this is the plate's own motion seen from the arm."""
    turn = wrap(v1[2] - v0[2])
    if abs(turn) < min_turn_deg:
        raise RegistrationError(f"the wrist turned {turn:.1f} deg between the two views: the module must turn the plate by "
                                f"90 deg or more for its axis to show")
    R = _rot(turn)
    a = np.linalg.solve(np.eye(2) - R, np.array(v1[:2], float) - R @ np.array(v0[:2], float))
    return (float(a[0]), float(a[1])), turn


def module_axis(v0: Pose, v1: Pose, commanded_deg: Optional[float] = None) -> Tuple[Tuple[float, float], float, List[str]]:
    axis, turn = rigid_centre(v0, v1)
    warn = []
    if commanded_deg is not None and abs(abs(turn) - abs(commanded_deg)) > 1.0:
        warn.append(f"the module was turned {commanded_deg:.1f} deg but the wrist turned {turn:.1f} deg between the views: "
                    f"the arm's yaw scale, or the servo, is off by {abs(abs(turn) - abs(commanded_deg)):.1f} deg")
    return axis, turn, warn


def height_from_scale(views: Sequence[Tuple[float, float]], camera_above_tcp: float) -> Tuple[float, float]:
    """Height of the tag plane in the wrist's z coordinate, and the relative fit residual, from (wrist z, tag side in px) at
    two or more heights over the tag: side = C / (z_wrist + above - z_tag), so 1/side is linear in z_wrist and its zero is
    z_tag - above. No focal length."""
    if len(views) < 2 or max(v[0] for v in views) - min(v[0] for v in views) < 20.0:
        raise RegistrationError("the tag's size is needed at two wrist heights at least 20 mm apart")
    z = np.array([v[0] for v in views], float)
    inv = 1.0 / np.array([v[1] for v in views], float)
    a, b = np.polyfit(z, inv, 1)
    zero = -b / a                                  # wrist z at which the camera would be in the tag plane
    resid = float(np.max(np.abs(np.polyval([a, b], z) - inv) / inv))
    return float(zero + camera_above_tcp), resid


# ------------------------------------------------------------------------------------------------ the registration
def _cad_points(cad: dict) -> Tuple[np.ndarray, np.ndarray, float]:
    j1 = cad["checks"]["j1_cad_xz"]                                     # (x_c, z_c)
    axis = cad["rotation_module"].get("axis_c")
    if axis is None:                                                    # older hardware.json: the nominal robot axes
        off = cad["mapping"].get("z_to_x_mm", 228.1)
        axis = {"x": cad["rotation_module"]["axis_r"][1], "z": cad["rotation_module"]["axis_r"][0] - off}
    tag_plane_y_c = cad["rotation_module"].get("pad_top_y_c", cad["rotation_module"]["top_y_c"]) + cad.get("plate", {}).get("height_mm", 14.5)
    return m(j1[0], j1[1]), m(axis["x"], axis["z"]), float(tag_plane_y_c)


def register(cad: dict, view0: Pose, view1: Pose, *, commanded_deg: Optional[float] = None,
             scale_views: Optional[Sequence[Tuple[float, float]]] = None, camera_above_tcp: Optional[float] = None,
             j1_robot: Optional[Tuple[float, float]] = None, k_assumed: float = K_ASSUMED) -> Registration:
    """CAD frame -> robot frame from the wrist poses of the module tag before and after the module turned (view0, view1), the
    arm's j1, and optionally the tag's size at two heights. Raises RegistrationError when the views cannot give an axis."""
    j1_r = np.array(j1_robot if j1_robot is not None else (fk.MODEL["x0"], fk.MODEL["y0"]), float)
    j1_c, axis_c, tag_y_c = _cad_points(cad)
    axis_r, turn, warnings = module_axis(view0, view1, commanded_deg)
    vec_c, vec_r = axis_c - j1_c, np.array(axis_r) - j1_r
    theta = wrap(math.degrees(math.atan2(vec_r[1], vec_r[0]) - math.atan2(vec_c[1], vec_c[0])))
    t = j1_r - _rot(theta) @ j1_c
    dist_c, dist_r = float(np.linalg.norm(vec_c)), float(np.linalg.norm(vec_r))
    checks = {"j1_to_module_axis_cad_mm": round(dist_c, 2), "j1_to_module_axis_robot_mm": round(dist_r, 2),
              "distance_difference_mm": round(dist_r - dist_c, 2), "module_turn_deg": round(turn, 2)}
    if abs(dist_r - dist_c) > 1.5:
        warnings.append(f"the module axis is {dist_r:.1f} mm from j1 in the robot frame and {dist_c:.1f} mm in the CAD: j1's place "
                        "in the arm model, a link length or the servo is off by that much")
    if scale_views is not None:
        above = camera_above_tcp if camera_above_tcp is not None else (cad.get("camera") or {}).get("above_tcp_mm")
        if above is None:
            raise RegistrationError("camera_above_tcp is needed for the height (hardware.json cad.camera.above_tcp_mm)")
        z_tag, resid = height_from_scale(scale_views, above)
        k, source = z_tag - tag_y_c, "the tag's size at two heights"
        checks["height_fit_residual"] = round(resid, 4)
        if resid > 0.03:
            warnings.append(f"the tag's size is not 1/depth within 3 % over the heights ({resid:.1%}): check the picture is in focus")
    else:
        k, source = k_assumed, "assumed (the wrist reading is at the fingers' mid height); give the tag's size at two heights to measure it"
    return Registration(theta, float(t[0]), float(t[1]), float(k), source, (float(axis_r[0]), float(axis_r[1])), checks, tuple(warnings))


# ------------------------------------------------------------------------------------------------ the arm's own check
@dataclass(frozen=True)
class SelfCheck:
    centre: Tuple[float, float]
    radius_mm: float
    residual_mm: float                 # worst distance of a pair's centre from the mean of the pairs' centres
    turn_errors_deg: Tuple[float, ...]
    ok: bool
    notes: Tuple[str, ...]


UNDETECTABLE = (
    "a joint-2 zero offset: it turns the whole frame about j1, which is the same as theta; harmless while every pose is in the "
    "robot frame, and it cannot be told apart",
    "a joint-4 (yaw) zero offset against the claw's true direction: the wrist yaw reads in the arm's own frame, so the claw's true "
    "yaw needs the camera's rotation on the hand (the tag upright in the picture), which these views do not give",
    "the camera's height above the fingers (only its sum with the tag's height is seen)",
    "an error the same at every view: the views sit around the module axis, 221 mm from j1, so they test the arm's xy over about "
    "100 mm of workspace and one reach; a link length error shows only as it changes between the views",
)


def arm_selfcheck(views: Dict[float, Pose], commanded: Optional[Sequence[float]] = None, tol_mm: float = 1.0) -> SelfCheck:
    """The plate turns about the module axis, so the wrist motion between any two views that re-find the tag is a turn about that
    axis. Every pair of views (turned by 30 deg or more) gives the axis; if the arm's kinematics are right to a millimetre the
    pairs agree, and the turn they measure is the turn the module made. A wrong link length or a joint-3 zero offset bends the
    reach differently at each view, so the pairs disagree. Module angles in degrees -> wrist poses; `commanded` is the angles
    the module was sent to, in the same order (else the keys). Three views at least (0, 90, 180)."""
    angles = list(views)
    if len(angles) < 3:
        raise RegistrationError("three module angles at least (0, 90, 180)")
    cmd = dict(zip(angles, commanded if commanded is not None else angles))
    centres, radii, notes = [], [], []
    turn_errors = []
    for i, a in enumerate(angles):
        for b in angles[i + 1:]:
            want = wrap(cmd[b] - cmd[a])
            if abs(want) < 30.0:
                continue
            try:
                centre, turn = rigid_centre(views[a], views[b], min_turn_deg=30.0)
            except RegistrationError:
                continue
            centres.append(centre)
            turn_errors.append(round(turn - want, 3))
            radii += [math.dist(views[a][:2], centre), math.dist(views[b][:2], centre)]
    if len(centres) < 2:
        raise RegistrationError("not enough views turned by 30 deg or more from each other")
    C = np.array(centres)
    mean = C.mean(0)
    resid = float(np.max(np.linalg.norm(C - mean, axis=1)))
    if resid > tol_mm:
        notes.append(f"the pairs of views disagree on the module axis by {resid:.2f} mm: a link length, a joint-3 zero offset or the servo "
                     "is off; the arm's xy is not good to a millimetre over this part of the workspace")
    if any(abs(e) > 0.5 for e in turn_errors):
        notes.append(f"the wrist turned {turn_errors} deg more or less than the module did between views: the yaw scale is off")
    return SelfCheck((float(mean[0]), float(mean[1])), float(np.mean(radii)), resid, tuple(turn_errors), not notes, tuple(notes))
