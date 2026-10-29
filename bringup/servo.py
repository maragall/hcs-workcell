"""
Where is the plate, without a camera calibration: a visual servo on the plate's tag.

The camera is rigid on the wrist, so when the tag looks the same in two pictures the wrist stands in the same place
relative to the tag. `Servo.align` moves the wrist (x, y, yaw at a fixed height) until the tag's four corners in the
down camera picture equal a reference set; the wrist pose reached IS the tag's place in the robot frame up to a constant
(the camera's offset on the hand) that is the same every time, so it cancels in every difference:

    reference   at the module: the tag centred, view pose V_module and its corners
    stage       the same servo to the same corners gives V_stage
    grip pose   G_stage = V_stage + R(yaw V_stage - yaw V_module) . (G_module - V_module),
                yaw  G_stage = yaw G_module + (yaw V_stage - yaw V_module)       (`transfer_grip`)
    offset      of the plate on the module now against an earlier run, at the taught grip pose: `plate_offset`

The servo needs no lens, no hand-eye and no marker size. It measures the picture's response to small moves
(finite-difference Jacobian, +-6 mm and +-3 deg), then damped Gauss-Newton on the corner error with Broyden updates,
re-measuring when the error grows; it stops under `tol_px` or when a step is below what the arm repeats (0.08 mm). It refuses when the tag leaves the picture or a move is refused.

    read_corners() -> (4, 2) pixel corners of the tag, or None when there is no whole tag in the picture
    move_wrist(dx, dy, dyaw)  a small checked move, dx, dy in mm in the robot frame, dyaw in degrees; may return the new
                              wrist pose (x, y, yaw); `get_pose()` is used when given, else the servo adds up its moves
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

SETTLED_MM, SETTLED_DEG = 0.08, 0.06          # a step this small is below the arm's repeatability: converged
BROYDEN_MIN_MM, BROYDEN_MIN_DEG = 3.0, 1.5      # steps smaller than this are noise to the Jacobian update
Pose = Tuple[float, float, float]            # wrist x, y in mm (robot frame), yaw in degrees


class ServoError(Exception):
    """The servo refused: the tag left the picture, a move was refused, or it did not converge."""


@dataclass(frozen=True)
class ServoResult:
    pose: Pose                 # the wrist pose reached
    residual_px: float         # rms distance of the four corners from the reference, in pixels
    moves: int                 # moves made, Jacobian probes included
    iterations: int            # Gauss-Newton steps
    mm_per_px: float           # what one pixel is worth in mm at this height (from the measured Jacobian)


def _rot(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def _rms(e: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum(np.asarray(e).reshape(-1, 2) ** 2, axis=1))))


STALL_OK_PX = 25.0               # the error floor accepted when three steps in a row stop improving it


class Servo:
    def __init__(self, read_corners: Callable[[], Optional[np.ndarray]],
                 move_wrist: Callable[[float, float, float], Optional[Sequence[float]]],
                 get_pose: Optional[Callable[[], Sequence[float]]] = None,
                 tol_px: float = 1.5, max_iter: int = 8, probe_mm: float = 6.0, probe_deg: float = 3.0,
                 max_step_mm: float = 15.0, max_step_deg: float = 8.0, damping: float = 1e-3):
        self.read_corners, self.move_wrist, self.get_pose = read_corners, move_wrist, get_pose
        self.tol_px, self.max_iter = tol_px, max_iter
        self.probe = np.array([probe_mm, probe_mm, probe_deg])
        self.max_step = np.array([max_step_mm, max_step_mm, max_step_deg])
        self.damping = damping
        self.moves = 0
        self._pose = np.zeros(3)

    # ---------------------------------------------------------------- the two things the arm and camera do
    def _move(self, delta: np.ndarray) -> None:
        try:
            got = self.move_wrist(float(delta[0]), float(delta[1]), float(delta[2]))
        except ServoError:
            raise
        except Exception as e:                       # the checked move refused (the planner, the safety layer)
            raise ServoError(f"a move was refused: {e}") from e
        self.moves += 1
        if self.get_pose is not None:
            self._pose = np.asarray(self.get_pose(), float)
        elif got is not None:
            self._pose = np.asarray(got, float)
        else:
            self._pose = self._pose + delta

    def _look(self, what: str) -> np.ndarray:
        c = self.read_corners()
        if c is None:
            raise ServoError(f"the tag is not in the picture {what}")
        return np.asarray(c, float).reshape(4, 2)

    def observe(self, start_pose: Optional[Sequence[float]] = None) -> Tuple[np.ndarray, Pose]:
        """The tag's corners and the wrist pose as they are now (use it to take the reference)."""
        if start_pose is not None:
            self._pose = np.asarray(start_pose, float)
        elif self.get_pose is not None:
            self._pose = np.asarray(self.get_pose(), float)
        return self._look("here"), tuple(float(v) for v in self._pose)

    # ---------------------------------------------------------------- the Jacobian
    def _measure_jacobian(self, ref: np.ndarray, base: np.ndarray) -> np.ndarray:
        """(8, 3) pixels per mm, mm, degree from forward differences, returning to where it started. A probe that would
        take the tag out of the picture is made the other way."""
        J = np.zeros((8, 3))
        for k in range(3):
            for sign in (1.0, -1.0):
                step = np.zeros(3)
                step[k] = sign * self.probe[k]
                self._move(step)
                moved = self.read_corners()
                self._move(-step)
                if moved is not None:
                    J[:, k] = (np.asarray(moved, float).ravel() - base.ravel()) / step[k]
                    break
            else:
                raise ServoError(f"the tag leaves the picture for a probe either way in axis {k}")
        return J

    @staticmethod
    def mm_per_px(J: np.ndarray) -> float:
        s = np.linalg.svd(J[:, :2], compute_uv=False)
        return float(2.0 / s[-1]) if s[-1] > 1e-9 else float("inf")        # four corners move together: sqrt(4) / s

    # ---------------------------------------------------------------- estimate without walking
    def estimate(self, reference: np.ndarray, max_move_mm: float = 1.5, max_yaw_deg: float = 0.4,
                 linear_mm: float = 4.0, linear_deg: float = 4.0) -> "Estimate":
        """Where the wrist should be, from here, without going there: for places where the arm may only nudge (the microscope
        zone: under 2 mm lateral, 0.5 deg yaw). Probe moves of 0.8 x the limits, both ways in each axis, give the image
        Jacobian; a linear solve gives (dx, dy, dyaw) that would bring the corners to `reference`. Nothing moves beyond the
        probe size and the wrist ends where it started. The estimate is linear, and its error is about 7 % of the offset (the
        camera sits 115 mm from the wrist axis, so a turn is an arc): within 0.3 mm for wrist offsets up to `linear_mm` /
        `linear_deg` = 4 (simulator), 0.5 mm at 7 mm; beyond that `warning` is set and the offset is only a direction, so
        move there (inside the zone: along the taught route) and estimate again."""
        ref = np.asarray(reference, float).reshape(4, 2)
        probe = np.array([0.8 * max_move_mm, 0.8 * max_move_mm, 0.8 * max_yaw_deg])
        self.moves = 0
        base = self._look("at the start")
        J = np.zeros((8, 3))
        for k in range(3):
            cols = []
            for sign in (1.0, -1.0):
                step = np.zeros(3)
                step[k] = sign * probe[k]
                self._move(step)
                got = self.read_corners()
                self._move(-step)
                if got is not None:
                    cols.append((np.asarray(got, float).ravel() - base.ravel()) / step[k])
            if not cols:
                raise ServoError(f"the tag leaves the picture for a probe either way in axis {k}")
            J[:, k] = np.mean(cols, axis=0)
        e = (base - ref).ravel()
        delta = -np.linalg.lstsq(J, e, rcond=None)[0]
        residual = _rms(e + J @ delta)
        warn = None
        if np.hypot(delta[0], delta[1]) > linear_mm or abs(delta[2]) > linear_deg:
            warn = (f"the offset {np.hypot(delta[0], delta[1]):.1f} mm, {delta[2]:.1f} deg is beyond the {linear_mm:g} mm, "
                    f"{linear_deg:g} deg the linear estimate holds to 0.3 mm for: take it as a direction and look again")
        elif residual > 3.0:
            warn = f"the linear fit leaves {residual:.1f} px: the picture does not move like a rigid tag (is it the right tag?)"
        return Estimate((float(delta[0]), float(delta[1]), float(delta[2])), residual, self.mm_per_px(J), self.moves, warn)

    # ---------------------------------------------------------------- the servo
    def align(self, reference: np.ndarray, start_pose: Optional[Sequence[float]] = None) -> ServoResult:
        """Move the wrist until the four corners equal `reference` (4, 2). Raises ServoError when it cannot."""
        ref = np.asarray(reference, float).reshape(4, 2)
        if start_pose is not None:
            self._pose = np.asarray(start_pose, float)
        elif self.get_pose is not None:
            self._pose = np.asarray(self.get_pose(), float)
        self.moves = 0
        cur = self._look("at the start")
        e = (cur - ref).ravel()
        if _rms(e) <= self.tol_px:
            return ServoResult(tuple(self._pose), _rms(e), 0, 0, float("nan"))
        J = self._measure_jacobian(ref, cur)
        mm_px = self.mm_per_px(J)
        scale = np.array([1.0, 1.0, 1.0])
        iters = 0
        seen = [round(_rms(e), 1)]
        for iters in range(1, self.max_iter + 1):
            A = J.T @ J + self.damping * np.trace(J.T @ J) / 3 * np.eye(3)
            delta = -np.linalg.solve(A, J.T @ e)
            over = np.max(np.abs(delta) / self.max_step)
            if over > 1.0:
                delta = delta / over
            before = _rms(e)
            self._move(delta)
            cur = self._look(f"after step {iters}")
            e_new = (cur - ref).ravel()
            if _rms(e_new) > max(before * 1.5, before + 2.0 * self.tol_px):     # clearly worse (not picture noise): go half way back and re-measure
                self._move(-delta / 2)
                cur = self._look(f"after backing off at step {iters}")
                e_new = (cur - ref).ravel()
                J = self._measure_jacobian(ref, cur)
                mm_px = self.mm_per_px(J)
            elif np.hypot(delta[0], delta[1]) > BROYDEN_MIN_MM or abs(delta[2]) > BROYDEN_MIN_DEG:
                dp = delta * scale                   # Broyden: a real step taught the Jacobian something (a tiny one is noise)
                J = J + np.outer(e_new - e - J @ dp, dp) / float(dp @ dp)
            e = e_new
            seen.append(round(_rms(e), 1))
            settled = np.hypot(delta[0], delta[1]) < SETTLED_MM and abs(delta[2]) < SETTLED_DEG and _rms(e) <= 3 * self.tol_px
            stalled = len(seen) > 3 and seen[-3] - seen[-1] < 0.3 and _rms(e) <= STALL_OK_PX       # a scale mismatch (the plate's height) leaves a floor that x, y and yaw cannot remove: they are right
            if _rms(e) <= self.tol_px or settled or stalled:    # under the tolerance, or the step is under what the arm can do
                return ServoResult(tuple(float(v) for v in self._pose), _rms(e), self.moves, iters, mm_px)
        raise ServoError(f"did not converge in {self.max_iter} steps: the corners are still {_rms(e):.1f} px off (px per step {seen})")


@dataclass(frozen=True)
class Estimate:
    offset: Tuple[float, float, float]   # (dx, dy mm, dyaw deg) the wrist would move to put the corners on the reference
    residual_px: float                   # what the linear fit leaves
    mm_per_px: float
    moves: int                           # probe moves (all of them within the limits, the wrist ends where it began)
    warning: Optional[str]               # set when the offset is large enough for the linear estimate to be unreliable


# --------------------------------------------------------------------------------------- what the poses give
def transfer_grip(view_ref: Pose, view_new: Pose, grip_ref: Pose) -> Pose:
    """The grip pose of the plate where it now is: the grip pose at the reference carried with the view poses.
    All poses are wrist (x, y mm, yaw deg) in the robot frame; view_ref and view_new are the servo's results at the
    reference place and the new one, grip_ref the pose that grips the plate at the reference place."""
    dyaw = view_new[2] - view_ref[2]
    v = _rot(dyaw) @ (np.asarray(grip_ref[:2], float) - np.asarray(view_ref[:2], float))
    return (float(view_new[0] + v[0]), float(view_new[1] + v[1]), float(grip_ref[2] + dyaw))


def plate_offset(view_before: Pose, view_now: Pose, grip: Pose) -> Pose:
    """How far the plate has moved, seen at its grip point: (dx, dy mm, dyaw deg) to add to the taught grip pose so it
    follows the plate. The plate is found with the servo at the earlier run (`view_before`) and now (`view_now`)."""
    new = transfer_grip(view_before, view_now, grip)
    return (new[0] - grip[0], new[1] - grip[1], new[2] - grip[2])


# --------------------------------------------------------------------------------------- the camera side
def corner_reader(get_frame: Callable[[], Optional[bytes]], edge_px: int = 20, keep: Optional[Callable[[bytes, Optional[np.ndarray]], None]] = None) -> Callable[[], Optional[np.ndarray]]:
    """read_corners for the down camera: a full-resolution frame, the plate tag found with calibration.marker.detect,
    None when there is no whole tag (not found, or a corner within edge_px of the picture's border)."""
    import cv2
    from calibration import marker

    def read() -> Optional[np.ndarray]:
        frame = get_frame()
        if frame is None:
            return None
        gray = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return None
        try:
            c = marker.detect(gray, None)
        except marker.MarkerNotFound:
            if keep:
                keep(frame, None)
            return None
        if keep:
            keep(frame, c)
        h, w = gray.shape
        if (c[:, 0] < edge_px).any() or (c[:, 1] < edge_px).any() or (c[:, 0] > w - 1 - edge_px).any() or (c[:, 1] > h - 1 - edge_px).any():
            return None
        return c

    return read
