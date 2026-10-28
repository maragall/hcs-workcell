"""The plate-tag servo on a simulated camera that the servo knows nothing about: no lens, no hand-eye, no marker size."""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from bringup.servo import Servo, ServoError, plate_offset, transfer_grip  # noqa: E402


def rot(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


class World:
    """A 40 mm tag lying flat at `plate` (x, y, yaw), a camera pointing down, 115 mm ahead of the wrist and 34 mm to the
    side, its image axes turned `cam_rot` degrees from the wrist's, 110 mm above the tag, focal 2632 px on a 3280 x 2464
    picture. Noise: corners `noise_px`, the arm's position `noise_mm` per move."""
    W, H_PX, F, HEIGHT = 3280, 2464, 2632.0, 110.0
    AHEAD, SIDE, CAM_ROT = 115.0, 34.0, -90.0

    def __init__(self, plate, wrist, noise_px=0.3, noise_mm=0.05, seed=1):
        self.plate = np.asarray(plate, float)
        self.wrist = np.asarray(wrist, float)
        self.noise_px, self.noise_mm = noise_px, noise_mm
        self.rng = np.random.default_rng(seed)

    def corners_at(self, wrist, noise=True):
        cam = wrist[:2] + rot(wrist[2]) @ np.array([self.AHEAD, self.SIDE])
        out = []
        for u, v in ((-20, 20), (20, 20), (20, -20), (-20, -20)):
            p = self.plate[:2] + rot(self.plate[2]) @ np.array([u, v], float)
            rel = rot(-(wrist[2] + self.CAM_ROT)) @ (p - cam)
            out.append([self.W / 2 + self.F * rel[0] / self.HEIGHT, self.H_PX / 2 + self.F * rel[1] / self.HEIGHT])
        out = np.array(out)
        return out + (self.rng.normal(0, self.noise_px, out.shape) if noise else 0.0)

    def read(self):
        c = self.corners_at(self.wrist)
        inside = (c[:, 0] > 20).all() and (c[:, 0] < self.W - 20).all() and (c[:, 1] > 20).all() and (c[:, 1] < self.H_PX - 20).all()
        return c if inside else None

    def move(self, dx, dy, dyaw):
        self.wrist = self.wrist + np.array([dx, dy, dyaw]) + np.concatenate([self.rng.normal(0, self.noise_mm, 2), [0.0]])
        return tuple(self.wrist)

    def wrist_for_tag_centred(self, yaw):
        """The wrist pose with the camera over the tag's centre at this yaw (the 'tag centred' view)."""
        return np.array([*(self.plate[:2] - rot(yaw) @ np.array([self.AHEAD, self.SIDE])), yaw])


def servo_for(world):
    return Servo(world.read, world.move)


def test_it_converges_to_the_arms_noise_floor_from_starts_up_to_25_mm_and_15_degrees_off():
    rng = np.random.default_rng(7)
    moves, worst, mm_px = [], 0.0, []
    for trial in range(20):
        w = World((313.0, 118.0, 0.0), (0, 0, 0), seed=trial)
        view = w.wrist_for_tag_centred(-91.0)
        w.wrist = view.copy()
        ref = w.corners_at(view, noise=False)
        r = rng.uniform(5, 25)
        a = rng.uniform(0, 2 * math.pi)
        w.wrist = view + np.array([r * math.cos(a), r * math.sin(a), rng.uniform(-15, 15)])
        got = servo_for(w).align(ref, start_pose=w.wrist)
        worst = max(worst, got.residual_px)
        moves.append(got.moves)
        mm_px.append(got.mm_per_px)
        assert np.abs(w.wrist[:2] - view[:2]).max() < 0.15 and abs(w.wrist[2] - view[2]) < 0.1     # where the tag looks like the reference
    assert worst < 3.0                                           # 0.05 mm of arm noise is 1.2 px at 0.042 mm per px
    assert max(moves) <= 20 and abs(np.mean(mm_px) - 110.0 / 2632.0) < 0.01


def test_the_seat_transfers_from_the_module_to_a_stage_plate_20_mm_and_20_degrees_away():
    """Module: the servo finds the view V_module; the grip pose is known there. At the stage the plate is 20 mm and 20 deg
    away: the servo to the same corners finds V_stage and the transfer gives the stage grip pose."""
    module_plate = np.array([313.0, 118.0, -91.0])
    w = World(module_plate, (0, 0, 0), seed=3)
    v_ref = w.wrist_for_tag_centred(-91.0)
    ref = w.corners_at(v_ref, noise=False)
    grip_module = np.array([*(module_plate[:2] + rot(module_plate[2]) @ np.array([-60.0, 10.0])), module_plate[2] + 90.0])
    w.wrist = v_ref + np.array([9.0, -6.0, 4.0])
    v_module = servo_for(w).align(ref, start_pose=w.wrist).pose

    stage_plate = np.array([123.0 + 20.0 * math.cos(math.radians(30)), 400.0 + 20.0 * math.sin(math.radians(30)), -91.0 + 20.0])
    w.plate = stage_plate
    w.wrist = w.wrist_for_tag_centred(stage_plate[2]) + np.array([-11.0, 8.0, -6.0])        # the arm arrives where the plate was expected
    v_stage = servo_for(w).align(ref, start_pose=w.wrist).pose

    got = transfer_grip(v_module, v_stage, tuple(grip_module))
    # truth: the grip pose keeps its place relative to the plate
    rel = rot(-module_plate[2]) @ (grip_module[:2] - module_plate[:2])
    truth_xy = stage_plate[:2] + rot(stage_plate[2]) @ rel
    truth_yaw = grip_module[2] + (stage_plate[2] - module_plate[2])
    assert np.hypot(got[0] - truth_xy[0], got[1] - truth_xy[1]) < 0.3
    assert abs(got[2] - truth_yaw) < 0.1


def test_the_plate_offset_at_the_grip_point_is_what_the_plate_moved_there():
    old = np.array([313.0, 118.0, -91.0])
    w = World(old, (0, 0, 0), seed=5)
    v0 = w.wrist_for_tag_centred(-91.0)
    ref = w.corners_at(v0, noise=False)
    grip = np.array([*(old[:2] + rot(old[2]) @ np.array([-60.0, 10.0])), old[2] + 90.0])
    w.wrist = v0 + np.array([4.0, 3.0, 2.0])
    before = servo_for(w).align(ref, start_pose=w.wrist).pose
    new = np.array([313.0 + 6.0, 118.0 - 4.0, -91.0 + 3.0])                                   # put back 6 mm, 4 mm, 3 deg off
    w.plate = new
    w.wrist = np.array(before) + np.array([-3.0, 2.0, 1.0])
    now = servo_for(w).align(ref, start_pose=w.wrist).pose
    dx, dy, dyaw = plate_offset(before, now, tuple(grip))
    rel = rot(-old[2]) @ (grip[:2] - old[:2])
    truth = new[:2] + rot(new[2]) @ rel - grip[:2]                                            # where the grip point went
    assert math.hypot(dx - truth[0], dy - truth[1]) < 0.3 and abs(dyaw - 3.0) < 0.15


def test_it_refuses_cleanly_when_the_tag_is_out_of_the_picture():
    w = World((313.0, 118.0, -91.0), (0, 0, 0), seed=2)
    v = w.wrist_for_tag_centred(-91.0)
    ref = w.corners_at(v, noise=False)
    w.wrist = v + np.array([160.0, 0.0, 0.0])                                                 # far off: the tag is not in the picture
    with pytest.raises(ServoError, match="not in the picture"):
        servo_for(w).align(ref, start_pose=w.wrist)


def test_it_refuses_cleanly_when_a_move_is_refused():
    w = World((313.0, 118.0, -91.0), (0, 0, 0), seed=2)
    v = w.wrist_for_tag_centred(-91.0)
    ref = w.corners_at(v, noise=False)
    w.wrist = v + np.array([12.0, 5.0, 3.0])

    def refuse(dx, dy, dyaw):
        raise RuntimeError("the claw would meet the hotel")
    with pytest.raises(ServoError, match="refused"):
        Servo(w.read, refuse).align(ref, start_pose=w.wrist)


def test_the_reader_finds_the_plate_tag_in_a_frame_and_none_without_one():
    import cv2
    from bringup.servo import corner_reader
    from calibration import marker
    cell = 60
    tag = np.zeros((6 * cell, 6 * cell), np.uint8)
    for i in range(4):
        for j in range(4):
            if marker.PLATE_TAG[i, j]:
                tag[(i + 1) * cell:(i + 2) * cell, (j + 1) * cell:(j + 2) * cell] = 255
    canvas = np.full((900, 1200), 235, np.uint8)
    canvas[200:200 + 6 * cell, 300:300 + 6 * cell] = tag
    ok, png = cv2.imencode(".png", canvas)
    corners = corner_reader(lambda: png.tobytes())()
    assert corners is not None and corners.shape == (4, 2)
    assert corner_reader(lambda: cv2.imencode(".png", np.full((900, 1200), 235, np.uint8))[1].tobytes())() is None
    canvas[200:200 + 6 * cell, 0:6 * cell] = tag                                               # touching the left edge: rejected
    assert corner_reader(lambda: cv2.imencode(".png", canvas)[1].tobytes())() is None


def true_offset(w, view):
    """The wrist move that puts the camera back where it was relative to the plate: the camera is ahead of the wrist, so a
    turn of the plate moves the wrist along an arc."""
    yaw = view[2] + (w.plate[2] - (-91.0))
    wrist = w.plate[:2] - rot(yaw) @ np.array([w.AHEAD, w.SIDE])
    return np.array([wrist[0] - view[0], wrist[1] - view[1], yaw - view[2]])


def _limited_world(plate_offset, seed=11):
    w = World((313.0, 118.0, -91.0), (0, 0, 0), seed=seed)
    view = w.wrist_for_tag_centred(-91.0)
    ref = w.corners_at(view, noise=False)
    w.plate = np.array([313.0, 118.0, -91.0]) + np.asarray(plate_offset, float)
    w.wrist = view.copy()
    plain = w.move

    def limited(dx, dy, dyaw):
        if math.hypot(dx, dy) > 2.0 or abs(dyaw) > 0.5:
            raise RuntimeError("inside the microscope zone: over 2 mm / 0.5 deg")
        return plain(dx, dy, dyaw)
    return w, view, ref, limited


def test_estimate_stays_inside_the_zone_limits_returns_to_the_start_and_finds_the_offset():
    w, view, ref, limited = _limited_world((1.5, -1.0, 1.5))
    start = w.wrist.copy()
    est = Servo(w.read, limited).estimate(ref)
    assert np.abs(w.wrist - start).max() < 0.5 and est.warning is None and est.moves == 12
    # the wrist offset that puts the tag back on the reference: the plate moved, the wrist follows it
    truth = true_offset(w, view)
    dx, dy, dyaw = est.offset
    assert math.hypot(dx - truth[0], dy - truth[1]) < 0.3 and abs(dyaw - truth[2]) < 0.2


def test_estimate_is_within_a_third_of_a_millimetre_up_to_a_4_mm_wrist_offset_then_warns():
    errs = {}
    for mm in (0.5, 1.0, 1.5):
        w, view, ref, limited = _limited_world((mm * 0.8, -mm * 0.6, mm))
        est = Servo(w.read, limited).estimate(ref)
        truth = true_offset(w, view)
        errs[mm] = (math.hypot(est.offset[0] - truth[0], est.offset[1] - truth[1]), abs(est.offset[2] - truth[2]), est.warning)
        assert errs[mm][0] < 0.3 and errs[mm][1] < 0.3 and errs[mm][2] is None, (mm, errs[mm])
    w, view, ref, limited = _limited_world((6.0, -4.0, 6.0))
    assert Servo(w.read, limited).estimate(ref).warning is not None


def test_estimate_refuses_when_the_tag_is_not_in_the_picture():
    w, view, ref, limited = _limited_world((0.0, 0.0, 0.0))
    w.wrist = view + np.array([160.0, 0.0, 0.0])
    with pytest.raises(ServoError, match="not in the picture"):
        Servo(w.read, limited).estimate(ref)
