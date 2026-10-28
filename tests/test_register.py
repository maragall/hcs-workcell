"""CAD-to-robot registration from the arm's own observations of the module: no taught pose, no camera calibration."""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from bringup import register  # noqa: E402
from procedures.cad_poses import CadPoses  # noqa: E402
from routes_opt import fk  # noqa: E402

ROOT = os.path.dirname(HERE)
HARDWARE = json.load(open(os.path.join(ROOT, "hardware.json"), encoding="utf-8"))
CAD = HARDWARE["cad"]
LAYOUT = json.load(open(os.path.join(ROOT, "layout.json"), encoding="utf-8"))
THETA = 0.22                                           # the rotation the cell has (the taught hotel poses give 0.224)
J1 = np.array([fk.MODEL["x0"], fk.MODEL["y0"]])


def truth_axis(theta=THETA):
    return register._rot(theta) @ (register.m(118.0, 84.5) - register.m(0.0, -102.1)) + J1


def views(v0, deg, rng, noise_mm=0.1, noise_deg=0.05, theta=THETA):
    """The wrist pose that re-finds the tag after the module turned the plate by `deg` about its axis."""
    A = truth_axis(theta)
    p = register._rot(deg) @ (np.array(v0[:2]) - A) + A
    return (p[0] + rng.normal(0, noise_mm), p[1] + rng.normal(0, noise_mm), v0[2] + deg + rng.normal(0, noise_deg))


def test_the_module_axis_is_the_fixed_point_of_the_wrist_motion_whatever_the_camera_does():
    rng = np.random.default_rng(1)
    A = truth_axis()
    for start in ((A[0] + 5, A[1] + 100, -91.3), (A[0] - 60, A[1] + 70, -80.0), (A[0] + 90, A[1] + 120, -100.0)):
        for deg in (90.0, -90.0, 180.0):
            axis, turn, warn = register.module_axis(start, views(start, deg, rng), commanded_deg=deg)
            assert np.linalg.norm(np.array(axis) - A) < 0.4 and abs(abs(turn) - abs(deg)) < 0.3 and not warn


def test_a_registration_from_one_view_pair_gets_the_frames_rotation_and_shift_to_a_fraction_of_a_degree_and_a_millimetre():
    rng = np.random.default_rng(2)
    A = truth_axis()
    v0 = (A[0], A[1] + 100.0, -91.3)
    reg = register.register(CAD, v0, views(v0, 90.0, rng), commanded_deg=90.0)
    taught_fit = CadPoses(LAYOUT, HARDWARE).fit
    assert abs(reg.theta_deg - THETA) < 0.1 and abs(reg.theta_deg - taught_fit["theta_deg"]) < 0.1
    true_t = J1 - register._rot(THETA) @ register.m(0.0, -102.1)
    assert abs(reg.tx - true_t[0]) < 0.3 and abs(reg.ty - true_t[1]) < 0.3
    assert not reg.warnings and abs(reg.checks["distance_difference_mm"]) < 0.4
    assert reg.k == register.K_ASSUMED and "assumed" in reg.k_source


def test_the_hotel_poses_from_the_registration_put_the_plates_where_the_taught_poses_say_to_within_the_teaching_scatter():
    """Taught wrist pose + the CAD grip reach along its own yaw is the plate centre the teaching implies; the registration puts
    the CAD plate centre within a systematic few mm along the claw (the grip reach, learned at the first grasp) and 1-2 mm of
    scatter."""
    rng = np.random.default_rng(3)
    A = truth_axis()
    v0 = (A[0], A[1] + 100.0, -91.3)
    reg = register.register(CAD, v0, views(v0, 90.0, rng), commanded_deg=90.0)
    reach = CAD["grip"]["head_centre_ahead_mm"] + CAD["grip"]["long_edges_back_mm"]
    diffs = []
    for slot, pose in LAYOUT["fixtures"]["hotel"]["taught"].items():
        if slot[0] not in "ABC" or not slot[1:].isdigit():
            continue
        yaw = math.radians(pose[5])
        taught_centre = np.array([pose[0] + reach * math.cos(yaw), pose[1] + reach * math.sin(yaw)])
        cad_centre = np.array(reg.xy(CAD["hotel"]["columns_y_r"][slot[0]], 253.5))
        diffs.append(taught_centre - cad_centre)
    diffs = np.array(diffs)
    mean = diffs.mean(0)
    assert np.linalg.norm(mean) < 5.0                                  # the grip reach (219.9 taught against 216.7), documented
    assert np.linalg.norm(diffs - mean, axis=1).max() < 3.0            # what is left is the teaching scatter (B1 was taught 4 mm off the CAD column)
    # and the trim the first grasp would learn closes the systematic part
    trimmed = CadPoses(LAYOUT, HARDWARE, registration=reg, reach_trim_mm=3.2)
    plain = CadPoses(LAYOUT, HARDWARE, registration=reg)
    assert 3.0 < plain.hotel_pose("A1")[0] - trimmed.hotel_pose("A1")[0] < 3.3
    assert abs(trimmed.hotel_pose("A1")[2] - 73.8) < 0.2 and abs(trimmed.hotel_pose("A1")[5] - reg.theta_deg) < 1e-9


def test_the_height_comes_from_the_tags_size_at_two_heights_without_a_focal_length():
    rng = np.random.default_rng(4)
    z_tag, above, C = 74.1, 35.0, 2632.0 * 40.0
    zs = [150.0, 190.0, 230.0]
    sides = [C / (z + above - z_tag) + rng.normal(0, 0.3) for z in zs]
    got, resid = register.height_from_scale(list(zip(zs, sides)), above)
    assert abs(got - z_tag) < 0.6 and resid < 0.01
    A = truth_axis()
    v0 = (A[0], A[1] + 100.0, -91.3)
    reg = register.register(CAD, v0, views(v0, 90.0, rng), commanded_deg=90.0, scale_views=list(zip(zs, sides)))
    assert abs(reg.k - (-57.4)) < 0.8 and "size" in reg.k_source
    with pytest.raises(register.RegistrationError, match="two wrist heights"):
        register.height_from_scale([(150.0, 600.0), (155.0, 590.0)], above)


def test_views_with_too_little_turn_or_a_module_that_did_not_turn_as_sent_are_refused_or_flagged():
    rng = np.random.default_rng(5)
    A = truth_axis()
    v0 = (A[0], A[1] + 100.0, -91.3)
    with pytest.raises(register.RegistrationError, match="turn the plate by 90"):
        register.register(CAD, v0, views(v0, 2.0, rng), commanded_deg=2.0)
    reg = register.register(CAD, v0, views(v0, 80.0, rng), commanded_deg=90.0)
    assert any("yaw scale" in w for w in reg.warnings)


def test_a_wrong_j1_or_link_length_shows_as_a_distance_the_cad_does_not_have():
    rng = np.random.default_rng(6)
    A = truth_axis()
    v0 = (A[0], A[1] + 100.0, -91.3)
    reg = register.register(CAD, v0, views(v0, 90.0, rng), commanded_deg=90.0, j1_robot=(J1[0] + 4.0, J1[1]))
    assert any("from j1" in w for w in reg.warnings)


def test_three_module_angles_agree_on_one_axis_when_the_arm_is_right_and_not_when_a_link_is_off():
    A = truth_axis()
    start = (A[0], A[1] + 100.0, -91.3)
    ok = {}
    for a in (0.0, 90.0, 180.0):
        p = register._rot(a) @ (np.array(start[:2]) - A) + A
        ok[a] = (p[0], p[1], start[2] + a)
    check = register.arm_selfcheck(ok, commanded=(0.0, 90.0, 180.0))
    assert check.ok and check.residual_mm < 0.05 and np.linalg.norm(np.array(check.centre) - A) < 0.05
    bent = dict(ok)
    x, y, yaw = bent[180.0]
    bent[180.0] = (x + 2.5, y - 1.5, yaw)                         # a link length or joint-3 zero that is off at that configuration
    bad = register.arm_selfcheck(bent, commanded=(0.0, 90.0, 180.0))
    assert not bad.ok and bad.residual_mm > 1.0 and any("disagree" in n for n in bad.notes)
    with pytest.raises(register.RegistrationError):
        register.arm_selfcheck({0.0: start, 90.0: start}, None)
