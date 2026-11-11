"""The planner models the claw (205 mm ahead of the wrist) and routes around the hotel and the pillar."""
from __future__ import annotations

import copy
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from run.safety import Safety, SafetyViolation  # noqa: E402
from planner import plan as planner  # noqa: E402
from planner.volumes import Volumes  # noqa: E402
from planner import fk  # noqa: E402

LAYOUT = json.load(open(os.path.join(os.path.dirname(HERE), "layout.json"), encoding="utf-8"))


def make():
    L = copy.deepcopy(LAYOUT)
    L["routes"]["safe_z_mm"] = None
    return L, Safety(L, "supervised", joints=lambda: [62.6, 31.738, -126.448, 3.447])    # the module hover, standard branch (taught)


def test_the_claw_pointing_along_y_is_clear_at_the_same_spot():
    L, safety = make()
    Volumes(L, safety).check_pose([199.3, 121.0, 335.0, 0.0, 0.0, 90.0])


def test_a_route_to_the_module_hover_exists_and_every_leg_keeps_the_claw_out_of_the_hotel():
    L, safety = make()
    here = [168.2, 61.6, 150.2, 0.0, 0.0, 9.0]
    target = copy.deepcopy(L["fixtures"]["polar"]["pose_at_0"]["pose"])
    target[2] += L["fixtures"]["polar"].get("hover_mm", 30.0)
    route = planner.plan(L, safety, here, target)
    vol, prev = Volumes(L, safety), here
    for p in route.poses:
        vol.check_leg(prev, p)
        prev = p
    assert route.poses[-1] == target and len(route.poses) >= 2


def test_a_start_with_the_claw_inside_the_hotel_is_refused_with_the_volume_named():
    L, safety = make()
    with pytest.raises(SafetyViolation, match="hotel"):
        planner.plan(L, safety, [199.3, 121.0, 335.0, 0.0, 0.0, 9.0], [300.0, 150.0, 200.0, 0.0, 0.0, 0.0])


def _approach(L, slot):
    from run.cad_poses import CadPoses
    p = CadPoses(L).hotel_pose(slot)
    h = L["fixtures"]["hotel"]
    return [p[0] - h["approach"]["mm"], p[1], p[2] + h["lift_mm"], 0.0, 0.0, p[5]]


def test_a_plate_in_the_claw_makes_the_swing_beside_the_hotel_longer_than_the_empty_claw_can_get_away_with():
    """The plate sticks out 70 mm past the fingertips: a pose that clears the hotel empty can put the plate into it."""
    L, safety = make()
    near = [185.0, 60.0, 150.0, 0.0, 0.0, 0.0]               # fingertips at 390: clear empty; the plate reaches 468
    Volumes(L, safety).check_pose(near)
    with pytest.raises(SafetyViolation, match="hotel"):
        Volumes(L, safety, "long_edges").check_pose(near)


def test_an_arm_that_stopped_against_the_hotel_backs_out_along_its_claw_before_anything_else():
    """The pose where the claw touched the hotel on 2026-10-27: the planner does not refuse to move, it withdraws straight
    back along the claw, then plans from the clear spot."""
    L, safety = make()
    stuck = [257.8, 262.6, 102.7, 0.0, 0.0, -48.3]
    with pytest.raises(SafetyViolation, match="hotel"):
        Volumes(L, safety).settle(stuck)
    target = list(L["fixtures"]["polar"]["pose_at_0"]["pose"])
    route = planner.plan(L, safety, stuck, target)
    first = route.poses[0]
    back = ((first[0] - stuck[0]) ** 2 + (first[1] - stuck[1]) ** 2) ** 0.5
    assert 5.0 <= back <= 100.0 and abs(first[5] - stuck[5]) < 1e-9          # straight back, claw not turned
    Volumes(L, safety).check_pose(first)                                     # the spot it backs out to is clear with the full margin
    assert route.poses[-1] == target


