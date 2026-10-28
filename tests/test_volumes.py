"""The planner models the claw (205 mm ahead of the wrist) and routes around the hotel and the pillar."""
from __future__ import annotations

import copy
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from procedures.safety import Safety, SafetyViolation  # noqa: E402
from routes import plan as planner  # noqa: E402
from routes.volumes import Volumes  # noqa: E402

LAYOUT = json.load(open(os.path.join(os.path.dirname(HERE), "layout.json"), encoding="utf-8"))


def make():
    L = copy.deepcopy(LAYOUT)
    L["routes"]["safe_z_mm"] = None
    return L, Safety(L, "supervised", joints=lambda: [150.0, 30.0, -90.0, 0.0])    # the elbow branch of the taught routes


def test_the_pose_that_crashed_the_arm_is_inside_the_hotel_for_the_claw_though_not_for_the_wrist():
    L, safety = make()
    crash = [199.3, 121.0, 335.0, 0.0, 0.0, 9.0]       # the pose the arm stopped at on 2026-10-27
    with pytest.raises(SafetyViolation, match="hotel"):
        Volumes(L, safety).check_pose(crash)
    safety.check_target(crash, "wrist only")           # the old check passed it


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
