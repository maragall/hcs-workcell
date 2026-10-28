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
from routes_opt import fk  # noqa: E402

LAYOUT = json.load(open(os.path.join(os.path.dirname(HERE), "layout.json"), encoding="utf-8"))


def make():
    L = copy.deepcopy(LAYOUT)
    L["routes"]["safe_z_mm"] = None
    return L, Safety(L, "supervised", joints=lambda: [62.6, 31.738, -126.448, 3.447])    # the module hover, standard branch (taught)


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


def _approach(L, slot):
    from procedures.cad_poses import CadPoses
    p = CadPoses(L).hotel_pose(slot)
    h = L["fixtures"]["hotel"]
    return [p[0] - h["approach"]["mm"], p[1], p[2] + h["lift_mm"], 0.0, 0.0, p[5]]


def test_the_way_from_the_module_to_a_hotel_slot_is_planned_with_and_without_a_plate_in_the_claw():
    L, safety = make()
    hover = list(L["fixtures"]["polar"]["pose_at_0"]["pose"])
    hover[2] += L["fixtures"]["polar"]["hover_mm"]
    for slot in ("A1", "B3", "C1", "A9"):
        for holding in (None, "long_edges"):
            if slot[0] == "B" and holding:
                continue                       # beside the arm's axis a 283 mm plate has no way in: B is for the empty claw
            ap = _approach(L, slot)
            route = planner.plan(L, safety, hover, ap, None, f"hotel approach {slot}", holding=holding)
            vol, prev, pj = Volumes(L, safety, holding), hover, None
            for i, p in enumerate(route.poses):
                if i in route.joint_targets:                                 # a joint move is the planner's own way past the axis
                    vol.check_joint_leg(prev, p, start_actual=i == 0, joints_a=pj, joints_b=route.joint_targets[i])
                    pj = route.joint_targets[i]
                    vol.branch = fk.elbow_sign(pj)          # after a joint leg the straight legs are on the branch it ended on
                else:
                    vol.check_leg(prev, p)
                    pj = None          # (the straight legs' joints are the planner's own continuity)
                prev = p
            assert route.poses[-1] == ap and not route.taught


def test_a_plate_in_the_claw_makes_the_swing_beside_the_hotel_longer_than_the_empty_claw_can_get_away_with():
    """The plate sticks out 70 mm past the fingertips: a pose that clears the hotel empty can put the plate into it."""
    L, safety = make()
    near = [185.0, 60.0, 150.0, 0.0, 0.0, 0.0]               # fingertips at 390: clear empty; the plate reaches 468
    Volumes(L, safety).check_pose(near)
    with pytest.raises(SafetyViolation, match="hotel"):
        Volumes(L, safety, "long_edges").check_pose(near)


def test_all_thirty_slots_have_an_approach_point_inside_the_workspace_and_a_planned_way_to_it():
    """3 columns x 10 rows. The top row's carry lift is cut to what the workspace allows; every approach point is then
    reachable from the module hover with a plate in the claw."""
    from procedures.hotel import Hotel

    class Stub(Hotel):
        def __init__(self, L, safety):
            self.L, self.safety = L, safety
    L, safety = make()
    L["fixtures"]["hotel"]["pose_source"] = "cad"
    hover = list(L["fixtures"]["polar"]["pose_at_0"]["pose"])
    hover[2] += L["fixtures"]["polar"]["hover_mm"]
    stub, planned = Stub(L, safety), 0
    for col in "ABC":
        for row in range(1, 11):
            slot = f"{col}{row}"
            seat, approach, _, carry = stub._hotel_geometry(slot)
            assert 4.0 <= carry[2] - seat[2] <= 14.6 and carry[2] <= safety.box["z"][1], slot       # the plate fits the 30 mm tray gap
            for holding in ((None,) if col == "B" else (None, "long_edges")):
                route = planner.plan(L, safety, hover, approach, None, f"hotel approach {slot}", holding=holding)
                assert route.poses[-1] == approach
            planned += 1
    assert planned == 30


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


def test_the_nest_is_clear_and_reachable_from_where_the_arm_stops():
    """routes.nest: a pose in front of the hotel that the planner reaches from the module, the approach point of a slot and the
    pose where the claw stopped against the hotel."""
    L, safety = make()
    nest = L["routes"]["nest"]["pose"]
    Volumes(L, safety).check_pose(nest, near_axis_ok=True)
    for here in ([312.5, 304.5, 101.2, 0, 0, -91.26], [161.7, 137.2, 99.6, 0, 0, 0.2], [257.8, 262.6, 102.7, 0, 0, -48.3]):
        route = planner.plan(L, safety, here, nest)
        assert route.poses[-1] == list(nest)


def test_the_nest_is_refused_with_a_plate_in_the_claw(tmp_path):
    from procedures.motion import Motion

    class Stub(Motion):
        def __init__(self, L):
            self.L = L
            self.store = type("S", (), {"s": type("T", (), {"grip": "long_edges"})()})()
    L, _ = make()
    with pytest.raises(SafetyViolation, match="holds a plate"):
        Stub(L).go_nest()
