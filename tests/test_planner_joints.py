"""The planner knows the arm's joints: limits, the elbow branch, the direction of a turn, and the way between branches."""
import copy
import json
import os
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from procedures.safety import Safety, SafetyViolation  # noqa: E402
from routes import joints as jt  # noqa: E402
from routes import plan as planner  # noqa: E402
from routes.volumes import Volumes  # noqa: E402
from routes_opt import fk  # noqa: E402

LAYOUT = json.load(open(os.path.join(os.path.dirname(HERE), "layout.json"), encoding="utf-8"))
NEST = LAYOUT["routes"]["nest"]
LIM = jt.limits(LAYOUT)


def make(joints):
    L = copy.deepcopy(LAYOUT)
    L["routes"]["safe_z_mm"] = None
    return L, Safety(L, "supervised", joints=lambda: list(joints))


def drive(L, safety, joints, route):
    """What Motion.go_to does, offline: every leg's end joints, the explicit ones for the joint legs, IK on the branch for the
    straight legs. Returns the list of joints after each leg."""
    cur, out = list(joints), []
    branch = fk.elbow_sign(cur)
    poses_prev = fk.fk(cur)
    for i, p in enumerate(route.poses):
        if i in route.joint_targets:
            nxt = list(route.joint_targets[i])
            assert max(abs(nxt[k] - cur[k]) for k in (1, 2, 3)) < 400
        else:
            nxt = jt.leg_joints(poses_prev, p, branch, LIM, start=cur)[1]
            assert max(abs(nxt[k] - cur[k]) for k in (1, 2, 3)) < 100, f"leg {i} would jump the joints"
        out.append(nxt)
        cur, branch, poses_prev = nxt, fk.elbow_sign(nxt), p
    return out


def test_the_limits_are_named_and_the_layout_can_override_them():
    assert LIM["j2"][1] >= 94.0 and LIM["j3"][0] == -180.0
    L = copy.deepcopy(LAYOUT)
    L.setdefault("limits", {})["joint_limits_deg"] = {"j2": [-100, 100]}
    assert jt.limits(L)["j2"] == (-100.0, 100.0)


def test_a_leg_that_leaves_the_elbow_range_is_refused_with_the_joint_named():
    # the straight leg the controller refused on 2026-10-28, on the branch the arm was on
    why = jt.leg_feasible([296.4, 278.2, 232.7, 0, 0, -91.3], [178.2, 85.3, 232.7, 0, 0, -91.3], -1, LIM,
                          start=[194.1, 93.9, 109.3, 65.6])
    assert why and "j2" in why and "mirror" in why


def test_a_turn_in_place_goes_the_short_way_inside_the_limits():
    here = fk.fk([194.1, 93.9, 109.3, -23.0])               # yaw about 180 on the mirror branch
    there = list(here)
    there[5] = -91.3
    ja, jb, turn = jt.leg_joints(here, there, -1, LIM, start=[194.1, 93.9, 109.3, -23.0])
    assert abs(turn) < 100 and jb[3] > ja[3]               # 88 degrees one way, not 272 the other
    assert LIM["j4"][0] < jb[3] < LIM["j4"][1]


def test_a_wrist_dragged_past_its_range_leaves_toward_the_range():
    start = [194.1, 93.9, 109.3, 221.0]                      # j4 outside the range the routes ever used
    here = fk.fk(start)
    there = list(here)
    there[5] = fk.wrap180(here[5] - 100.0)
    ja, jb, turn = jt.leg_joints(here, there, -1, LIM, start=start)
    assert jb[3] < ja[3]                                     # toward the range
    assert jt.leg_feasible(here, there, -1, LIM, start=start) is None


def test_from_the_mirror_branch_with_the_claw_turned_the_route_turns_by_joints_and_the_short_way():
    L, safety = make([194.1, 93.9, 109.3, -23.0])
    here = fk.fk([194.1, 93.9, 109.3, -23.0])
    hover = list(L["fixtures"]["polar"]["pose_at_0"]["pose"])
    hover[2] += L["fixtures"]["polar"]["hover_mm"]
    route = planner.plan(L, safety, here, hover, None, "polar.pose_at_0")
    ends = drive(L, safety, [194.1, 93.9, 109.3, -23.0], route)
    for i, p in enumerate(route.poses):
        if i in route.joint_targets:
            assert LIM["j4"][0] <= route.joint_targets[i][3] <= LIM["j4"][1]
    assert max(abs(j[3]) for j in ends) <= LIM["j4"][1]
    assert route.poses[-1] == hover


def test_the_real_failure_from_the_microscope_hover_reaches_the_nest_on_joints_the_controller_can_drive():
    """2026-10-28 02:16: dragged to the microscope hover on the mirror branch, the straight leg to the nest was refused."""
    start = [194.1, 93.9, 109.3, 65.6]
    L, safety = make(start)
    here = fk.fk(start)
    t0 = time.time()
    route = planner.plan(L, safety, here, list(NEST["pose"]), None, "nest")
    assert time.time() - t0 < 30
    ends = drive(L, safety, start, route)
    assert route.last_joint and route.last_joints == pytest.approx(NEST["joints_at_teach"][:4], abs=1e-6)
    vol, prev = Volumes(L, safety), here
    for i, p in enumerate(route.poses):                      # every leg again, as the controller would drive it
        if i in route.joint_targets:
            vol.check_joint_leg(prev, p, start_actual=i == 0, joints_a=ends[i - 1] if i else None, joints_b=route.joint_targets[i])
        else:
            vol.check_leg(prev, p)
        prev = p
    for j in ends:
        assert jt.inside(j, LIM, 0.0) is None or j is ends[0]


def test_a_branch_change_goes_through_the_nest_and_every_joint_leg_carries_its_joints():
    """The standard-branch hover to column C: only reachable on the mirror branch (taught C9 has j3 = +64)."""
    start = [62.6, 31.738, -126.448, 3.447]
    L, safety = make(start)
    hover = list(L["fixtures"]["polar"]["pose_at_0"]["pose"])
    hover[2] += L["fixtures"]["polar"]["hover_mm"]
    from procedures.cad_poses import CadPoses
    h = L["fixtures"]["hotel"]
    p = CadPoses(L).hotel_pose("C1")
    target = [p[0] - h["approach"]["mm"], p[1], p[2] + h["lift_mm"], 0.0, 0.0, p[5]]
    route = planner.plan(L, safety, hover, target, None, "hotel approach C1")
    assert route.name == "via-the-nest" and 0 in route.joint_targets
    assert route.joint_targets[0] == pytest.approx(NEST["joints_at_teach"][:4], abs=1e-6)
    ends = drive(L, safety, start, route)
    assert fk.elbow_sign(ends[-1]) == -1                     # arrived on the mirror branch
    assert route.poses[-1] == target


def test_a_plan_takes_seconds_not_minutes():
    start = [194.1, 93.9, 109.3, 65.6]
    L, safety = make(start)
    t0 = time.time()
    planner.plan(L, safety, fk.fk(start), list(NEST["pose"]), None, "nest")
    assert time.time() - t0 < 30


def test_without_the_arms_joints_the_planner_behaves_as_before():
    L = copy.deepcopy(LAYOUT)
    L["routes"]["safe_z_mm"] = None
    safety = Safety(L, "supervised")
    assert Volumes(L, safety).branch is None
    route = planner.plan(L, safety, [250.0, 100.0, 233.0, 0, 0, 90.0], [250.0, -60.0, 233.0, 0, 0, 90.0], None, "x")
    assert not route.joint_targets
