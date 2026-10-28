"""After a collision or controller fault the arm goes to the nest and the run stops for good."""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from test_sim import Rig  # noqa: E402

from devices.fake_arm import SimFaults  # noqa: E402
from procedures.recovery import is_motion_fault, mark, recover_to_nest  # noqa: E402

COLLISION = [{"errCode": 3873, "errMsg": "Motion error: axis 4 collision detected"}]


def test_a_stop_from_the_controller_is_a_motion_fault_and_a_vision_stop_is_not():
    assert is_motion_fault("polar_to_hotel[5/5]: controller error [3873] (possible collision); move stopped")
    assert is_motion_fault("controller lost enable")
    assert not is_motion_fault("lower blocked at microscope")
    assert not is_motion_fault("grasp failed 3 times at polar")


def test_after_a_collision_the_errors_clear_the_arm_enables_and_goes_to_the_nest():
    r = Rig(SimFaults(refuse_move_at=3, controller_errors=list(COLLISION)))
    try:
        r.expect_stop("controller error")
        assert r.robot.errors()                                  # the controller still holds the fault
        r.robot.faults.controller_errors = []                    # what clear_errors on the real controller does
        r.robot.faults.refuse_move_at = -1
        out = recover_to_nest(r.robot, r.cyc, r.tele.log)
        assert out.endswith("at the nest"), out
        nest = r.L["routes"]["nest"]["pose"]
        assert max(abs(a - b) for a, b in zip(r.robot.pose()[:3], nest[:3])) < 1.0
    finally:
        r.close()


def test_with_a_plate_in_the_claw_the_arm_stays_where_it_stopped_and_says_so():
    r = Rig()
    try:
        r.store.s.plate_at, r.store.s.grip = "gripper", "long_edges"
        before = r.robot.pose()
        out = recover_to_nest(r.robot, r.cyc, r.tele.log)
        assert "holds a plate" in out and r.robot.pose() == before
    finally:
        r.close()


def test_an_error_the_controller_will_not_clear_leaves_the_arm_alone():
    r = Rig()
    try:
        r.robot.faults.controller_errors = list(COLLISION)
        r.robot.clear_errors = lambda: None                      # the controller keeps reporting it
        before = r.robot.pose()
        out = recover_to_nest(r.robot, r.cyc, r.tele.log)
        assert "still reports" in out and r.robot.pose() == before
    finally:
        r.close()


def test_the_marker_names_the_reason_for_the_supervisor(tmp_path):
    path = mark(str(tmp_path), "controller error 3873 | at the nest")
    assert os.path.basename(path) == "COLLISION" and "3873" in open(path, encoding="utf-8").read()
