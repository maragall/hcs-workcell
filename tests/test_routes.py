"""Route planning and move_to on the simulator."""
from __future__ import annotations

import os
import shutil
import sys
import tempfile
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from rig import FakeVision, layout  # noqa: E402

from devices.fake_arm import SimFaults, SimRobot  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.safety import Safety, SafetyViolation  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402
from routes.plan import plan  # noqa: E402

FREE_A = [200.0, 100.0, 120.0, 0, 0, 0]          # outside every region
FREE_B = [300.0, 180.0, 100.0, 0, 0, 0]


def test_lift_travel_lower_gives_three_checked_legs():
    L = layout()
    r = plan(L, Safety(L, "supervised"), FREE_A, FREE_B)
    assert r.name == "lift-travel-lower" and not r.taught
    assert [p[2] for p in r.poses] == [160.0, 160.0, 100.0]
    assert r.poses[1][:2] == FREE_B[:2] and r.poses[-1] == FREE_B


def test_refuses_without_a_safe_height_or_a_taught_route():
    L = layout()
    L["routes"]["safe_z_mm"] = None
    with pytest.raises(SafetyViolation, match="safe_z_mm"):
        plan(L, Safety(L, "supervised"), FREE_A, FREE_B)


def test_refuses_to_travel_sideways_into_the_no_swing_zone():
    L = layout()
    scope = L["fixtures"]["microscope"]["pose"]
    with pytest.raises(SafetyViolation, match="lateral"):
        plan(L, Safety(L, "supervised"), FREE_A, scope)


def test_a_taught_route_wins_and_reverses():
    L = layout()
    r = plan(L, Safety(L, "supervised"), FREE_A, FREE_B, "microscope", "polar.pose_at_0")
    assert r.taught and r.name == "microscope_to_polar" and not r.reverse
    back = plan(L, Safety(L, "supervised"), FREE_B, FREE_A, "polar", "microscope")
    assert back.reverse and back.poses == r.poses[::-1] and back.joints == r.joints[::-1]


def test_move_to_on_the_simulator_ends_at_the_hover():
    tmp = tempfile.mkdtemp()
    L = layout()
    L["fixtures"]["free_spot"] = {"pose": FREE_B, "hover_mm": 20}
    tele = Telemetry(os.path.join(tmp, "run"))
    robot = SimRobot(FREE_A, SimFaults(), tele.log, jaw=10.0)
    cyc = Cycle(L, robot, FakeVision(), tele, StateStore(os.path.join(tmp, "state.json")), "unsupervised",
                os.path.join(tmp, "run"), os.path.join(tmp, "refs"))
    try:
        assert cyc.clock is cyc.polar.clock is cyc.corr.clock is time     # nothing injected: the waits are real seconds
        cyc.move_to("free_spot")
        assert robot.pose()[:3] == [300.0, 180.0, 120.0]
        assert [k for k, _ in robot.trace] == ["L", "L", "L"]           # lift, travel, lower
    finally:
        tele.close()
        shutil.rmtree(tmp)
