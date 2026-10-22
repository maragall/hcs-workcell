"""The camera check before the stage rotates: collects references first, then blocks on a wrong scene."""
from __future__ import annotations

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from rig import BAD, GATES, layout  # noqa: E402
from test_sim import Rig  # noqa: E402

from procedures.corrector import StopSafe  # noqa: E402
from procedures.preflight import preflight  # noqa: E402


def test_stage_rotation_is_blocked_by_the_camera_once_trained():
    r = Rig(refs=GATES + [("stage", "to_90")])        # three references, and auto_refs lowered to match
    r.cyc.L["vision"]["stage_check"]["auto_refs"] = 3
    P0 = r.L["fixtures"]["polar"]["pose_at_0"]["pose"]
    r.robot.set_pose_for_test(P0)
    r.vis.scene["auto_teach_camera"] = BAD             # something on the stage that was never seen before
    try:
        with pytest.raises(StopSafe, match="clear to rotate"):
            r.cyc._polar_rotate(90, P0)
        assert r.robot._ext.get(1) == r.L["polar"]["ext_axis"]["zero_reading"]   # the stage did not turn
    finally:
        r.close()


def test_stage_rotation_collects_references_while_untrained():
    r = Rig()
    P0 = r.L["fixtures"]["polar"]["pose_at_0"]["pose"]
    r.robot.set_pose_for_test(P0)
    try:
        r.cyc._polar_rotate(90, P0)
        assert r.cyc.lower.count("stage", "to_90") == 1
    finally:
        r.close()


def test_preflight_refuses_a_hotel_lift_at_or_above_the_headroom():
    L = layout()
    L["fixtures"]["hotel"]["headroom_mm"] = 17
    L["fixtures"]["hotel"]["lift_mm"] = 25.0
    assert any("headroom" in p for p in preflight(L, "/nonexistent"))
    L["fixtures"]["hotel"]["lift_mm"] = 8.0
    assert not any("headroom" in p for p in preflight(L, "/nonexistent"))


def test_front_camera_blocks_the_lowering_once_trained():
    r = Rig(refs=GATES + [("microscope_front", "plate_present")])
    r.cyc.L["vision"]["lower_check"]["front"]["auto_refs"] = 3
    r.vis.scene["scan_camera"] = BAD                   # the down camera still looks fine, the front view does not
    try:
        r.expect_stop("front camera")
    finally:
        r.close()
