"""The two stage layers offline: the keep-out (tool point, elbow and links via the fitted FK) and the parked interlock."""
from __future__ import annotations

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from rig import layout  # noqa: E402
from test_sim import Rig  # noqa: E402

from procedures.preflight import preflight  # noqa: E402
from procedures.safety import Safety, SafetyViolation  # noqa: E402
from routes_opt import fk  # noqa: E402

J_A, J_B = [100.0, 55.0, -170.0, 0.0], [100.0, 85.0, -170.0, 0.0]     # j1 column mm, j2 shoulder, j3 elbow, j4 wrist
A, B = fk.fk(J_A), fk.fk(J_B)                                          # a 30 deg sweep near full reach, both in the workspace


def around(xy, mm=8.0):
    return {"x": [xy[0] - mm, xy[0] + mm], "y": [xy[1] - mm, xy[1] + mm]}


def with_keep_out(box):
    L = layout()
    L["regions"]["stage_sweep"]["box"] = box
    return L


def test_segment_refused_when_the_elbow_crosses_the_keep_out_and_the_tool_point_does_not():
    mid = [(a + b) / 2 for a, b in zip(A, B)]
    box = around(fk.elbow_xy(fk.ik(mid, elbow=1)))
    for p in (A, mid, B):
        assert not (box["x"][0] <= p[0] <= box["x"][1] and box["y"][0] <= p[1] <= box["y"][1])   # the tool stays out
    s = Safety(with_keep_out(box), "supervised", joints=lambda: J_A)
    with pytest.raises(SafetyViolation, match="stage_sweep keep-out") as e:
        s.check_segment(A, B, "sweep")
    assert "tool point" not in str(e.value)
    with pytest.raises(SafetyViolation, match="stage_sweep keep-out"):
        Safety(with_keep_out(box), "supervised").check_segment(A, B, "sweep")      # branch unknown: both elbows must clear


def test_target_and_joint_move_refused_on_the_elbow_too():
    box = around(fk.elbow_xy(J_B))
    s = Safety(with_keep_out(box), "supervised", joints=lambda: J_A)
    with pytest.raises(SafetyViolation, match="elbow .* stage_sweep keep-out"):
        s.check_target(B, "arrive")
    swing = around(fk.elbow_xy([100.0, 70.0, -170.0, 0.0]))                        # the elbow half way through the swing
    s = Safety(with_keep_out(swing), "supervised", joints=lambda: J_A)
    with pytest.raises(SafetyViolation, match="stage_sweep keep-out"):
        s.check_joint_move(A, B, "swing")


def test_tool_point_in_the_keep_out_is_refused_by_name():
    s = Safety(with_keep_out(around(B[:2])), "supervised", joints=lambda: J_A)
    with pytest.raises(SafetyViolation, match="tool point .* stage_sweep keep-out"):
        s.check_target(B, "arrive")


def test_null_box_is_inactive_and_preflight_lists_it():
    L = layout()
    assert L["regions"]["stage_sweep"]["box"] is None
    s = Safety(L, "supervised", joints=lambda: J_A)
    assert s.keep_outs == {}
    s.check_segment(A, B, "sweep")
    s.check_joint_move(A, B, "swing")
    assert s.region_of(A) != "stage_sweep"
    assert "regions.stage_sweep keep-out box not set" in preflight(L, "/nonexistent")
    L["regions"]["stage_sweep"]["box"] = around((0.0, 0.0))
    assert not any("keep-out" in p for p in preflight(L, "/nonexistent"))


def test_preflight_lists_unmeasured_park_numbers_for_a_real_squid_only():
    L = layout()
    problems = preflight(L, "/nonexistent", ["microscope"], squid=["microscope"])
    assert {"squid.microscope.park_xy_mm not set", "squid.microscope.park_tolerance_mm not set"} <= set(problems)
    assert not any(p.startswith("squid.") for p in preflight(L, "/nonexistent", ["microscope"]))


def test_cycle_stays_out_of_the_microscope_zone_while_the_stage_is_not_parked():
    r = Rig()
    r.cyc.parked = lambda: False
    try:
        r.expect_stop("stage is not parked")
        assert r.robot._moves == 0
    finally:
        r.close()
