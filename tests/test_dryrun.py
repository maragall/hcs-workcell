"""The collision oracle: a simulated arm in the real frame that checks every motion against the CAD volumes."""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from bringup.dryrun import OracleRobot, hotel_visit, slot_window  # noqa: E402
from procedures import positions  # noqa: E402
from procedures.hotel import Hotel  # noqa: E402

L = positions.load(os.path.join(os.path.dirname(HERE), "layout.json"))


def test_the_oracle_flags_the_taught_route_that_stopped_the_arm_against_the_hotel():
    """polar_to_hotel replayed from the module hover: the claw is inside the hotel's front from the third waypoint on, which is
    where the controller reported 'axis 4 collision detected' on 2026-10-27."""
    tr = L["trajectories"]["polar_to_hotel"]
    robot = OracleRobot(L, list(tr["poses"][0]), raise_on_collision=False)
    robot._joints = list(tr["joints"][0][:4]) + [0.0] * 4
    for j in tr["joints"][1:]:
        robot.move_abs_joints(list(j))
    hits = robot.report.collisions
    assert hits and all("hotel" in e.note for e in hits)
    assert min(e.pose[0] for e in hits) > 255.0                 # on the way in, not at the module


def test_planned_hotel_visits_in_the_outer_columns_collide_with_nothing_with_or_without_a_lid():
    for lid in (False, True):
        for slot in ("A1", "C5", "A10"):
            report, err = hotel_visit(L, slot, lid=lid)
            assert err is None, f"{slot} lid={lid}: {err}"
            assert not report.collisions


def test_the_carry_lift_follows_the_gap_between_two_trays():
    class Stub(Hotel):
        def __init__(self, rows):
            import copy
            self.L = copy.deepcopy(L)
            self.L["fixtures"]["hotel"]["rows"] = rows
            self.safety = type("S", (), {"box": {"z": (45.0, 435.0)}})()
    thirty = Stub(list(range(1, 11)))._lift_for([260.0, 137.0, 74.0])
    fifteen = Stub([1, 3, 5, 7, 9])._lift_for([260.0, 137.0, 74.0])
    lo, hi = slot_window(L["plate"]["height"], lid=True)
    assert lo <= thirty <= hi + 1.5 and thirty < 14.0           # 30 slots: the plate has to fit a 30 mm gap
    assert fifteen == L["fixtures"]["hotel"]["lift_mm"]          # 15 slots keep the configured lift


def test_the_taught_route_between_the_microscope_and_the_module_collides_with_nothing_either_way_with_or_without_a_plate():
    for name in ("microscope_to_polar", "microscope_to_polar_fast"):
        tr = L["trajectories"][name]
        for reverse in (False, True):
            pts = list(zip(tr["joints"], tr["poses"]))[::-1 if reverse else 1]
            for holding in (None, "short_edges"):
                robot = OracleRobot(L, list(pts[0][1]), grip=lambda h=holding: h, raise_on_collision=False)
                robot._joints = list(pts[0][0][:4]) + [0.0] * 4
                for j, _ in pts[1:]:
                    robot.move_abs_joints(list(j))
                assert not robot.report.collisions, (name, reverse, holding, robot.report.lines()[:2])
