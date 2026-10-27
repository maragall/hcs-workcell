"""The one-click setup on the simulator: every section runs in order with scripted Enters, refusals stop it with a
message and leave the files alone, a second run recognises the hotel it learned."""
from __future__ import annotations

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from bringup import geometry  # noqa: E402
from bringup.runner import Runner  # noqa: E402
from bringup.sections import Bringup  # noqa: E402
from devices.fake_squid import FakeSquid  # noqa: E402
from gui import sim  # noqa: E402
from gui.cell import Cell  # noqa: E402
from perception.hotel_size import HotelSize  # noqa: E402


def world_markers(cell, stage_shift=(0.0, 0.0)):
    """The practice plate's marker as the box would read it: on the module it sits 3 mm and 2 mm off the grip point,
    on the stage the same plate sits at the microscope seat (plus stage_shift, a microscope that moved)."""
    from devices.vision_box import ArucoPose
    F = cell.layout()["fixtures"]
    p0, seat = F["polar"]["pose_at_0"]["pose"], F["microscope"]["pose"]
    module = [p0[0] - 3.0, p0[1] + 2.0, p0[2] - 20.0, 0, 0, p0[5] + 30.0]
    stage = [seat[0] - 3.0 + stage_shift[0], seat[1] + 2.0 + stage_shift[1], seat[2] - 20.0, 0, 0, seat[5] + 30.0]

    def aruco_pose(camera, marker_id=1, size=None, led=0):
        here = cell.robot.pose()
        near_module = (here[0] - p0[0]) ** 2 + (here[1] - p0[1]) ** 2 < (here[0] - seat[0]) ** 2 + (here[1] - seat[1]) ** 2
        return ArucoPose([1.0, 1.0, 1.0, 1.0], [0.0] * 6, list(module if near_module else stage), "robot", None)

    cell.vision.aruco_pose = aruco_pose


def make(answers=None):
    cell = Cell(*sim.build(), sim=True)
    cell.connect()
    cell.discover()
    said, asked = [], []
    answers = dict(answers or {"15 or 30": "30"})

    def ask(prompt):
        asked.append(prompt)
        for key, value in answers.items():
            if key in prompt:
                return value
        return ""

    runner = Runner(say=said.append, ask=ask, stop=cell.stop_event)
    L = cell.layout()
    L["squid"]["microscope"]["stage_limits_mm"] = [[0, 100], [0, 75]]
    cell.save(L)
    squid = FakeSquid("microscope")
    world_markers(cell)
    return cell, Bringup(cell, squid, ask), runner, said, asked


def test_geometry_carries_the_grip_pose_with_the_marker():
    rel = geometry.grip_in_marker_frame([100, 200, 10, 0, 0, 30], [105, 210, 12, 0, 0, 120])
    seat = geometry.seat_from_marker([300, 50, 20, 0, 0, 30], rel, 120, [0, 0, 0, 0, 0, 0])
    assert seat[2] == pytest.approx(22.0) and seat[5] == pytest.approx(120.0)
    assert geometry.moved_mm([300, 50, 20], seat) == pytest.approx(((seat[0] - 300) ** 2 + (seat[1] - 50) ** 2 + 4) ** 0.5, abs=1e-6)
    # the jaw is symmetric: the yaw is taken modulo 180 nearest to the old one
    flipped = geometry.seat_from_marker([300, 50, 20, 0, 0, 210], rel, 120, [0, 0, 0, 0, 0, 0])
    assert abs(geometry.wrap(flipped[5] - 120)) <= 90


def test_hotel_size_learns_what_the_user_says_then_recognises_it(tmp_path):
    from tests import rig
    det = HotelSize(str(tmp_path))
    assert det.verdict(rig.jpeg(1)).size is None
    det.learn(30, rig.jpeg(1))
    det.learn(15, rig.jpeg(99))
    assert det.verdict(rig.jpeg(1)).size == 30
    assert det.verdict(rig.jpeg(99)).size == 15
    with pytest.raises(ValueError):
        det.learn(20, rig.jpeg(1))


def test_the_whole_setup_runs_on_the_simulator():
    cell, todo, runner, said, asked = make()
    stopped = runner.run(todo.sections())
    text = "\n".join(said)
    assert stopped is None, text
    L = cell.layout()
    assert L["setup"]["hotel"]["size"] == 30 and L["setup"]["hotel"]["method"] == "declared"
    assert L["setup"]["grips"]["grip_offset"] == 31.675
    assert L["squid"]["microscope"]["park_xy_mm"] == [0.0, 0.0]
    assert L["setup"]["microscope"]["found_at"]
    assert any("aruco plate to the microscope stage" in line for line in said)
    assert any("real plate on the rotation module" in line for line in said)
    assert text.index("move the aruco plate") < text.index("grips.toml") < text.index("remove the aruco plate")
    cell.frames.stop()


def test_a_second_run_recognises_the_hotel_without_asking():
    cell, todo, runner, said, asked = make()
    assert runner.run(todo.sections()[:3]) is None
    n = len([a for a in asked if "15 or 30" in a])
    cell2, todo2, runner2, said2, asked2 = make()
    cell2.ref_dir = cell.ref_dir
    assert runner2.run(todo2.sections()[:3]) is None
    assert n == 1
    cell.frames.stop()
    cell2.frames.stop()


def test_a_bad_hotel_answer_stops_the_run_and_names_the_section():
    cell, todo, runner, said, asked = make({"15 or 30": "20"})
    assert runner.run(todo.sections()) == "hotel.toml"
    assert any("REFUSED" in line and "neither 15 nor 30" in line for line in said)
    cell.frames.stop()


def test_a_calibration_above_the_limit_is_refused_and_nothing_is_uploaded():
    cell, todo, runner, said, asked = make()
    sections = todo.sections()
    todo.found["camera"] = None
    from gui import cameras
    real = cameras.solve
    cameras.solve = lambda cell, camera=None: {"auto_teach_camera": {"px": 0.9, "mm": 2.0, "pictures": 7, "ok": False}}
    try:
        assert runner.run(sections[:2]) == "cameras.toml"
    finally:
        cameras.solve = real
    assert any("above" in line for line in said)
    assert "uploaded_at" not in cell.layout()["vision"]["calibration"].get(cell.layout()["vision"]["down_camera"], {})
    cell.frames.stop()


def test_a_hotel_with_no_grip_offset_is_refused_with_what_to_measure():
    cell, todo, runner, said, asked = make({"15 or 30": "15"})
    assert runner.run(todo.sections()[:5]) == "grips.toml"
    assert any("grip_back_mm" in line for line in said)
    cell.frames.stop()


def test_the_microscope_without_stage_limits_is_refused_before_the_arm_moves():
    cell, todo, runner, said, asked = make()
    L = cell.layout()
    L["squid"]["microscope"]["stage_limits_mm"] = None
    cell.save(L)
    assert runner.run(todo.sections(), "microscope.toml") == "microscope.toml"
    assert any("stage_limits_mm" in line for line in said)
    cell.frames.stop()


def test_a_microscope_that_moved_beyond_the_taught_route_is_refused_and_nothing_is_written():
    cell, todo, runner, said, asked = make()
    before = cell.layout()["fixtures"]["microscope"]["pose"]
    world_markers(cell, stage_shift=(30.0, 0.0))
    assert runner.run(todo.sections()) == "microscope.toml"
    assert any("beyond the taught route" in line for line in said)
    assert cell.layout()["fixtures"]["microscope"]["pose"] == before
    cell.frames.stop()


def test_a_seat_a_little_off_is_written_with_the_old_one_kept():
    cell, todo, runner, said, asked = make()
    before = cell.layout()["fixtures"]["microscope"]["pose"]
    world_markers(cell, stage_shift=(1.5, 0.0))
    assert runner.run(todo.sections()) is None, "\n".join(said)
    f = cell.layout()["fixtures"]["microscope"]
    assert f["pose"][0] == pytest.approx(before[0] + 1.5, abs=0.2)
    assert any(k.startswith("previous_pose_") for k in f)
    cell.frames.stop()
