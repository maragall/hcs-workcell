"""The setup (python cli.py setup) on the fakes: its sections in order, and the preset section writing the arm's program and
handing it to the arm. The camera section is not run here."""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from utils.devices.fake_squid import FakeSquid  # noqa: E402
from run import positions  # noqa: E402
from setup import sim  # noqa: E402
from setup.cell import Cell  # noqa: E402
from setup.runner import Runner  # noqa: E402
from setup.sections import Bringup  # noqa: E402


def bring():
    cell = Cell(*sim.build(), sim=True)
    return cell, Bringup(cell, FakeSquid("microscope", 0.0), lambda p: "")


def test_the_setup_is_connect_then_the_program_then_the_cameras():
    cell, b = bring()
    try:
        assert [s.name for s in b.sections()] == ["connect", "preset", "cameras"]
        assert not any(st.moves for st in b.preset().steps)                 # the upload moves nothing
    finally:
        cell.frames.stop()


def test_the_preset_section_writes_the_program_and_the_arm_holds_what_was_sent():
    cell, b = bring()
    said = []
    try:
        cell.save(positions.load())                                         # the stored routes belong to the real positions file, not the simulator's
        stopped = Runner(say=said.append, ask=lambda p: "").run([b.preset()])
        assert stopped is None, said
        folder = os.path.join(cell.runs_dir, "lua_project")
        assert sorted(os.listdir(folder)) == ["data.lua", "mod_Polar.lua", "mod_Socket.lua", "mod_TorqueProbe.lua"]
        assert cell.robot.program == os.path.abspath(folder)
        assert "POLAR=" in open(os.path.join(folder, "data.lua"), encoding="utf-8").read()
    finally:
        cell.frames.stop()
