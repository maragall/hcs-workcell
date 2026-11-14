"""The setup (python cli.py setup) on the fakes: its sections in order, and the preset section writing the arm's program and
handing it to the arm, and the connect section asking the arm and Squid again in place."""
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


def test_the_setup_is_connect_then_the_places_then_the_program():
    cell, b = bring()
    assert [s.name for s in b.sections()] == ["connect", "places", "preset"]
    assert not any(st.moves for st in b.connect().steps)                    # connecting and enabling move nothing
    assert not any(st.moves for st in b.preset().steps)                     # the upload moves nothing


def test_the_preset_section_writes_the_program_and_the_arm_holds_what_was_sent():
    cell, b = bring()
    said = []
    if True:
        cell.save(positions.load())                                         # the stored routes belong to the real positions file, not the simulator's
        stopped = Runner(say=said.append, ask=lambda p: "").run([b.preset()])
        assert stopped is None, said
        folder = os.path.join(cell.runs_dir, "lua_project")
        assert sorted(os.listdir(folder)) == ["data.lua", "mod_Polar.lua", "mod_Socket.lua", "mod_TorqueProbe.lua"]
        assert cell.robot.program == os.path.abspath(folder)
        assert "POLAR=" in open(os.path.join(folder, "data.lua"), encoding="utf-8").read()


# ---------------------------------------------------------------- the connect section: the arm and Squid, asked again in place
def real(monkeypatch, squid=None, arm_ok=True):
    """A Cell with sim=False on the fakes. `arm_ok` and `squid` are lists the test changes while the section runs."""
    from setup import discover
    cell = Cell(*sim.build(), sim=False)
    cell.connect()
    state = {"arm": arm_ok, "squid": squid if squid is not None else {}}
    monkeypatch.setattr(discover, "arm", lambda L, connected=False: {"ip": "192.168.1.220", "port": 52525} if state["arm"]
                        else {"error": "no controller answers on ports 52525 or 56788", "fix": "cable to port 1"})
    monkeypatch.setattr(discover, "squid", lambda L: state["squid"])
    return cell, state


def run_connect(cell, ask, **kw):
    said = []
    b = Bringup(cell, FakeSquid("microscope", 0.0), ask, **kw)
    stopped = Runner(say=said.append, ask=ask).run([b.connect()])
    return stopped, "\n".join(said)


def test_connect_passes_on_the_simulator_without_asking_anything():
    cell, b = bring()
    cell.connect()
    asked = []
    stopped = Runner(say=lambda s: None, ask=lambda p: asked.append(p) or "").run([b.connect()])
    assert stopped is None and asked == []


def test_connect_finds_the_arm_enables_it_and_hears_squid(monkeypatch):
    cell, _ = real(monkeypatch, squid={"microscope": {"at": "127.0.0.1:5050", "acquiring": False, "python_exec": True}})
    cell.robot.faults.drag_on = False
    stopped, said = run_connect(cell, lambda p: "")
    assert stopped is None, said
    assert "arm = 192.168.1.220" in said and "Squid microscope = 127.0.0.1:5050 answers" in said
    assert cell.robot.state().enabled


def test_connect_asks_for_the_arm_again_in_place_and_goes_on_once_it_answers(monkeypatch):
    import time
    cell, state = real(monkeypatch, arm_ok=False)
    prompts = []

    def ask(prompt):
        prompts.append(prompt)
        time.sleep(0.06)                      # a person took a moment
        state["arm"] = True                   # ... and plugged the cable in
        return ""
    stopped, said = run_connect(cell, ask)
    assert stopped is None, said
    assert len(prompts) == 1 and "cable to port 1" in prompts[0] and "try again" in prompts[0]


def test_connect_stops_with_the_fix_when_nobody_is_at_the_keyboard(monkeypatch):
    cell, _ = real(monkeypatch, arm_ok=False)
    stopped, said = run_connect(cell, lambda p: "")          # --yes: every prompt answered at once
    assert stopped == "connect" and "REFUSED" in said and "cable to port 1" in said


def test_connect_stops_when_the_user_types_q(monkeypatch):
    import time
    cell, _ = real(monkeypatch, arm_ok=False)
    stopped, said = run_connect(cell, lambda p: (time.sleep(0.06), "q")[1])
    assert stopped == "connect" and "the arm is not connected" in said


def test_connect_gives_up_after_asking_three_times(monkeypatch):
    import time
    cell, _ = real(monkeypatch, arm_ok=False)
    prompts = []
    stopped, said = run_connect(cell, lambda p: (prompts.append(p), time.sleep(0.06), "")[2])
    assert stopped == "connect" and len(prompts) == 3


def test_connect_asks_for_squid_again_and_no_squid_skips_it(monkeypatch):
    import time
    silent = {"microscope": {"at": "127.0.0.1:5050", "error": "microscope: Squid does not answer on 127.0.0.1:5050"}}
    cell, state = real(monkeypatch, squid=silent)
    prompts = []

    def ask(prompt):
        prompts.append(prompt)
        time.sleep(0.06)
        state["squid"] = {"microscope": {"at": "127.0.0.1:5050", "acquiring": False, "python_exec": False}}      # the person started Squid
        return ""
    stopped, said = run_connect(cell, ask)
    assert stopped is None, said
    assert len(prompts) == 1 and "Enable MCP Control Server" in prompts[0] and "--no-squid" in prompts[0]
    cell, _ = real(monkeypatch, squid=silent)
    stopped, said = run_connect(cell, lambda p: "", check_squid=False)
    assert stopped is None and "not asked (--no-squid)" in said


def test_connect_with_no_microscope_in_the_positions_file_has_nothing_to_ask(monkeypatch):
    cell, _ = real(monkeypatch, squid={})
    stopped, said = run_connect(cell, lambda p: "")
    assert stopped is None and "nothing to ask" in said


def test_connect_refuses_an_arm_whose_controller_reports_an_error(monkeypatch):
    cell, _ = real(monkeypatch)
    cell.robot.faults.controller_errors = [{"errCode": 3873, "errMsg": "wrist collision"}]
    stopped, said = run_connect(cell, lambda p: "")
    assert stopped == "connect" and "3873" in said and "python cli.py recover" in said
