"""The setup program's real-cell path (no --sim) on fakes: discovery never writes the SDK port with the BRCAPI one,
never probes the SDK port while it is connected, a host that is not a vision box is not saved; STOP file; one-line
errors for a null layout value. Nothing touches the network or the hardware."""
from __future__ import annotations

import json
import os
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from gui import discover, sim  # noqa: E402
from gui.cell import Cell, plain_error  # noqa: E402
from procedures import positions  # noqa: E402
from procedures.preflight import preflight  # noqa: E402


def real_cell(monkeypatch, tmp=None):
    """A Cell with sim=False on the simulator's fakes."""
    cell = Cell(*sim.build(), sim=False)
    return cell


def test_arm_keeps_the_sdk_port_when_only_brcapi_answers(monkeypatch):
    seen = []
    monkeypatch.setattr(discover, "scan", lambda port, first=None: (seen.append(port), "10.1.2.3" if port == 56788 else None)[1])
    got = discover.arm({"robot": {"ip": "192.168.1.220", "port": 52525}})
    assert got == {"ip": "10.1.2.3", "port": 52525}, got
    assert seen == [52525, 56788]


def test_arm_does_not_probe_the_sdk_port_while_connected(monkeypatch):
    seen = []
    monkeypatch.setattr(discover, "scan", lambda port, first=None: (seen.append(port), first)[1])
    got = discover.arm({"robot": {"ip": "192.168.1.220", "port": 52525}}, connected=True)
    assert seen == [56788] and got == {"ip": "192.168.1.220", "port": 52525}


def test_a_host_that_is_not_a_vision_box_is_not_saved(monkeypatch):
    cell = real_cell(monkeypatch)
    before = cell.layout()
    monkeypatch.setattr(discover, "arm", lambda L, connected=False: {"error": "none"})
    monkeypatch.setattr(discover, "vision_box", lambda L: {"host": "10.9.9.9", "error": "10.9.9.9 answers but not as a vision box: x"})
    monkeypatch.setattr(discover, "squid", lambda *a: {"error": "no"})
    cell.vision_error = None
    cell.discover()
    assert cell.layout()["vision"]["host"] == before["vision"]["host"]
    monkeypatch.setattr(discover, "vision_box", lambda L: {"host": "10.9.9.9", "version": "v", "cameras": 2})
    cell.discover()
    assert cell.layout()["vision"]["host"] == "10.9.9.9"


def test_ipconfig_output_in_any_code_page_is_read(monkeypatch):
    seen = {}

    class R:
        stdout = "IPv4 Address. . . : 192.168.7.5\n"

    def run(cmd, **kw):
        seen.update(kw)
        return R()

    monkeypatch.setattr(discover.subprocess, "run", run)
    assert [str(n) for n in discover.local_subnets()] == ["192.168.7.0/24"]
    assert seen.get("errors") == "replace"


def test_preflight_says_a_null_hotel_lift_in_words():
    L = positions.load()
    L["fixtures"]["hotel"]["lift_mm"] = None
    assert "hotel lift not set: fixtures.hotel.lift_mm" in preflight(L, os.path.join(HERE, "no_refs"))


def test_a_missing_value_is_one_plain_line():
    assert "layout.json" in plain_error(KeyError("pose"))
    assert "layout.json" in plain_error(TypeError("'NoneType' object is not subscriptable"))
    assert plain_error(RuntimeError("boom")) == "boom"


def test_a_job_that_hits_a_null_says_so_in_one_line(monkeypatch):
    cell = real_cell(monkeypatch)

    def job():
        cell.layout()["fixtures"]["microscope_2"]["pose"][0]

    assert cell.start("probe", job)
    for _ in range(50):
        if not cell.busy:
            break
        time.sleep(0.05)
    text = cell.notes[-1][1]
    assert text.startswith("probe: a value is missing in layout.json") and "\n" not in text


def test_the_stop_file_stops_a_running_job(monkeypatch):
    cell = real_cell(monkeypatch)
    os.makedirs(cell.runs_dir, exist_ok=True)
    gate = __import__("threading").Event()
    assert cell.start("job", lambda: gate.wait(5))
    assert not cell.stop_file_seen()                        # no file, no stop
    open(os.path.join(cell.runs_dir, "STOP"), "a").close()
    assert cell.stop_file_seen() and cell.stop_event.is_set()
    assert "STOP is present" in cell.notes[-1][1]
    gate.set()


def test_cli_turns_a_device_error_into_one_line(monkeypatch):
    import cli
    from devices.arm import RobotFault

    def boom(args):
        raise RobotFault("cannot connect to controller 192.168.1.220:52525 (one client only)")

    monkeypatch.setattr(cli, "cmd_positions", boom)
    monkeypatch.setattr(sys, "argv", ["cli.py", "positions"])
    with pytest.raises(SystemExit) as e:
        cli.main()
    assert str(e.value) == "RobotFault: cannot connect to controller 192.168.1.220:52525 (one client only)"


def test_cli_turns_a_null_into_one_line(monkeypatch):
    import cli

    def boom(args):
        nothing = None
        return nothing["x"]

    monkeypatch.setattr(cli, "cmd_positions", boom)
    monkeypatch.setattr(sys, "argv", ["cli.py", "positions"])
    with pytest.raises(SystemExit) as e:
        cli.main()
    assert "layout.json" in str(e.value) and "\n" not in str(e.value)


def test_the_page_comes_up_before_the_arm_connects(monkeypatch):
    """gui.app.main serves first and connects in a job, so an arm that is off cannot hold the start."""
    from gui import app
    order = []
    cell = real_cell(monkeypatch)
    monkeypatch.setattr(app, "build", lambda is_sim: cell)
    monkeypatch.setattr(app.server, "serve", lambda c, port: (order.append("serve"), type("S", (), {"server_address": ("127.0.0.1", 1), "shutdown": lambda s: None})())[1])
    monkeypatch.setattr(cell, "start_up", lambda: order.append("connect"))
    monkeypatch.setattr(app.time, "sleep", lambda s: (_ for _ in ()).throw(KeyboardInterrupt()))
    assert app.main(["--no-browser"]) == 0
    for _ in range(50):
        if "connect" in order:
            break
        time.sleep(0.05)
    assert order == ["serve", "connect"]


def test_squid_is_found_in_the_saved_folder_then_the_fallback_and_else_asks_for_a_folder(tmp_path, monkeypatch):
    from gui import discover
    ctl = tmp_path / "software" / "control"
    ctl.mkdir(parents=True)
    (ctl / "microscope.py").write_text("")
    monkeypatch.setattr(discover, "SQUID_FALLBACKS", ())
    monkeypatch.setattr(discover.importlib.util, "find_spec", lambda name: None)
    assert discover.squid({"squid_software": {"folder": str(tmp_path)}})["folder"] == str(tmp_path / "software")
    out = discover.squid({})
    assert out["error"] == "can't find the Squid software folder" and out["browse"]
    monkeypatch.setattr(discover, "SQUID_FALLBACKS", (str(tmp_path),))
    assert "module" in discover.squid({})
