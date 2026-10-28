"""The setup's real-cell path (no --sim) on fakes: discovery never writes the SDK port with the BRCAPI one,
never probes the SDK port while it is connected, a host that is not a vision box is not saved; STOP file; one-line
errors for a null layout value. Nothing touches the network or the hardware."""
from __future__ import annotations

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from bringup import discover, sim  # noqa: E402
from bringup.cell import Cell  # noqa: E402
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
    monkeypatch.setattr(discover, "vision_box", lambda L: {"host": "10.9.9.9", "port": 9000, "error": "10.9.9.9 answers but not as a vision box: x"})
    monkeypatch.setattr(discover, "squid", lambda *a: {"error": "no"})
    cell.vision_error = None
    cell.discover()
    assert cell.layout()["vision"]["host"] == before["vision"]["host"]
    monkeypatch.setattr(discover, "vision_box", lambda L: {"host": "10.9.9.9", "port": 9000, "version": "v", "cameras": 2})
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


def test_the_stop_file_stops_the_setup(monkeypatch):
    cell = real_cell(monkeypatch)
    os.makedirs(cell.runs_dir, exist_ok=True)
    assert not cell.stop_file_seen()                        # no file, no stop
    open(os.path.join(cell.runs_dir, "STOP"), "a").close()
    assert cell.stop_file_seen() and cell.stop_event.is_set()
    assert "STOP is present" in cell.notes[-1][1]


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


def test_squid_is_found_in_the_saved_folder_then_the_fallback_and_else_asks_for_a_folder(tmp_path, monkeypatch):
    from bringup import discover
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


def test_vision_box_found_on_another_port_by_its_get_version_reply(monkeypatch):
    """The saved port is dead; 9001 is open but silent, 8888 answers get_version: that one is the box, and is reported with its port."""
    from bringup import discover
    from devices import nova

    class Client:
        def __init__(self, host, port, timeout=5, **kw):
            self.host, self.port = host, port

        def version(self):
            if self.port != 8888:
                raise TimeoutError("timed out")
            return "service_name:VIBVision Service"

        def status(self):
            return type("S", (), {"cameras": 2})()

    monkeypatch.setattr(nova, "VisionClient", Client)
    monkeypatch.setattr(discover, "port_open", lambda host, port, timeout=0.4: host == "10.0.0.5" and port in (9001, 8888))
    got = discover.vision_box({"vision": {"host": "10.0.0.5", "port": 9000}})
    assert got["host"] == "10.0.0.5" and got["port"] == 8888 and got["cameras"] == 2


def test_vision_box_nothing_answering_says_what_each_port_did(monkeypatch):
    from bringup import discover
    from devices import nova

    class Client:
        def __init__(self, *a, **kw):
            pass

        def version(self):
            raise TimeoutError("timed out")

    monkeypatch.setattr(nova, "VisionClient", Client)
    monkeypatch.setattr(discover, "port_open", lambda host, port, timeout=0.4: host == "10.0.0.5" and port == 9001)
    monkeypatch.setattr(discover, "local_subnets", lambda: [])
    got = discover.vision_box({"vision": {"host": "10.0.0.5", "port": 9000}})
    assert "error" in got and "10.0.0.5:9000 timed out" in got["error"] and "9001 open" in got["error"]
