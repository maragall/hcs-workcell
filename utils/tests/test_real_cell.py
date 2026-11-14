"""The setup's real-cell path (no --sim) on fakes: discovery never writes the SDK port with the BRCAPI one,
never probes the SDK port while it is connected; the arm is tried again and its failure says what to do; every Squid is asked;
STOP file; one-line errors for a null layout value. Nothing touches the network or the hardware."""
from __future__ import annotations

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from setup import discover, sim  # noqa: E402
from setup.cell import Cell  # noqa: E402
from run import positions  # noqa: E402
from run.preflight import preflight  # noqa: E402


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


class Flaky:
    """An arm whose connect fails `fails` times, then answers like the simulator's."""

    def __init__(self, robot, fails, error="cannot connect to controller 192.168.1.220:52525 (one client only)"):
        self._robot, self.fails, self.error, self.tries = robot, fails, error, 0
        self.ip, self.port = "192.168.1.220", 52525

    def connect(self):
        self.tries += 1
        if self.tries <= self.fails:
            raise RuntimeError(self.error)

    def __getattr__(self, name):
        return getattr(self._robot, name)


def test_the_arm_is_tried_again_when_the_controller_is_not_ready_yet(monkeypatch):
    from setup import cell as cell_mod
    monkeypatch.setattr(cell_mod, "CONNECT_PAUSE_S", 0.0)
    cell = real_cell(monkeypatch)
    cell.robot = Flaky(cell.robot, fails=2)
    cell.connect_arm_again()
    assert cell.arm_error is None and cell.robot.tries == 3


def test_a_vendor_package_that_does_not_load_is_not_tried_again_and_the_fix_names_the_installer(monkeypatch):
    from setup import cell as cell_mod
    monkeypatch.setattr(cell_mod, "CONNECT_PAUSE_S", 0.0)
    cell = real_cell(monkeypatch)
    cell.robot = Flaky(cell.robot, fails=99, error="vendor SDK not importable (pythonnet + bioyondsdk wheel + .NET 8): No module named 'BioyondSDK'")
    cell.connect_arm_again()
    assert cell.robot.tries == 1 and "install.bat" in cell.arm_fix({})


def test_an_arm_that_answers_but_refuses_this_program_says_to_close_the_vendor_software(monkeypatch):
    from setup import cell as cell_mod
    monkeypatch.setattr(cell_mod, "CONNECT_PAUSE_S", 0.0)
    cell = real_cell(monkeypatch)
    cell.robot = Flaky(cell.robot, fails=99)
    monkeypatch.setattr(discover, "arm", lambda L, connected=False: {"ip": "192.168.1.220", "port": 52525})
    monkeypatch.setattr(discover, "squid", lambda L: {})
    cell.discover()
    arm = cell.devices[0]
    assert not arm["ok"] and "BioyondWorks" in arm["fix"] and "one program at a time" in arm["fix"]


def test_an_arm_found_at_another_address_is_saved_and_connected(monkeypatch):
    from setup import cell as cell_mod
    monkeypatch.setattr(cell_mod, "CONNECT_PAUSE_S", 0.0)
    cell = real_cell(monkeypatch)
    cell.robot = Flaky(cell.robot, fails=0)
    cell.arm_error = "cannot connect"
    monkeypatch.setattr(discover, "arm", lambda L, connected=False: {"ip": "10.1.2.3", "port": 52525})
    monkeypatch.setattr(discover, "squid", lambda L: {})
    cell.discover()
    assert cell.layout()["robot"]["ip"] == "10.1.2.3" and cell.robot.ip == "10.1.2.3"
    assert cell.arm_error is None and cell.devices[0]["ok"]


def test_no_arm_on_the_network_says_which_address_range_this_pc_needs(monkeypatch):
    monkeypatch.setattr(discover, "scan", lambda port, first=None: None)
    monkeypatch.setattr(discover, "local_subnets", lambda: [])
    got = discover.arm({"robot": {"ip": "192.168.1.220", "port": 52525}})
    assert "ip" not in got and "192.168.1.x" in got["fix"] and "port 1" in got["fix"]


def test_every_squid_of_the_positions_file_is_asked_and_a_silent_one_says_what_to_do():
    import socket
    with socket.socket() as s:                              # a port nothing listens on
        s.bind(("127.0.0.1", 0))
        dead = s.getsockname()[1]
    got = discover.squid({"squid": {"microscope": {"port": dead}}})
    assert got["microscope"]["at"] == f"127.0.0.1:{dead}"
    assert "Enable MCP Control Server" in got["microscope"]["error"]
    assert discover.squid({}) == {} and discover.squid({"squid": {}}) == {}


def test_a_squid_row_is_listed_for_each_microscope(monkeypatch):
    cell = real_cell(monkeypatch)
    monkeypatch.setattr(discover, "arm", lambda L, connected=False: {"ip": "192.168.1.220", "port": 52525})
    monkeypatch.setattr(discover, "squid", lambda L: {"microscope": {"at": "127.0.0.1:5050", "acquiring": False, "python_exec": False},
                                                     "microscope_2": {"at": "127.0.0.1:5051", "error": "does not answer"}})
    cell.discover()
    rows = {d["name"]: d for d in cell.devices}
    assert rows["Squid microscope"]["ok"] and "Python Exec off" in rows["Squid microscope"]["state"]
    assert not rows["Squid microscope_2"]["ok"] and "Enable MCP Control Server" in rows["Squid microscope_2"]["fix"]
    assert not any("ision" in d["name"] for d in cell.devices)


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
    assert "hotel lift not set: fixtures.hotel.lift_mm" in preflight(L)


def test_the_stop_file_stops_the_setup(monkeypatch):
    cell = real_cell(monkeypatch)
    os.makedirs(cell.runs_dir, exist_ok=True)
    assert not cell.stop_file_seen()                        # no file, no stop
    open(os.path.join(cell.runs_dir, "STOP"), "a").close()
    assert cell.stop_file_seen() and cell.stop_event.is_set()
    assert "STOP is present" in cell.notes[-1][1]


def test_cli_turns_a_device_error_into_one_line(monkeypatch):
    import cli
    from utils.devices.arm import RobotFault

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
