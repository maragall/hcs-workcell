"""The link to a running Squid (utils/devices/squid_link.py) against a stand-in for Squid's control server. No microscope."""
from __future__ import annotations

import json
import os
import socketserver
import sys
import threading

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from utils.devices.squid_link import SquidError, SquidLink  # noqa: E402


class Stage:
    pos, busy, log, slide, exec_on = [30.0, 40.0, 3.0], 0, [], None, True


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        req = json.loads(self.rfile.readline())
        cmd, p, S = req["command"], req["params"], Stage
        S.log.append(cmd)
        if cmd == "get_position":
            out = {"x_mm": S.pos[0], "y_mm": S.pos[1], "z_mm": S.pos[2]}
        elif cmd == "get_acquisition_status":
            S.busy = max(0, S.busy - 1)
            out = {"in_progress": S.busy > 0}
        elif cmd == "get_python_exec_status":
            out = {"enabled": S.exec_on}
        elif cmd == "python_exec" and not S.exec_on:
            out = {"error": "python_exec is disabled. Enable it in the GUI first for security.", "enabled": False}
        elif cmd == "python_exec" and "slide_position" in p["code"]:
            out = {"result": S.slide or "unknown"}
        elif cmd == "python_exec" and "btn_load_slide.clicked.emit()" in p["code"]:
            S.log.append("button")
            S.slide = "scanning" if S.slide == "loading" else "loading"
            S.pos = [0.5, 0.5, 0.1] if S.slide == "loading" else [20.0, 20.0, 3.0]
            out = {}
        elif cmd == "move_to":
            S.pos = [p["x_mm"], p["y_mm"], p["z_mm"]]
            out = {}
        else:
            self.wfile.write((json.dumps({"success": False, "error": "Unknown command"}) + "\n").encode())
            return
        self.wfile.write((json.dumps({"success": True, "result": out}) + "\n").encode())


@pytest.fixture
def link():
    srv = socketserver.ThreadingTCPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    Stage.pos, Stage.busy, Stage.log, Stage.slide, Stage.exec_on = [30.0, 40.0, 3.0], 0, [], None, True
    yield SquidLink("microscope", {"port": srv.server_address[1]})
    srv.shutdown()
    srv.server_close()


def test_wait_returns_when_the_acquisition_has_finished(link):
    Stage.busy = 3
    link.wait(poll_s=0.01)
    assert Stage.log.count("get_acquisition_status") == 3


def test_home_then_back_returns_the_stage_to_where_the_acquisition_ended(link):
    assert not link.parked()                    # never homed by this script: the arm stays out
    link.home()
    assert Stage.pos == [0.5, 0.5, 0.1] and link.parked()
    link.back()
    assert Stage.pos == [30.0, 40.0, 3.0] and not link.parked()


def test_the_stage_is_not_homed_under_a_running_acquisition(link):
    Stage.busy = 5
    with pytest.raises(SquidError, match="acquisition is running"):
        link.home()
    assert "button" not in Stage.log


def test_the_button_is_pressed_once_each_way(link):
    link.home()
    link.home()                                 # already at the loading position: not pressed again
    assert Stage.log.count("button") == 1
    link.back()
    assert Stage.log.count("button") == 2 and Stage.pos == [30.0, 40.0, 3.0]    # back to where it stood before the first home()


def test_python_exec_switched_off_says_which_tick(link, monkeypatch):
    monkeypatch.setattr(link, "_ask_for_python_exec", lambda: False)      # no Squid window here to ask in
    Stage.exec_on = False
    with pytest.raises(SquidError, match="Enable MCP Python Exec"):
        link.home()


def test_a_squid_that_is_not_running_says_how_to_start_the_server():
    with pytest.raises(SquidError, match="Enable MCP Control Server"):
        SquidLink("microscope", {"port": 1}).position()


def test_ping_says_squid_answers_and_whether_python_exec_is_on_without_moving_anything(link):
    assert link.ping() == {"acquiring": False, "python_exec": True}
    Stage.exec_on, Stage.busy = False, 3
    assert link.ping() == {"acquiring": True, "python_exec": False}
    assert "button" not in Stage.log and "move_to" not in Stage.log


def test_ping_of_a_squid_that_is_not_running_says_how_to_start_its_server():
    import socket
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        dead = s.getsockname()[1]
    with pytest.raises(SquidError, match="Enable MCP Control Server"):
        SquidLink("microscope", {"port": dead}).ping()
