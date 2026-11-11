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
    pos, busy, log = [30.0, 40.0, 3.0], 0, []


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
        elif cmd == "home":
            S.pos = [0.1, 0.2, 0.0]
            out = {"homed": True}
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
    Stage.pos, Stage.busy, Stage.log = [30.0, 40.0, 3.0], 0, []
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
    assert Stage.pos == [0.1, 0.2, 0.0] and link.parked()
    link.back()
    assert Stage.pos == [30.0, 40.0, 3.0] and not link.parked()


def test_the_stage_is_not_homed_under_a_running_acquisition(link):
    Stage.busy = 5
    with pytest.raises(SquidError, match="acquisition is running"):
        link.home()
    assert "home" not in Stage.log


def test_a_squid_that_is_not_running_says_how_to_start_the_server():
    with pytest.raises(SquidError, match="Enable MCP Control Server"):
        SquidLink("microscope", {"port": 1}).position()
