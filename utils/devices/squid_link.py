"""A running Squid, through the control server its own GUI offers (Squid: Settings > Enable MCP Control Server; TCP port 5050,
one JSON line in, one JSON line out; control/microscope_control_server.py in the Squid repo). Nothing of Squid is imported here.

What the plate change needs from the microscope:
    wait()      until the acquisition that is running has finished
    home()      remember where the stage is, then run Squid's own loading procedure: the loading position (Squid's config,
                loading_x_mm, loading_y_mm) is where the arm reaches the plate holder
    parked()    the stage still stands where home() left it
    back()      Squid's own return to the scanning position, then the stage back to where it was before home()
    acquire(yaml_path)   start an acquisition saved by the GUI (acquisition.yaml)

config (layout.json squid.<name>): host (default 127.0.0.1), port (default 5050), park_tolerance_mm (default 0.5).
"""
from __future__ import annotations

import json
import socket
import time
from typing import Optional


class SquidError(RuntimeError):
    """Squid did not answer, or refused the command."""


class SquidLink:
    def __init__(self, name: str = "microscope", config: Optional[dict] = None, log=None):
        config = config or {}
        self.name = name
        self.host, self.port = config.get("host", "127.0.0.1"), int(config.get("port", 5050))
        self.tolerance = float(config.get("park_tolerance_mm") or 0.5)
        self._log = log or (lambda *a: None)
        self._before = None            # x, y, z before the last home()
        self._park = None              # x, y where home() left the stage

    def _call(self, command: str, timeout: float = 30.0, **params):
        try:
            with socket.create_connection((self.host, self.port), timeout=5.0) as s:
                s.settimeout(timeout)
                s.sendall((json.dumps({"command": command, "params": params}) + "\n").encode("utf-8"))
                buf = b""
                while b"\n" not in buf:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
        except OSError as e:
            raise SquidError(f"{self.name}: Squid does not answer on {self.host}:{self.port} ({e}). "
                             "Start Squid and tick Settings > Enable MCP Control Server") from e
        try:
            reply = json.loads(buf.decode("utf-8").strip())
        except ValueError as e:
            raise SquidError(f"{self.name}: Squid's answer to {command} is not readable: {buf[:80]!r}") from e
        if not reply.get("success"):
            raise SquidError(f"{self.name}: Squid refused {command}: {reply.get('error')}")
        return reply.get("result")

    def position(self) -> tuple:
        p = self._call("get_position")
        return (float(p["x_mm"]), float(p["y_mm"]), float(p["z_mm"]))

    def done(self) -> bool:
        return not self._call("get_acquisition_status")["in_progress"]

    def wait(self, poll_s: float = 2.0, timeout_s: Optional[float] = None) -> None:
        t0 = time.time()
        while not self.done():
            if timeout_s is not None and time.time() - t0 > timeout_s:
                raise SquidError(f"{self.name}: the acquisition is still running after {timeout_s:.0f} s")
            time.sleep(poll_s)
        self._log("squid", self.name, "acquisition finished")

    def home(self, timeout_s: float = 180.0) -> None:
        if not self.done():
            raise SquidError(f"{self.name}: an acquisition is running: wait for it before moving the stage")
        self._before = self.position()
        self._call("move_to_loading_position", timeout=timeout_s)
        x, y, _ = self.position()
        self._park = (x, y)
        self._log("squid", self.name, f"at the loading position {self._park}, was at {self._before}")

    def parked(self) -> bool:
        if self._park is None:
            return False
        x, y, _ = self.position()
        return max(abs(x - self._park[0]), abs(y - self._park[1])) <= self.tolerance

    def back(self, timeout_s: float = 180.0) -> None:
        if self._before is None:
            raise SquidError(f"{self.name}: nowhere to go back to: home() has not been called")
        x, y, z = self._before
        self._call("move_to_scanning_position", timeout=timeout_s)
        self._call("move_to", timeout=timeout_s, x_mm=x, y_mm=y, z_mm=z, blocking=True)
        self._log("squid", self.name, f"back at {self._before}")

    def acquire(self, yaml_path: str) -> dict:
        got = self._call("run_acquisition_from_yaml", timeout=120.0, yaml_path=yaml_path)
        self._log("squid", self.name, f"acquisition started: {got.get('experiment_id')}")
        return got

    def abort(self) -> None:
        self._call("abort_acquisition")
