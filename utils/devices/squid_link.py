"""A running Squid, through the control server its own GUI offers (Squid: Settings > Enable MCP Control Server; TCP port 5050,
one JSON line in, one JSON line out; control/microscope_control_server.py in the Squid repo). Nothing of Squid is imported here and
nothing is added to Squid. The loading position is reached by Squid's own button ("Move To Loading Position"), pressed through the
server's python_exec command (Squid: Settings > Enable MCP Python Exec): the server has no command of its own for it, and the
position lies outside the stage's travel limits, which only Squid's own procedure opens.

What the plate change needs from the microscope:
    wait()      until the acquisition that is running has finished
    home()      remember where the stage is, then press Squid's loading button: the loading position (Squid's config,
                loading_x_mm, loading_y_mm) is where the arm reaches the plate holder
    parked()    the stage still stands where home() left it
    back()      press the button again (Squid's return to the scanning position), then the stage back to where it was before home()
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

    def _py(self, code: str):
        got = self._call("python_exec", code=code)
        if isinstance(got, dict) and got.get("error"):
            raise SquidError(f"{self.name}: {got['error']} Tick Settings > Enable MCP Python Exec in Squid")
        return (got or {}).get("result")

    def _slide(self, want: str, timeout_s: float) -> None:
        """Squid's loading button, pressed until the stage is where `want` says ("loading" or "scanning"). The press is the button's
        own signal, so Squid runs its procedure in its own thread and its button shows the new state."""
        where = "result = gui.stageUtils.slide_position or 'unknown'"
        if (self._py(where) == "loading") != (want == "loading"):
            self._py("gui.stageUtils.btn_load_slide.clicked.emit()")
            t0 = time.time()
            while self._py(where) != want:
                if time.time() - t0 > timeout_s:
                    raise SquidError(f"{self.name}: the stage did not reach its {want} position in {timeout_s:.0f} s")
                time.sleep(0.5)

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
        if self._py("result = gui.stageUtils.slide_position or 'unknown'") != "loading":      # already there: what was remembered stands
            self._before = self.position()
        self._slide("loading", timeout_s)
        x, y, _ = self.position()
        self._park = (x, y)
        self._log("squid", self.name, f"at the loading position {self._park}, was at {self._before}")

    def parked(self) -> bool:
        if self._park is None:
            return False
        x, y, _ = self.position()
        return max(abs(x - self._park[0]), abs(y - self._park[1])) <= self.tolerance

    def back(self, timeout_s: float = 180.0) -> None:
        self._slide("scanning", timeout_s)
        if self._before is not None:                # nothing remembered (the stage stood at the loading position from the start): Squid's scanning position
            x, y, z = self._before
            self._call("move_to", timeout=timeout_s, x_mm=x, y_mm=y, z_mm=z, blocking=True)
        self._log("squid", self.name, f"back at {self._before or 'the scanning position'}")

    def acquire(self, yaml_path: str) -> dict:
        got = self._call("run_acquisition_from_yaml", timeout=120.0, yaml_path=yaml_path)
        self._log("squid", self.name, f"acquisition started: {got.get('experiment_id')}")
        return got

    def abort(self) -> None:
        self._call("abort_acquisition")
