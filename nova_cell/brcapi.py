"""
Client for the controller's documented protocol, BRCAPI 1.0 (TCP port 56788, one JSON per line).

The controller has no motion commands on this interface. Motion is a Lua program on the controller;
this client starts it, hands it jobs through system variables, and waits for the completion push.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

SYS_STATE = {0: "init", 1: "idle", 2: "running", 3: "error", 4: "pause", 5: "enabled"}
PROG_STATE = {0: "stopped", 1: "running", 2: "paused", 3: "error"}


class BrcapiError(Exception):
    def __init__(self, code, message, cmd=""):
        super().__init__(f"{cmd}: {code} {message}")
        self.code, self.message, self.cmd = code, message, cmd


@dataclass
class ControllerState:
    sys_state: str
    ctrl_mode: str
    enabled: bool
    prog_state: str
    drag: bool
    errors: List[dict]
    lua_ready: bool


class Brcapi:
    KEEPALIVE_S = 60.0

    def __init__(self, host: str, port: int = 56788, timeout: float = 10.0, log=None):
        self.host, self.port, self.timeout = host, port, timeout
        self._log = log or (lambda *a: None)
        self._sock: Optional[socket.socket] = None
        self._buf = bytearray()
        self._id = 100
        self._lock = threading.Lock()
        self._pushes: Dict[int, list] = {}
        self._last_tx = 0.0

    # --------------------------------------------------------------- transport
    def connect(self):
        self.close()
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._log("ctrl", "connect", f"{self.host}:{self.port}")

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _readline(self, timeout: float) -> Optional[dict]:
        self._sock.settimeout(timeout)
        while True:
            i = self._buf.find(b"\n")
            if i >= 0:
                line = bytes(self._buf[:i])
                del self._buf[: i + 1]
                if line.strip():
                    return json.loads(line)
                continue
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                return None
            if not chunk:
                raise ConnectionError("controller closed the connection")
            self._buf += chunk

    def _dispatch_push(self, msg: dict):
        self._pushes.setdefault(int(msg.get("id", 0)), []).append(msg)

    def call(self, cmd: str, params=None):
        with self._lock:
            if self._sock is None:
                self.connect()
            self._id += 1
            rid = self._id
            msg = {"BRCAPI": "1.0", "cmd": cmd, "id": rid}
            if params is not None:
                msg["params"] = params
            self._sock.sendall((json.dumps(msg) + "\n").encode())
            self._last_tx = time.time()
            deadline = time.time() + self.timeout
            while time.time() < deadline:
                m = self._readline(max(0.1, deadline - time.time()))
                if m is None:
                    continue
                if m.get("id") == rid and ("ret" in m or "error" in m):
                    if "error" in m:
                        e = m["error"]
                        raise BrcapiError(e.get("code"), e.get("message"), cmd)
                    return m.get("ret")
                self._dispatch_push(m)
            raise BrcapiError("timeout", f"no ack within {self.timeout}s", cmd)

    def wait_push(self, rid: int, timeout: float, predicate=lambda m: True) -> Optional[dict]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                for m in list(self._pushes.get(rid, [])):
                    if predicate(m):
                        self._pushes[rid].remove(m)
                        return m
                m = self._readline(0.5)
                if m is not None:
                    self._dispatch_push(m)
            if time.time() - self._last_tx > self.KEEPALIVE_S:
                self.call("getSysState")
        return None

    # ------------------------------------------------------------------ reads
    def state(self) -> ControllerState:
        errs = [e for e in (self.call("getRobotErr") or []) if e.get("errCode") is not None]
        return ControllerState(
            SYS_STATE.get(self.call("getSysState")["value"], "?"),
            {0: "manual", 1: "auto"}.get(self.call("getCtrlMode")["value"], "?"),
            self.call("getEnableStatus")["value"] == 1,
            PROG_STATE.get(self.call("getProgState")["value"], "?"),
            self.call("isDragMode")["dragMode"] == 1,
            errs,
            self.call("getLuaAppStatus")["value"] == 1,
        )

    def pose_mm_rad(self) -> List[float]:
        p = self.call("getFrameWithU0T0")
        return [p["X"] * 1000, p["Y"] * 1000, p["Z"] * 1000, p["Rx"], p["Ry"], p["Rz"]]

    def joints(self):
        j = self.call("getJointInfo")
        return [x["value"] for x in j["joints"]], [x["value"] for x in j["extjoints"]]

    def sysvars(self, indices=range(0, 10)) -> Dict[int, int]:
        return {x["index"]: x["value"] for x in self.call("getSysVarI", [{"index": i} for i in indices])}

    def version(self) -> str:
        return self.call("getSysVersion")["version"]

    # ----------------------------------------------------------------- control
    def _action(self, cmd: str, timeout: float = 15.0) -> str:
        with self._lock:
            rid = self._id + 1
        self.call(cmd)
        m = self.wait_push(rid, timeout, lambda m: "reply" in m and isinstance(m["reply"], str))
        reply = m["reply"] if m else "NoReply"
        self._log("ctrl", cmd, reply)
        if reply not in ("CmdDone",):
            raise BrcapiError(reply, f"{cmd} did not complete", cmd)
        return reply

    def switch_auto(self):
        return self._action("switchToAuto")

    def switch_manual(self):
        return self._action("switchToManual")

    def enable(self):
        return self._action("remoteEnable")

    def disable(self):
        return self._action("remoteDisable")

    def start_program(self):
        return self._action("remoteStart")

    def pause_program(self):
        return self._action("remotePause")

    def stop_program(self):
        return self._action("remoteStop")

    def clear_error(self):
        return self._action("remoteClearError")

    def drag(self, on: bool):
        return self._action("enDragMode" if on else "disDragMode")

    def set_params_f(self, values: Dict[int, float]):
        return self.call("setSysParaF", [{"index": i, "value": round(float(v), 6)} for i, v in values.items()])

    def set_params_i(self, values: Dict[int, int]):
        return self.call("setSysParaI", [{"index": i, "value": int(v)} for i, v in values.items()])

    # --------------------------------------------------------------------- jobs
    def run_job(self, sysvars: Dict[int, int], timeout_s: float) -> int:
        """Hand a job to the Lua dispatcher and wait for SendSysOutVarI(1, code). Returns the code."""
        busy = self.sysvars(range(0, 8))
        if busy.get(0, 0) != 0:
            raise BrcapiError("busy", f"a job is still active: {busy}", "run_job")
        with self._lock:
            rid = self._id + 1
        self.call("setSysVarI", [{"index": i, "value": int(v)} for i, v in sorted(sysvars.items())])
        self._log("ctrl", "job", str(sysvars))
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            m = self.wait_push(rid, 2.0, lambda m: isinstance(m.get("reply"), dict) and "value" in m["reply"])
            if m:
                code = int(m["reply"]["value"])
                self._log("ctrl", "job done", f"code {code} in {time.time() - t0:.1f}s")
                return code
            st = self.call("getProgState")["value"]
            if st == 3:
                raise BrcapiError("prog_error", f"Lua program in error: {self.call('getRobotErr')}", "run_job")
            if self.sysvars(range(0, 1)).get(0, 0) == 0 and time.time() - t0 > 5:
                self._log("ctrl", "job done", "sysvar0 cleared without push; treating as done")
                return 1
        raise BrcapiError("timeout", f"job not finished in {timeout_s}s", "run_job")
