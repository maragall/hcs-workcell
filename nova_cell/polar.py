"""
Polar (rotating) stage driver.  How the unit is commanded is NOT yet known, so three drivers exist
behind one interface.  Pick one in layout.json -> "polar": {"driver": ...}.

  do      : robot digital output(s) via the controller: "rotate" pulse + "home" pulse, feedback DI
  tcp     : a line-based TCP device: sends "ROT 90\\n" / "ROT 0\\n", expects "OK"
  manual  : asks the operator (supervised mode only); refuses in unsupervised mode
"""
from __future__ import annotations

import socket
import time


class PolarError(Exception):
    pass


class PolarStage:
    def __init__(self, cfg: dict, robot, log, supervised: bool, initial_angle: int = 0):
        self.cfg, self.robot, self.log, self.supervised = cfg, robot, log, supervised
        self.driver = cfg.get("driver", "manual")
        self.settle = float(cfg.get("settle_s", 3.0))
        self.angle = int(initial_angle)
        if not supervised and not cfg.get("di"):
            raise PolarError("unsupervised mode needs polar stage feedback (layout.polar.di)")

    def rotate_to(self, deg: int):
        if deg == self.angle:
            self.log("polar", "rotate", f"already at {deg}")
            return
        self.log("polar", "rotate", f"{self.angle} -> {deg} via {self.driver}")
        getattr(self, f"_drv_{self.driver}")(deg)
        time.sleep(self.settle)
        self._verify(deg)
        self.angle = deg

    # ---- drivers --------------------------------------------------------------
    def _drv_do(self, deg):
        sig = self.cfg["do"]["rotate_90" if deg == 90 else "home"]
        self.robot.write_do(sig, 1)
        time.sleep(float(self.cfg["do"].get("pulse_s", 0.5)))
        self.robot.write_do(sig, 0)

    def _drv_tcp(self, deg):
        host, port = self.cfg["tcp"]["host"], int(self.cfg["tcp"]["port"])
        s = socket.create_connection((host, port), timeout=10)
        try:
            s.sendall(self.cfg["tcp"].get("fmt", "ROT {deg}\n").format(deg=deg).encode())
            reply = s.recv(256).decode(errors="replace").strip()
        finally:
            s.close()
        if self.cfg["tcp"].get("ok", "OK") not in reply:
            raise PolarError(f"polar stage replied {reply!r}")

    def _drv_manual(self, deg):
        if not self.supervised:
            raise PolarError("manual polar driver cannot be used unsupervised")
        input(f"  >>> Rotate the polar stage to {deg} degrees, then press Enter: ")

    # ---- feedback -------------------------------------------------------------
    def _verify(self, deg):
        di = self.cfg.get("di")
        if di:
            sig = di["at_90" if deg == 90 else "at_0"]
            if self.robot.read_di(sig) != 1:
                raise PolarError(f"polar stage feedback {sig} not set after rotate to {deg}")
