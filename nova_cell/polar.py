"""
Polar (rotating) stage driver.

  ext_axis : the stage is the robot's external axis (confirmed 2026-10-15: E2 on this cell). Commanded
             through the robot adapter and verified by reading the axis back. Default.
  do       : robot digital output pulses with DI feedback
  tcp      : a line-based TCP device
  manual   : asks the operator; supervised mode only
"""
from __future__ import annotations

import socket
import time


class PolarError(Exception):
    pass


class PolarStage:
    def __init__(self, cfg: dict, robot, log, supervised: bool, initial_angle: int = 0):
        self.cfg, self.robot, self.log, self.supervised = cfg, robot, log, supervised
        self.driver = cfg.get("driver", "ext_axis")
        self.settle = float(cfg.get("settle_s", 1.0))
        self.angle = int(initial_angle)
        if self.driver == "ext_axis" and cfg["ext_axis"].get("zero_reading") is None:
            raise PolarError("polar zero reference not set (run.py polar zero)")
        if not supervised and self.driver != "ext_axis" and not cfg.get("di"):
            raise PolarError("unsupervised mode needs polar stage feedback (ext_axis driver or layout.polar.di)")

    def rotate_to(self, deg: int):
        if deg == self.angle and self._at(deg):
            self.log("polar", "rotate", f"already at {deg}")
            return
        self.log("polar", "rotate", f"{self.angle} -> {deg} via {self.driver}")
        getattr(self, f"_drv_{self.driver}")(deg)
        time.sleep(self.settle)
        self._verify(deg)
        self.angle = deg

    # ---- ext_axis -------------------------------------------------------------
    def _axis_target(self, deg: int) -> float:
        a = self.cfg["ext_axis"]
        return a["zero_reading"] + a["direction"] * deg

    def read_angle(self) -> float:
        a = self.cfg["ext_axis"]
        return (self.robot.ext_axis(int(a["index"])) - a["zero_reading"]) * a["direction"]

    def _at(self, deg: int) -> bool:
        if self.driver != "ext_axis":
            return True
        a = self.cfg["ext_axis"]
        return abs(self.robot.ext_axis(int(a["index"])) - self._axis_target(deg)) <= float(a.get("tolerance_deg", 0.5))

    def _drv_ext_axis(self, deg: int):
        a = self.cfg["ext_axis"]
        self.robot.set_speed(float(a.get("speed", 0.02)))
        self.robot.ext_axis_to(int(a["index"]), self._axis_target(deg))

    # ---- other drivers --------------------------------------------------------
    def _drv_do(self, deg: int):
        sig = self.cfg["do"]["rotate_90" if deg == 90 else "home"]
        self.robot.write_do(sig, 1)
        time.sleep(float(self.cfg["do"].get("pulse_s", 0.5)))
        self.robot.write_do(sig, 0)

    def _drv_tcp(self, deg: int):
        host, port = self.cfg["tcp"]["host"], int(self.cfg["tcp"]["port"])
        s = socket.create_connection((host, port), timeout=10)
        try:
            s.sendall(self.cfg["tcp"].get("fmt", "ROT {deg}\n").format(deg=deg).encode())
            reply = s.recv(256).decode(errors="replace").strip()
        finally:
            s.close()
        if self.cfg["tcp"].get("ok", "OK") not in reply:
            raise PolarError(f"polar stage replied {reply!r}")

    def _drv_manual(self, deg: int):
        if not self.supervised:
            raise PolarError("manual polar driver cannot be used unsupervised")
        input(f"  >>> Rotate the polar stage to {deg} degrees, then press Enter: ")

    # ---- feedback -------------------------------------------------------------
    def _verify(self, deg: int):
        if self.driver == "ext_axis":
            a = self.cfg["ext_axis"]
            actual = self.robot.ext_axis(int(a["index"]))
            err = abs(actual - self._axis_target(deg))
            if err > float(a.get("tolerance_deg", 0.5)):
                raise PolarError(f"polar axis at {actual:.2f}, expected {self._axis_target(deg):.2f} ({err:.2f} deg off)")
            return
        di = self.cfg.get("di")
        if di:
            sig = di["at_90" if deg == 90 else "at_0"]
            if self.robot.read_di(sig) != 1:
                raise PolarError(f"polar stage feedback {sig} not set after rotate to {deg}")
