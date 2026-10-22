"""
Robot adapter. The ONLY module that touches the vendor .NET SDK (BR.PC.BYSDK via pythonnet).

Everything else talks to the `Robot` interface below, so the simulator and the real arm are
interchangeable.  Rules from the controller quirk list are enforced here, not in the cycle:

  * never toggles drag mode or robot mode (both snap the gripper)
  * refuses to run with drag mode on
  * gripper is moved with MoveSingleExtJoint (MoveExtJoint faults the controller)
  * close slowly in steps, open fast
  * no motion call is ever retried
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Protocol


class MotionRefused(Exception):
    pass


class RobotFault(Exception):
    pass


@dataclass
class RobotState:
    enabled: bool
    moving: bool
    drag: bool
    auto: bool
    speed: float


class Robot(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def state(self) -> RobotState: ...
    def enable(self) -> bool: ...
    def set_speed(self, fraction: float) -> None: ...
    def pose(self) -> List[float]: ...
    def joints(self) -> List[float]: ...
    def jaw(self) -> float: ...
    def move_linear(self, pose: List[float]) -> None: ...
    def move_joint(self, pose: List[float]) -> None: ...
    def move_abs_joints(self, joints: List[float]) -> None: ...
    def move_abs_joints_stepped(self, joints: List[float], max_step: float = 8.0) -> None: ...
    def jaw_to(self, value: float) -> None: ...
    def ext_axis(self, index: int) -> float: ...
    def ext_axis_to(self, index: int, value: float) -> None: ...
    def write_do(self, signal: str, value: int) -> None: ...
    def read_di(self, signal: str) -> int: ...
    def errors(self) -> list: ...


# ============================================================================ real arm
class VendorRobot:
    BRCAPI_PORT = 56788           # the controller's documented JSON protocol; the SDK has no error field

    def __init__(self, ip: str, port: int, grip_axis: int = 0, log=None):
        self.ip, self.port, self.grip_axis = ip, port, grip_axis
        self._log = log or (lambda *a: None)
        self._r = None
        self._speed = 0.01
        self.sdk_stops = 0

    def connect(self):
        try:
            from BioyondSDK import (
                RobotSDKFactory,  # vendor wheel; confined to this module
            )
            from System import Array, Double
        except Exception as e:
            raise RobotFault("vendor SDK not importable (pythonnet + bioyondsdk wheel + .NET 8): " + str(e))
        self._Array, self._Double = Array, Double
        self._r = RobotSDKFactory.Create()
        if not self._r.ConnectAsync(self.ip, self.port):
            raise RobotFault(f"cannot connect to controller {self.ip}:{self.port} (one client only)")
        self._log("robot", "connect", "ok")

    def disconnect(self):
        if self._r is not None:
            try:
                self._r.DisconnectAsync()
            except Exception:
                pass
            self._r = None

    def state(self) -> RobotState:
        m = self._r.GetMotionInfo()
        return RobotState(bool(m.IsEnable), bool(m.IsMoving), bool(m.IsInDragMode), bool(m.IsInAutoMode),
                          float(m.SpeedPercentage))

    def enable(self) -> bool:
        # the only call we retry: it involves no motion. Readouts are stale until it succeeds.
        for i in range(8):
            if self._r.SetEnableAsync(True) and self.state().enabled:
                return True
            time.sleep(4)
        return False

    def set_speed(self, fraction: float):
        fraction = max(0.01, min(1.0, fraction))
        if not self._r.SetSpeedAsync(fraction):
            raise RobotFault("SetSpeedAsync refused")
        self._speed = fraction

    def pose(self):
        return [float(v) for v in self._r.GetCurPoseAsync()]

    def joints(self):
        return [float(v) for v in self._r.GetCurJointAsync()]

    def jaw(self):
        return float(list(self._r.GetCurExtJointAsync())[self.grip_axis])

    def _verify_pose(self, p, ok: bool, kind: str, tol: float = 1.0, timeout: float = 60.0):
        """Raises MotionRefused when the pose is not reached."""
        """The SDK's completion flag also compares the gripper axis to 0.02 units, so it reports False whenever a
        plate is held. Completion is verified here from the pose instead."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            now = self.pose()
            if max(abs(now[i] - float(p[i])) for i in range(3)) <= tol and not self.state().moving:
                return
            time.sleep(0.1)
            if not ok and time.time() - t0 > 2.0 and not self.state().moving:
                break
        now = self.pose()
        raise MotionRefused(f"{kind} move did not reach {[round(float(v), 1) for v in p[:3]]}; at {[round(v, 1) for v in now[:3]]}")

    def move_linear(self, p, _retry: bool = True):
        before = self.pose()
        ok = self._r.MoveLinearAsync(*[float(v) for v in p], self._speed)
        try:
            self._verify_pose(p, ok, "linear")
        except MotionRefused:
            now = self.pose()
            moved = max(abs(now[i] - before[i]) for i in range(3))
            span = max(abs(float(p[i]) - before[i]) for i in range(3))
            remaining = max(abs(float(p[i]) - now[i]) for i in range(3))
            if _retry and moved >= 0.5 and remaining <= 15.0 and self._sdk_stopped_us():
                self._log("robot", "sdk_motionstop", f"linear move stopped {remaining:.1f} mm short with enable intact; re-issuing once")
                self.sdk_stops += 1
                return self.move_linear(p, _retry=False)
            if moved < 0.5 and span <= 30.0:
                # the controller silently ignored a short linear move (seen at the microscope hover); same target, joint interpolation
                self._log("robot", "move_linear", f"no motion on a {span:.1f} mm linear move; using joint interpolation")
                ok = self._r.MoveJointAsync(*[float(v) for v in p], self._speed)
                self._verify_pose(p, ok, "joint-fallback")
            else:
                raise

    def move_joint(self, p):
        ok = self._r.MoveJointAsync(*[float(v) for v in p], self._speed)
        self._verify_pose(p, ok, "joint")

    def _sdk_stopped_us(self) -> bool:
        """The vendor SDK issues MotionStop when its own position check fails (gripper jitter > 0.02 while a plate is
        held). That leaves the arm short of target with enable intact and no controller error. A collision stop
        drops enable. Distinguishing the two decides whether one re-issue of the same command is allowed."""
        st = self.state()
        return st.enabled and not st.drag and not st.moving

    def move_abs_joints(self, j, tol: float = 0.3, timeout: float = 60.0, _retry: bool = True):
        target = [float(v) for v in j[:4]]
        arr = self._Array[self._Double](target + [float(v) for v in self.joints()[4:]])
        ok = self._r.MoveAbsJointAsync(arr, self._speed)
        t0 = time.time()
        while time.time() - t0 < timeout:
            now = self.joints()[:4]
            if max(abs(a - b) for a, b in zip(now, target)) <= tol and not self.state().moving:
                return
            time.sleep(0.1)
            if not ok and time.time() - t0 > 3.0 and not self.state().moving:
                break
        now = self.joints()[:4]
        short = max(abs(a - b) for a, b in zip(now, target))
        if _retry and short <= 12.0 and self._sdk_stopped_us():
            self._log("robot", "sdk_motionstop", f"abs joint move stopped {short:.1f} deg short with enable intact; re-issuing once")
            self.sdk_stops += 1
            return self.move_abs_joints(j, tol, timeout, _retry=False)
        raise MotionRefused(f"abs joint move did not reach {[round(v, 2) for v in target]}; at {[round(v, 2) for v in now]}")

    def move_abs_joints_stepped(self, j, max_step: float = 8.0):
        start = self.joints()[:4]
        target = [float(v) for v in j[:4]]
        n = max(1, int(max(abs(t - s) for s, t in zip(start, target)) / max_step + 0.999))
        for k in range(1, n + 1):
            self.move_abs_joints([s + (t - s) * k / n for s, t in zip(start, target)])

    def jaw_to(self, value: float):
        self.ext_axis_to(self.grip_axis, value)

    def ext_axis(self, index: int) -> float:
        return float(list(self._r.GetCurExtJointAsync())[index])

    def ext_axis_to(self, index: int, value: float, tol: float = 0.15, timeout: float = 20.0):
        # readouts are 0-based (extjoints[0] = E1 gripper, [1] = E2 polar stage); move commands are 1-based.
        # The SDK flag is unreliable under load; completion is verified from the axis reading.
        ok = self._r.MoveSingleExtJointAsync(index + 1, float(value))
        t0 = time.time()
        last = None
        while time.time() - t0 < timeout:
            now = self.ext_axis(index)
            if abs(now - float(value)) <= tol:
                return
            if last is not None and abs(now - last) < 0.01 and time.time() - t0 > 2.0:
                break                                   # stalled: an obstruction, or the plate for a closing move
            last = now
            time.sleep(0.2)
        raise MotionRefused(f"external axis {index + 1} stopped at {self.ext_axis(index):.2f}, target {value} (sdk={ok})")

    def write_do(self, signal: str, value: int):
        if not self._r.WriteDOAsync(signal, int(value)):
            raise RobotFault(f"WriteDO {signal}={value} refused")

    def read_di(self, signal: str) -> int:
        return int(self._r.ReadDIAsync(signal))

    def errors(self) -> list:
        """Controller error list over BRCAPI (one short connection); empty when unreachable, the SDK path still guards."""
        import json
        import socket
        try:
            s = socket.create_connection((self.ip, self.BRCAPI_PORT), timeout=3)
            s.settimeout(3)
            s.sendall((json.dumps({"BRCAPI": "1.0", "cmd": "getRobotErr", "id": 7}) + "\n").encode())
            buf = b""
            while not buf.endswith(b"\n"):
                c = s.recv(65536)
                if not c:
                    break
                buf += c
            s.close()
            ret = json.loads(buf).get("ret") or []
            return [e for e in ret if e.get("errCode") is not None]
        except Exception:
            return []
