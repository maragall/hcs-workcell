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

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Protocol


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


# ============================================================================ real arm
class VendorRobot:
    def __init__(self, ip: str, port: int, grip_axis: int = 0, log=None):
        self.ip, self.port, self.grip_axis = ip, port, grip_axis
        self._log = log or (lambda *a: None)
        self._r = None
        self._speed = 0.01

    def connect(self):
        try:
            from BioyondSDK import RobotSDKFactory  # vendor wheel; confined to this module
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

    def move_linear(self, p):
        ok = self._r.MoveLinearAsync(*[float(v) for v in p], self._speed)
        self._verify_pose(p, ok, "linear")

    def move_joint(self, p):
        ok = self._r.MoveJointAsync(*[float(v) for v in p], self._speed)
        self._verify_pose(p, ok, "joint")

    def move_abs_joints(self, j, tol: float = 0.3, timeout: float = 120.0):
        """The SDK's own completion check is unreliable (returns False on moves that complete), so completion is
        verified here by reading the joints. Sub-stepped by the caller so each call stays inside the SDK window."""
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

    def ext_axis_to(self, index: int, value: float):
        # readouts are 0-based (extjoints[0] = E1 gripper, [1] = E2 polar stage); move commands are 1-based
        if not self._r.MoveSingleExtJointAsync(index + 1, float(value)):
            raise MotionRefused(f"external axis {index + 1} move refused to {value} (drag mode on? index invalid?)")

    def write_do(self, signal: str, value: int):
        if not self._r.WriteDOAsync(signal, int(value)):
            raise RobotFault(f"WriteDO {signal}={value} refused")

    def read_di(self, signal: str) -> int:
        return int(self._r.ReadDIAsync(signal))


# ============================================================================ simulator
@dataclass
class SimFaults:
    """Fault injection for tests; counters refer to the n-th motion call or n-th jaw close."""
    refuse_move_at: int = -1
    miss_grasp_at: int = -1           # that close ends at 0: nothing between the fingers
    obstruct_at: int = -1             # that close stops at 3.6: obstruction
    stall_at: int = -1                # that motion stops 3 mm short of target
    drop_enable_at: int = -1
    drag_on: bool = False
    ext_error: float = 0.0            # degrees the polar axis lands away from its command
    ext_error: float = 0.0            # degrees the polar axis lands away from its command


class SimRobot:
    """Kinematics-free simulator: moves are instantaneous, poses are what you command."""

    def __init__(self, start_pose, faults: Optional[SimFaults] = None, log=None, jaw=10.0):
        self._pose = list(start_pose)
        self._joints = [0.0] * 8
        self._jaw = jaw
        self._enabled = True
        self._speed = 0.01
        self.faults = faults or SimFaults()
        self._moves = 0
        self._closes = 0
        self._log = log or (lambda *a: None)
        self.dos = {}
        self.dis = {}
        self.trace = []
        self.joint_pose_map = {}      # tests map waypoint joints to the pose the arm would reach
        self._ext = {}
        self._ext = {}

    def connect(self):
        self._log("robot", "connect", "SIMULATOR")

    def disconnect(self):
        pass

    def state(self):
        return RobotState(self._enabled, False, self.faults.drag_on, False, self._speed)

    def enable(self):
        self._enabled = True
        return True

    def set_speed(self, fraction):
        self._speed = fraction

    def pose(self):
        return list(self._pose)

    def joints(self):
        return list(self._joints)

    def jaw(self):
        return self._jaw

    def _motion(self, target, kind):
        self._moves += 1
        if self._moves == self.faults.drop_enable_at:
            self._enabled = False
        if not self._enabled:
            raise MotionRefused("controller not enabled")
        if self._moves == self.faults.refuse_move_at:
            raise MotionRefused(f"simulated refusal ({kind})")
        self._pose = list(target)
        if self._moves == self.faults.stall_at:
            self._pose[2] -= 3.0
        self.trace.append((kind, list(target)))

    def move_linear(self, p):
        self._motion(p, "L")

    def move_joint(self, p):
        self._motion(p, "J")

    def move_abs_joints_stepped(self, j, max_step: float = 8.0):
        self.move_abs_joints(j)

    def move_abs_joints(self, j):
        if not self._enabled:
            raise MotionRefused("controller not enabled")
        self._joints = list(j)
        pose = self.joint_pose_map.get(tuple(round(v, 3) for v in j[:4]))
        if pose is not None:
            self._pose = list(pose)
        self.trace.append(("AJ", list(j)))

    def set_pose_for_test(self, p):
        self._pose = list(p)

    def jaw_to(self, value):
        if self.faults.drag_on:
            raise MotionRefused("gripper refused in drag mode")
        if value >= 9.0:
            self._missed = self._obstructed = False
        elif value <= 3.0 < self._jaw:
            self._closes += 1
            self._missed = self._closes == self.faults.miss_grasp_at
            self._obstructed = self._closes == self.faults.obstruct_at
        if getattr(self, "_missed", False):
            self._jaw = 0.0
            return
        if getattr(self, "_obstructed", False):
            self._jaw = max(value, 3.6)
            return
        self._jaw = value

    def ext_axis(self, index):
        return self._jaw if index == 0 else self._ext.get(index, 0.0)

    def ext_axis_to(self, index, value):
        if index == 0:
            return self.jaw_to(value)
        if self.faults.drag_on:
            raise MotionRefused("external axis refused in drag mode")
        self._ext[index] = float(value) + self.faults.ext_error

    def ext_axis(self, index):
        return self._jaw if index == 0 else self._ext.get(index, 0.0)

    def ext_axis_to(self, index, value):
        if index == 0:
            return self.jaw_to(value)
        if self.faults.drag_on:
            raise MotionRefused("external axis refused in drag mode")
        self._ext[index] = float(value) + self.faults.ext_error

    def write_do(self, signal, value):
        self.dos[signal] = value

    def read_di(self, signal):
        return self.dis.get(signal, 0)
