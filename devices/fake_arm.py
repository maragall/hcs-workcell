"""
Fake arm for tests: kinematics-free, moves are instantaneous, poses are what you command.
Same interface as devices.arm.VendorRobot, with fault injection (SimFaults).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from devices.arm import MotionRefused, RobotState


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

    def follow_layout(self, layout: dict) -> None:
        """Let the fake reach the layout's taught routes: each joint waypoint maps to its demo pose, the stage
        axis reads its zero, the stage sensors say "seated". What tests/rig.py and exercise.py need."""
        for wp in layout["trajectories"].values():
            if wp:
                for j, pose in zip(wp["joints"], wp["poses"]):
                    self.joint_pose_map[tuple(round(v, 3) for v in j[:4])] = pose
        self._ext[1] = layout["polar"]["ext_axis"].get("zero_reading") or 0.0
        self.dis = {"P0": 1, "P90": 1}

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

    def write_do(self, signal, value):
        self.dos[signal] = value

    def read_di(self, signal):
        return self.dis.get(signal, 0)
