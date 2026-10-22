"""
The Bioyond Nova: the Polar arm with its gripper, and the vision box. One import for both.

    arm            the .NET SDK path (devices.arm.VendorRobot): move_linear, move_joint, jaw_to, pose, state, enable
    vision_box     the TCP text protocol (devices.vision_box.VisionClient): frame, led, scan, pose_offset
    controller     the JSON port that starts Lua jobs on the controller (devices.controller.Brcapi)

`arm(layout, log, sim=True)` returns the fake instead, so the loop runs without hardware.
"""
from __future__ import annotations

from devices.arm import (  # noqa: F401
    MotionRefused,
    Robot,
    RobotFault,
    RobotState,
    VendorRobot,
)
from devices.controller import Brcapi, BrcapiError  # noqa: F401
from devices.fake_arm import SimFaults, SimRobot  # noqa: F401
from devices.vision_box import VisionClient, VisionError  # noqa: F401


def arm(layout: dict, log=None, sim: bool = False, faults: SimFaults | None = None) -> Robot:
    """The arm named in layout.json `robot`, or a fake that starts at the microscope seat."""
    if sim:
        start = layout["fixtures"]["microscope"]["pose"] or [120, 60, 120, 0, 0, 0]
        return SimRobot(start, faults or SimFaults(), log, jaw=layout["gripper"]["open"]["short_edges"])
    r = layout["robot"]
    return VendorRobot(r["ip"], r["port"], r["grip_axis"], log)


def vision_box(layout: dict, log=None) -> VisionClient:
    v = layout["vision"]
    return VisionClient(v["host"], v["port"], log=log)


def controller(layout: dict, log=None) -> Brcapi:
    r = layout["robot"]
    return Brcapi(r["ip"], r.get("brcapi_port", 56788), log=log)
