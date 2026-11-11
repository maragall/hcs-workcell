"""A station's viewing pose: offsets in the claw's frame at the seat, and the two legs from the hover to it."""
from __future__ import annotations

import math


def tool_frame(p0, pose):
    """pose - p0 in the claw's frame at p0: forward (along the fingers), left, up."""
    yaw = math.radians(p0[5])
    dx, dy, dz = pose[0] - p0[0], pose[1] - p0[1], pose[2] - p0[2]
    return [round(dx * math.cos(yaw) + dy * math.sin(yaw), 2), round(-dx * math.sin(yaw) + dy * math.cos(yaw), 2), round(dz, 2)]


def from_tool_frame(p0, fwd, left, up):
    yaw = math.radians(p0[5])
    return [p0[0] + fwd * math.cos(yaw) - left * math.sin(yaw), p0[1] + fwd * math.sin(yaw) + left * math.cos(yaw), p0[2] + up] + list(p0[3:])


def to_view(cyc, station, target):
    """From the station's hover (the planner's end) to its viewing pose: straight up to the viewing height first, then sideways at that
    height. Inside the microscope's zone a sideways move is allowed only above its instrument (regions.microscope.free_above_z); a single
    diagonal leg from the hover is refused there (2026-11-03). Both legs at the slow contact speed."""
    here = cyc.robot.pose()
    if target[2] > here[2] + 1.0:
        cyc._move("L", [here[0], here[1], target[2]] + list(here[3:]), f"{station} view: up to z {target[2]:.0f}", cyc.safety.profile.contact)
    cyc._move("L", target, f"{station} view pose", cyc.safety.profile.contact)
