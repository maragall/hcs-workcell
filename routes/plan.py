"""
Route planning between two positions.

    plan(layout, safety, here, target, from_name, to_name) -> Route

A taught route wins: if the positions file has a trajectory named <from>_to_<to> (or the reverse),
the route replays its joint waypoints. Otherwise the route is lift, travel at the safe height,
lower: three straight lines, each checked against the workspace box, the region floors and the
no-swing rule before anything moves. The safe height is `routes.safe_z_mm` in the positions file;
without it the route goes around the volumes in layout.json; with neither, and without a taught route, planning
refuses. That is on purpose: a number nobody measured must not move the arm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

from procedures.safety import Safety, SafetyViolation
from routes.volumes import Volumes
from routes_opt import fk

SINGULAR_MM = 60.0     # a wrist this close to the arm's own axis cannot make a straight move
UNFOLD_MM = 100.0      # where the joint move unfolds the arm to, along the line to the target


@dataclass
class Route:
    name: str                                   # trajectory name, or "lift-travel-lower"
    poses: List[List[float]]                    # Cartesian waypoints, straight lines between them
    joints: Optional[List[List[float]]] = None  # a taught route: replay these; poses are its checkpoints
    reverse: bool = False                       # the taught route is stored the other way round
    first_joint: bool = False                   # the first leg is a joint move (unfolding from beside the arm's own axis)

    @property
    def taught(self) -> bool:
        return self.joints is not None


def _family(name: Optional[str]) -> Optional[str]:
    """polar.pose_at_0 -> polar: taught routes are named by fixture family."""
    return name.split(".")[0] if name else None


def taught_route(layout: dict, from_name: Optional[str], to_name: Optional[str]) -> Optional[Route]:
    a, b = _family(from_name), _family(to_name)
    if not a or not b:
        return None
    tr = layout.get("trajectories", {})
    for name, reverse in ((f"{a}_to_{b}", False), (f"{b}_to_{a}", True)):
        wp = tr.get(name)
        if wp:
            poses, joints = list(wp["poses"]), list(wp["joints"])
            if reverse:
                poses, joints = poses[::-1], joints[::-1]
            return Route(name, poses, joints, reverse)
    return None


def lift_travel_lower(layout: dict, safety: Safety, here: List[float], target: List[float],
                      holding: Optional[str] = None) -> Route:
    """Without a taught route: lift, travel, lower. With `routes.safe_z_mm` the travel is at that height; without it the
    travel is at the higher of the two ends and goes around the solid volumes (routes/volumes.py). Either way every
    leg is checked against the claw and the links, not just the wrist."""
    vol = Volumes(layout, safety, holding)
    safe_z = layout.get("routes", {}).get("safe_z_mm")
    if safe_z is None and not vol.active:
        raise SafetyViolation("no taught route between these positions and routes.safe_z_mm is not set")
    if vol.active:
        vol.settle(here)
        vol.check_pose(target, "the target")
    unfold = None
    if safe_z is None and vol.active and math.dist(here[:2], (fk.MODEL["x0"], fk.MODEL["y0"])) < SINGULAR_MM:
        # the wrist is beside the arm's own axis: a straight move from there is a singularity, the controller refuses
        # it. A joint move unfolds the arm first, to a point UNFOLD_MM from the axis at a direction and claw yaw whose
        # joint-interpolated path keeps the claw and links clear, and from which a route to the target exists.
        x0, y0 = fk.MODEL["x0"], fk.MODEL["y0"]
        toward = math.degrees(math.atan2(target[1] - y0, target[0] - x0))
        why = None
        for ang in dict.fromkeys([toward, 90.0, 135.0, 45.0, 180.0, 0.0, -45.0]):
            for yaw in dict.fromkeys([here[5], target[5], 0.0, 90.0, -90.0, 180.0]):
                cand = [x0 + UNFOLD_MM * math.cos(math.radians(ang)), y0 + UNFOLD_MM * math.sin(math.radians(ang)), here[2]]                     + list(here[3:5]) + [yaw]
                try:
                    vol.check_joint_leg(here, cand, "unfold")
                    safety.check_target(cand, "unfold")
                    z = max(cand[2], target[2])
                    vol.search(list(cand[:2]) + [z] + list(cand[3:]), target)
                except SafetyViolation as e:
                    why = e
                    continue
                unfold = cand
                break
            if unfold:
                break
        if unfold is None:
            raise SafetyViolation(f"no way to unfold the arm from beside its own axis without the claw meeting the hotel or the pillar: {why}")
        here = unfold
    if safe_z is None:
        z = max(here[2], target[2])
        up = list(here[:2]) + [z] + list(here[3:])
        way = vol.search(up, target)
        candidates = [up] + way[1:] + [target]
    else:
        up = list(here[:2]) + [safe_z] + list(here[3:])
        candidates = [up, list(target[:2]) + [safe_z] + list(target[3:]), target]
    start = unfold or here
    poses, prev = ([unfold] if unfold else []), here
    for p in candidates:                       # a zero-length leg is skipped; the route always ends at the target
        if p != prev:
            poses.append(p)
            prev = p
    prev = start
    for i, p in enumerate(poses[1 if unfold else 0:], 1):
        safety.check_segment(prev, p, f"route {i}/{len(poses)}")
        if vol.active:
            vol.check_leg(prev, p, f"route {i}/{len(poses)}")
        prev = p
    return Route("lift-travel-lower", poses, first_joint=unfold is not None)


def plan(layout: dict, safety: Safety, here: List[float], target: List[float],
         from_name: Optional[str] = None, to_name: Optional[str] = None, holding: Optional[str] = None) -> Route:
    route = taught_route(layout, from_name, to_name)
    if route is not None:
        for i, p in enumerate(route.poses, 1):
            safety.check_target(p, f"{route.name}[{i}/{len(route.poses)}]")
        return route
    return lift_travel_lower(layout, safety, here, target, holding)
