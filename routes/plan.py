"""
Route planning between two positions.

    plan(layout, safety, here, target, from_name, to_name) -> Route

A taught route wins: if the positions file has a trajectory named <from>_to_<to> (or the reverse),
the route replays its joint waypoints. Otherwise the route is lift, travel at the safe height,
lower: three straight lines, each checked against the workspace box, the region floors and the
no-swing rule before anything moves. The safe height is `routes.safe_z_mm` in the positions file;
without it, and without a taught route, planning refuses. That is on purpose: a number nobody
measured must not move the arm.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from procedures.safety import Safety, SafetyViolation


@dataclass
class Route:
    name: str                                   # trajectory name, or "lift-travel-lower"
    poses: List[List[float]]                    # Cartesian waypoints, straight lines between them
    joints: Optional[List[List[float]]] = None  # a taught route: replay these; poses are its checkpoints
    reverse: bool = False                       # the taught route is stored the other way round

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


def lift_travel_lower(layout: dict, safety: Safety, here: List[float], target: List[float]) -> Route:
    safe_z = layout.get("routes", {}).get("safe_z_mm")
    if safe_z is None:
        raise SafetyViolation("no taught route between these positions and routes.safe_z_mm is not set")
    up = list(here[:2]) + [safe_z] + list(here[3:])
    across = list(target[:2]) + [safe_z] + list(target[3:])
    poses, prev = [], here
    for p in (up, across, target):            # a zero-length leg is skipped; the route always ends at the target
        if p != prev:
            poses.append(p)
            prev = p
    prev = here
    for i, p in enumerate(poses, 1):
        safety.check_segment(prev, p, f"route {i}/{len(poses)}")
        prev = p
    return Route("lift-travel-lower", poses)


def plan(layout: dict, safety: Safety, here: List[float], target: List[float],
         from_name: Optional[str] = None, to_name: Optional[str] = None) -> Route:
    route = taught_route(layout, from_name, to_name)
    if route is not None:
        for i, p in enumerate(route.poses, 1):
            safety.check_target(p, f"{route.name}[{i}/{len(route.poses)}]")
        return route
    return lift_travel_lower(layout, safety, here, target)
