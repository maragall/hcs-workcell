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
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from procedures.safety import Safety, SafetyViolation
from routes import joints as jt
from routes.volumes import Volumes
from routes_opt import fk

SINGULAR_MM = 60.0     # a wrist this close to the arm's own axis cannot make a straight move
FREE_AIR_LIFT_MM = 20.0   # over a zone's free_above_z by this much when the arm goes in or out sideways
FOLD_MM = 60.0          # a target this close to the axis (the folded nest, the B column's approach) is reached by a joint move, not a straight one
UNFOLD_MM = 100.0      # where the joint move unfolds the arm to, along the line to the target


@dataclass
class Route:
    name: str                                   # trajectory name, or "lift-travel-lower"
    poses: List[List[float]]                    # Cartesian waypoints, straight lines between them
    joints: Optional[List[List[float]]] = None  # a taught route: replay these; poses are its checkpoints
    reverse: bool = False                       # the taught route is stored the other way round
    # Legs that are joint moves: index into `poses` -> the joints to drive to (abs joints, stepped). The planner checked the
    # interpolation from the previous waypoint's joints to these. The controller is never asked for a joint move to a pose:
    # it then picks the elbow branch itself (on 2026-10-28 it chose the mirror one and the body hit the pillar).
    joint_targets: Dict[int, List[float]] = field(default_factory=dict)

    @property
    def taught(self) -> bool:
        return self.joints is not None

    @property
    def first_joint(self) -> bool:             # the first leg is a joint move (unfolding from beside the arm's own axis)
        return 0 in self.joint_targets

    @property
    def last_joint(self) -> bool:              # the last leg is a joint move (folding in to a pose beside the arm's own axis)
        return bool(self.poses) and (len(self.poses) - 1) in self.joint_targets

    @property
    def last_joints(self) -> Optional[List[float]]:
        return self.joint_targets.get(len(self.poses) - 1) if self.poses else None

    def shifted(self, by: int) -> Dict[int, List[float]]:
        return {i + by: j for i, j in self.joint_targets.items()}


_CACHE: Optional[dict] = None      # the edge checks of the plan() in progress: its nested searches share them


class _Fixed:
    """The same Safety, but the arm stands where `joints` says: the planner's view of the rest of a route after a joint leg
    (so its straight legs are checked on the elbow branch the arm will be on)."""

    def __init__(self, safety: Safety, joints: List[float]):
        self._s = safety
        self.joints = lambda: list(joints)

    def __getattr__(self, name):
        return getattr(self._s, name)

    def _branches(self):
        return (fk.elbow_sign(self.joints()),)


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


def _zone(safety: Safety, p: List[float]) -> Optional[dict]:
    """The no-swing region with a free air above it that p is in the restricted air of (below that height), or None."""
    r = safety.regions.get(safety.region_of(p), {})
    return r if r.get("no_swing") and r.get("free_above_z") is not None and p[2] < r["free_above_z"] else None


def _through_the_free_air(layout: dict, safety: Safety, here: List[float], target: List[float],
                          holding: Optional[str]) -> Optional[Route]:
    """No taught route into or out of a no-swing zone (a microscope), but the instrument has a highest part: above it the arm
    may move sideways (regions.<zone>.free_above_z). In: travel to a point over the target at the free height plus
    FREE_AIR_LIFT_MM, then straight down. Out: straight up to that height, then the usual travel. Every leg is checked with the
    claw volumes by the planner it calls. None when neither end is in such a zone."""
    into, out_of = _zone(safety, target), _zone(safety, here)
    if into is not None and out_of is None:
        zfree = into["free_above_z"] + FREE_AIR_LIFT_MM
        over = [target[0], target[1], max(zfree, here[2], target[2])] + list(target[3:])
        route = lift_travel_lower(layout, safety, here, over, holding)
        down = list(target)
        safety.check_segment(over, down, "straight down into the zone")
        Volumes(layout, safety, holding).check_leg(over, down, "straight down into the zone")
        route.poses.append(down)
        return route
    if out_of is not None and into is None:
        up = [here[0], here[1], out_of["free_above_z"] + FREE_AIR_LIFT_MM] + list(here[3:])
        safety.check_segment(here, up, "straight up out of the zone")
        route = lift_travel_lower(layout, safety, up, target, holding)
        route.poses.insert(0, up)
        return route
    return None


def lift_travel_lower(layout: dict, safety: Safety, here: List[float], target: List[float],
                      holding: Optional[str] = None) -> Route:
    """Without a taught route: lift, travel, lower. With `routes.safe_z_mm` the travel is at that height; without it the
    travel is at the higher of the two ends and goes around the solid volumes (routes/volumes.py). Either way every
    leg is checked against the claw and the links, not just the wrist."""
    vol = Volumes(layout, safety, holding, _CACHE)
    safe_z = layout.get("routes", {}).get("safe_z_mm")
    if safe_z is None and not vol.active:
        raise SafetyViolation("no taught route between these positions and routes.safe_z_mm is not set")
    if safe_z is None and vol.active:
        zone_route = _through_the_free_air(layout, safety, here, target, holding)
        if zone_route is not None:
            return zone_route
    x0, y0 = fk.MODEL["x0"], fk.MODEL["y0"]
    if safe_z is None and vol.active and math.dist(target[:2], (x0, y0)) < FOLD_MM:
        # the target is beside the arm's own axis (the folded nest): a straight move cannot end there. Plan to a point
        # UNFOLD_MM out, where the arm is unfolded, and fold in with one joint move that keeps the claw and links clear.
        toward = math.degrees(math.atan2(here[1] - y0, here[0] - x0))
        nest = layout.get("routes", {}).get("nest") or {}
        tj = nest.get("joints_at_teach") if nest.get("pose") and math.dist(nest["pose"][:3], target[:3]) < 1.0 else None
        branch0 = vol.branch
        why = None
        for ang in dict.fromkeys([toward, 90.0, 135.0, 45.0, 180.0, 0.0, -45.0]):
            cand = [x0 + UNFOLD_MM * math.cos(math.radians(ang)), y0 + UNFOLD_MM * math.sin(math.radians(ang)), target[2]] \
                + list(target[3:])
            try:
                safety.check_target(cand, "fold")
                vol.check_pose(cand, "fold")
                route = lift_travel_lower(layout, safety, here, cand, holding)
                # the arm reaches `cand` on the branch it is on; the fold is a joint move from there to the target's joints
                arrive = jt.pose_joints(cand, branch0, None, vol.lim) if branch0 is not None else None
                end = vol.check_joint_leg(cand, target, "fold", joints_b=tj, joints_a=arrive,
                                          branch_b=None if tj else branch0)
            except (SafetyViolation, ValueError) as e:
                why = e
                continue
            route.poses.append(list(target))
            if vol.start_joints is not None or tj:
                route.joint_targets[len(route.poses) - 1] = list(end)
            return route
        raise SafetyViolation(f"no way to fold the arm in to the pose beside its own axis: {why}")
    withdrawn = None
    if vol.active:
        try:
            vol.settle(here)
        except SafetyViolation as why:      # the arm stands inside an obstacle (it touched it and stopped): back out first
            withdrawn = vol.withdraw(here)
            if withdrawn is None:
                raise SafetyViolation(f"{why}; and there is no clear spot straight back along the claw")
            safety.check_segment(here, withdrawn, "withdraw")
            vol.relax = None
            here = withdrawn
        vol.check_pose(target, "the target", near_axis_ok=True)
    if safe_z is None and vol.active and math.dist(here[:2], (x0, y0)) < SINGULAR_MM:
        # the wrist is beside the arm's own axis: a straight move from there is a singularity, the controller refuses it. A
        # joint move unfolds the arm first, to a point UNFOLD_MM from the axis at a direction and claw yaw whose joint-
        # interpolated path keeps the claw and links clear. From a folded arm either elbow branch can be unfolded to (the
        # joints pass j3 = 0): the first branch a route exists on is taken, and the rest is planned from there on that branch.
        if withdrawn is not None:
            raise SafetyViolation("the arm stands inside an obstacle beside its own axis: move it by hand")
        toward = math.degrees(math.atan2(target[1] - y0, target[0] - x0))
        have = vol.start_joints is not None
        own = vol.branch
        branches = ([own] + [b for b in (1, -1) if b != own]) if have else [None]
        why = None
        for ang in dict.fromkeys([toward, 90.0, 135.0, 45.0, 180.0, 0.0, -45.0]):
            for yaw in dict.fromkeys([here[5], target[5], 0.0, 90.0, -90.0, 180.0]):
                for branch in branches:
                    cand = [x0 + UNFOLD_MM * math.cos(math.radians(ang)), y0 + UNFOLD_MM * math.sin(math.radians(ang)), here[2]] \
                        + list(here[3:5]) + [yaw]
                    try:
                        end = vol.check_joint_leg(here, cand, "unfold", start_actual=True, branch_b=branch)
                        safety.check_target(cand, "unfold")
                        rest = lift_travel_lower(layout, _Fixed(safety, end) if have else safety, cand, target, holding)
                    except (SafetyViolation, ValueError) as e:
                        why = e
                        continue
                    route = Route("lift-travel-lower", [cand] + rest.poses, joint_targets=rest.shifted(1))
                    if have:
                        route.joint_targets[0] = list(end)
                    return route
        raise SafetyViolation("no way to unfold the arm from beside its own axis without the claw meeting the hotel or the "
                              f"pillar, or leaving the joints' range: {why}")
    if safe_z is None:
        z = max(here[2], target[2])
        up = list(here[:2]) + [z] + list(here[3:])
        way = vol.search(up, target)
        candidates = [up] + way[1:] + [target]
    else:
        up = list(here[:2]) + [safe_z] + list(here[3:])
        candidates = [up, list(target[:2]) + [safe_z] + list(target[3:]), target]
    prefix = [withdrawn] if withdrawn else []
    start = prefix[-1] if prefix else here
    poses, prev = list(prefix), here
    for p in candidates:                       # a zero-length leg is skipped; the route always ends at the target
        if p != prev:
            poses.append(p)
            prev = p
    # a turn of more than a quarter turn while moving is a turn in place first (the controller picks its own direction for a
    # straight leg that turns the wrist far; the in-place turn is a joint leg with the direction the planner checked)
    spread, prev = list(poses[:len(prefix)]), start
    for p in poses[len(prefix):]:
        if vol.branch is not None and math.dist(prev[:3], p[:3]) > 1.0 and abs(fk.wrap180(p[5] - prev[5])) > 90.0:
            spread.append(list(prev[:5]) + [p[5]])
        spread.append(p)
        prev = p
    poses = spread
    route = Route("lift-travel-lower", poses)
    prev, cj = start, (list(vol.start_joints) if vol.start_joints is not None else None)
    if cj is not None and prefix:
        cj = jt.pose_joints(start, vol.branch, cj, vol.lim)       # the straight withdraw leaves the wrist where it was
    for i, p in enumerate(poses[len(prefix):], 1):
        safety.check_segment(prev, p, f"route {i}/{len(poses)}")
        if vol.active:
            vol.check_leg(prev, p, f"route {i}/{len(poses)}")
        if cj is not None:
            ja, jb, _ = jt.leg_joints(prev, p, vol.branch, vol.lim, cj)
            if math.dist(prev[:3], p[:3]) <= 1.0 and abs(fk.wrap180(p[5] - prev[5])) > 0.5:
                route.joint_targets[len(prefix) + i - 1] = list(jb)           # a turn in place: j4 (and the rest) by joint value
            cj = jb
        prev = p
    return route


def _via_nest(layout: dict, safety: Safety, here: List[float], target: List[float], holding: Optional[str]) -> Route:
    """The arm cannot get there on the branch it is on (its straight legs leave the joints' range, or the target is reachable only
    on the other elbow branch). The two branches meet only in the folded pose, where j3 passes through 0: the nest, taught by the
    operator, is one. A joint leg from where the arm stands to the nest's own joints, then a route from the nest (a folded start
    unfolds to either branch). Both joint legs are checked sample by sample against the claw, the links and the joint limits and
    carry the joints to drive to."""
    nest = layout.get("routes", {}).get("nest") or {}
    nj, npose = nest.get("joints_at_teach"), nest.get("pose")
    if not nj or not npose:
        raise SafetyViolation("routes.nest has no joints_at_teach: the way through the folded pose is not known")
    if not safety.joints:
        raise SafetyViolation("the arm's joints are not known: cannot plan a branch change")
    vol = Volumes(layout, safety, holding, _CACHE)
    if vol.active:
        vol.settle(here)
    end = vol.check_joint_leg(here, list(npose), "to the nest", start_actual=True, joints_b=nj)
    poses = [list(npose)]
    targets = {0: list(end)}
    if math.dist(npose[:3], target[:3]) > 1.0:
        rest = lift_travel_lower(layout, _Fixed(safety, list(nj)), list(npose), target, holding)
        poses += rest.poses
        targets.update(rest.shifted(1))
    return Route("via-the-nest", poses, joint_targets=targets)


def plan(layout: dict, safety: Safety, here: List[float], target: List[float],
         from_name: Optional[str] = None, to_name: Optional[str] = None, holding: Optional[str] = None) -> Route:
    route = taught_route(layout, from_name, to_name)
    if route is not None:
        for i, p in enumerate(route.poses, 1):
            safety.check_target(p, f"{route.name}[{i}/{len(route.poses)}]")
        return route
    global _CACHE
    _CACHE = {}
    try:
        return _plan(layout, safety, here, target, holding)
    finally:
        _CACHE = None


def _plan(layout, safety, here, target, holding):
    try:
        return lift_travel_lower(layout, safety, here, target, holding)
    except SafetyViolation as direct:
        if not (safety.joints and layout.get("routes", {}).get("nest")):
            raise
        try:
            return _via_nest(layout, safety, here, target, holding)
        except SafetyViolation as hub:
            raise SafetyViolation(f"{direct}; and through the nest: {hub}")
