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

import copy
import hashlib
import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from planner.safety import Safety, SafetyViolation
from planner import joints as jt
from planner.volumes import Volumes
from planner import fk

SINGULAR_MM = 60.0     # a wrist this close to the arm's own axis cannot make a straight move
FREE_AIR_LIFT_MM = 20.0   # over a zone's free_above_z by this much when the arm goes in or out sideways
FOLD_MM = 60.0          # a target this close to the axis (the B column's entry point) is reached by a joint move, not a straight one
UNFOLD_MM = 100.0      # where the joint move unfolds the arm to, along the line to the target
ENTRY_ZONE_MM = 30.0   # a held plate square to the solid hotel with its front edge this close to the hotel's front stands at a slot's entry point


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


def _at_entry(vol: Volumes, p: List[float]) -> bool:
    """A plate in the claw, square to a solid hotel, its front edge within ENTRY_ZONE_MM of the hotel's front: a slot's entry point."""
    h = vol.obstacles.get("hotel") or {}
    ahead = [x[1] for x, _, held in vol.footprint if held]
    return bool(h.get("solid") and ahead and abs(fk.wrap180(p[5])) <= vol.front_yaw_tol
                and p[0] + max(ahead) >= h["x"][0] - ENTRY_ZONE_MM and h["y"][0] <= p[1] <= h["y"][1])


def _turn_in_place(vol: Volumes, joints: Optional[List[float]], a: List[float], b: List[float], label: str) -> Optional[List[float]]:
    """The wrist stays at a, the claw turns to b's yaw: joint 4 alone, the short way, or the long way when the short one leaves the
    joint's range or meets something. Every 2 degrees of the turn is checked with the claw and the plate. Returns the end joints (None
    when the planner does not know the arm's joints: then both elbow branches are checked)."""
    if joints is None:
        vol.check_joint_leg(a, b, label)
        return None
    turn = fk.wrap180(b[5] - fk.fk(joints)[5])
    why = None
    for t in (turn, turn - 360.0 if turn > 0 else turn + 360.0):
        try:
            return vol.check_joint_leg(a, b, label, joints_a=list(joints[:4]), joints_b=list(joints[:3]) + [joints[3] + t])
        except SafetyViolation as e:
            why = why or e
    raise why


def end_joints(route: "Route", here: List[float], joints: List[float], lim=None) -> List[float]:
    """The joints the arm has at the end of a planned route that started at `here` with `joints`: a joint leg ends at its target, a
    straight leg keeps the elbow branch and turns the wrist by the joints' own direction."""
    cj, prev = list(joints[:4]), here
    for i, p in enumerate(route.poses):
        if i in route.joint_targets:
            cj = list(route.joint_targets[i][:4])
        else:
            cj = jt.leg_joints(prev, p, fk.elbow_sign(cj), lim, cj)[1]
        prev = p
    return cj


def _entry_turns(layout: dict, safety: Safety, vol: Volumes, here: List[float], target: List[float], holding: Optional[str]) -> Route:
    """The one way a carried plate reaches or leaves a slot's entry point, the same for every column (operator, 2026-11-10: column B's
    route turned the plate while its front was 16 mm inside the hotel). The wrist travels with the plate pointing sideways or away from
    the hotel; the turn to the slot's yaw is a turn in place at the entry point, joint 4 alone, and a route from an entry point begins
    with that turn before the wrist moves. The yaws tried: the plate toward the arm's side of the cell, the other side, straight back;
    the other end's own yaw instead of one of these when it is within 5 degrees of it (the module seat at -90.75: no extra small turn)."""
    leave, arrive = _at_entry(vol, here), _at_entry(vol, target)

    def sides(p, other):
        toward = -90.0 if p[1] > fk.MODEL["y0"] else 90.0
        return [other[5] if abs(fk.wrap180(other[5] - y)) <= 5.0 else y for y in (toward, -toward, 180.0)]

    why = []
    for ya in (sides(here, target) if leave else [None]):
        for yb in (sides(target, here) if arrive else [None]):
            tag = ", ".join(f"{k} the entry point at yaw {y:.0f}" for k, y in (("from", ya), ("to", yb)) if y is not None)
            try:
                poses, targets, start, s, cj = [], {}, here, safety, vol.start_joints
                goal = (list(target[:5]) + [yb]) if arrive else list(target)
                if arrive:
                    safety.check_target(target, "the entry point")
                    if cj is not None and vol.branch is not None and not leave:      # the cheap check first: the last turn, from the joints the wrist will have there
                        _turn_in_place(vol, jt.pose_joints(goal, vol.branch, None, vol.lim), goal, target, "turn at the entry point")
                if leave:
                    out = list(here[:5]) + [ya]
                    end = _turn_in_place(vol, cj, here, out, "turn at the entry point")
                    poses.append(out)
                    if end is not None:
                        targets[0], s, cj = list(end), _Fixed(safety, end), end
                    start = out
                rest = lift_travel_lower(layout, s, start, goal, holding)
                targets.update(rest.shifted(len(poses)))
                poses += rest.poses
                if arrive:
                    if cj is not None:
                        cj = end_joints(rest, start, cj, vol.lim)
                    end = _turn_in_place(vol, cj, goal, target, "turn at the entry point")
                    poses.append(list(target))
                    if end is not None:
                        targets[len(poses) - 1] = list(end)
            except (SafetyViolation, ValueError) as e:
                why.append(f"{tag}: {e}")
                continue
            return Route("lift-travel-lower", poses, joint_targets=targets)
    raise SafetyViolation("no way to carry the plate to or from the entry point with the turn in place there, for any yaw tried:\n  "
                          + "\n  ".join(why))


def lift_travel_lower(layout: dict, safety: Safety, here: List[float], target: List[float],
                      holding: Optional[str] = None) -> Route:
    """Without a taught route: lift, travel, lower. With `routes.safe_z_mm` the travel is at that height; without it the
    travel is at the higher of the two ends and goes around the solid volumes (planner/volumes.py). Either way every
    leg is checked against the claw and the links, not just the wrist."""
    vol = Volumes(layout, safety, holding, _CACHE)
    safe_z = layout.get("routes", {}).get("safe_z_mm")
    if safe_z is None and not vol.active:
        raise SafetyViolation("no taught route between these positions and routes.safe_z_mm is not set")
    if safe_z is None and vol.active:
        zone_route = _through_the_free_air(layout, safety, here, target, holding)
        if zone_route is not None:
            return zone_route
    if safe_z is None and vol.active and (_at_entry(vol, here) or _at_entry(vol, target)):
        return _entry_turns(layout, safety, vol, here, target, holding)
    x0, y0 = fk.MODEL["x0"], fk.MODEL["y0"]
    if safe_z is None and vol.active and math.dist(target[:2], (x0, y0)) < FOLD_MM:
        # the target is beside the arm's own axis: a straight move cannot end there. Plan to a point
        # UNFOLD_MM out, where the arm is unfolded, and fold in with one joint move that keeps the claw and links clear.
        # The claw keeps the target's yaw over the whole fold (both ends have it, the joints are interpolated). With a plate the
        # target handed in here is the entry point with the plate turned sideways (_entry_turns): the plate never points into the hotel
        # while the wrist travels, and the turn to the slot's yaw follows in place.
        toward = math.degrees(math.atan2(here[1] - y0, here[0] - x0))
        branch0 = vol.branch
        why = []                                   # every candidate's own reason: the last one alone misled a diagnosis (2026-11-09)
        for ang in dict.fromkeys([toward, 90.0, 135.0, 45.0, 180.0, 0.0, -45.0]):
            cand = [x0 + UNFOLD_MM * math.cos(math.radians(ang)), y0 + UNFOLD_MM * math.sin(math.radians(ang)), target[2]] \
                + list(target[3:])
            try:
                safety.check_target(cand, "fold")
                vol.check_pose(cand, "fold")
                route = lift_travel_lower(layout, safety, here, cand, holding)
                # the arm reaches `cand` on the branch it is on; the fold is a joint move from there to the target's joints
                arrive = jt.pose_joints(cand, branch0, None, vol.lim) if branch0 is not None else None
                end = vol.check_joint_leg(cand, target, "fold", joints_a=arrive, branch_b=branch0)
            except (SafetyViolation, ValueError) as e:
                why.append(f"from {ang:.0f} deg: {e}")
                continue
            route.poses.append(list(target))
            if vol.start_joints is not None:
                route.joint_targets[len(route.poses) - 1] = list(end)
            return route
        raise SafetyViolation("no way to fold the arm in to the pose beside its own axis, from any direction tried:\n  "
                              + "\n  ".join(why))
    withdrawn = None
    if vol.active:
        try:
            vol.settle(here)
        except SafetyViolation as why:      # the arm stands inside an obstacle (it touched it and stopped): back out first
            withdrawn = vol.withdraw(here, _HEADING)
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
        why = []                                   # every candidate's own reason, as the fold lists them
        carried = bool(holding and (vol.obstacles.get("hotel") or {}).get("solid"))
        for ang in dict.fromkeys([toward, 90.0, 135.0, 45.0, 180.0, 0.0, -45.0]):
            for yaw in dict.fromkeys([here[5], target[5], 0.0, 90.0, -90.0, 180.0]):
                if carried and yaw != here[5] and abs(fk.wrap180(yaw)) < 90.0:
                    continue                       # a carried plate is not turned toward the hotel while the arm unfolds
                for branch in branches:
                    cand = [x0 + UNFOLD_MM * math.cos(math.radians(ang)), y0 + UNFOLD_MM * math.sin(math.radians(ang)), here[2]] \
                        + list(here[3:5]) + [yaw]
                    try:
                        end = vol.check_joint_leg(here, cand, "unfold", start_actual=True, branch_b=branch)
                        safety.check_target(cand, "unfold")
                        rest = lift_travel_lower(layout, _Fixed(safety, end) if have else safety, cand, target, holding)
                    except (SafetyViolation, ValueError) as e:
                        first = str(e).split("\n")[0]
                        why.append(f"to {ang:.0f} deg, yaw {yaw:.0f}" + (f", branch {branch}" if branch else "") + f": {first}")
                        continue
                    route = Route("lift-travel-lower", [cand] + rest.poses, joint_targets=rest.shifted(1))
                    if have:
                        route.joint_targets[0] = list(end)
                    return route
        raise SafetyViolation("no way to unfold the arm from beside its own axis without the claw meeting the hotel or the "
                              "pillar, or leaving the joints' range, for any direction tried:\n  " + "\n  ".join(why))
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


_HEADING: Optional[List[float]] = None      # the vector of the move that last stopped the arm; a withdraw backs out against it first


def plan(layout: dict, safety: Safety, here: List[float], target: List[float],
         from_name: Optional[str] = None, to_name: Optional[str] = None, holding: Optional[str] = None,
         heading: Optional[List[float]] = None) -> Route:
    global _HEADING
    _HEADING = heading
    route = taught_route(layout, from_name, to_name)
    if route is not None:
        for i, p in enumerate(route.poses, 1):
            safety.check_target(p, f"{route.name}[{i}/{len(route.poses)}]")
        return route
    leg = None
    if GRAPH is not None and safety.joints:
        try:
            sig, branch = layout_sig(layout), fk.elbow_sign(list(safety.joints())[:4])
            a, b = GRAPH.name_of(sig, holding, here), GRAPH.name_of(sig, holding, target)
            if a and b and a != b:
                leg = GRAPH.key(leg_sig(layout, a, b), a, b, holding, branch)
                hit = GRAPH.get(leg)
                if hit is not None:                      # a leg between two stations, planned and checked before: no search
                    return Route(hit["name"], [list(q) for q in hit["poses"]],
                                 joint_targets={int(i): list(j) for i, j in hit["joint_targets"].items()})
        except Exception:
            leg = None
    key = _route_key(layout, safety, here, target, holding)
    if key in _ROUTES:                        # the same leg from the same place, planned and checked before in this run
        return copy.deepcopy(_ROUTES[key])
    global _CACHE
    _CACHE = {}
    try:
        route = _plan(layout, safety, here, target, holding)
    finally:
        _CACHE = None
    if key is not None:
        _ROUTES[key] = copy.deepcopy(route)
    if leg is not None and not route.taught:
        GRAPH.put(leg, route.poses, route.joint_targets, route.name)
    return route


_ROUTES: dict = {}
GRAPH = None                # planner.graph.StationGraph, set by the Cycle: legs between named stations are stored and replayed


GEOMETRY_KEYS = ("pose", "pose_at_0", "pose_at_90", "hover_mm", "clear_mm", "lift_mm", "lip_mm", "grid", "taught", "rows", "pose_source",
                 "trajectory", "place_lift_mm", "free_above_z", "hotel_approach")


def layout_sig(layout: dict) -> str:
    """A hash of the geometry a route depends on: the volumes, the regions, the routes block, and of every fixture only the keys that place
    or size it. Flags (a plate on the module), widths, the occupancy, the setup notes and the vision settings are left out: the first
    version of this hash held them, changed every move, and no stored leg ever matched (2026-11-02)."""
    fixtures = {name: {k: v for k, v in fx.items() if k in GEOMETRY_KEYS} for name, fx in layout.get("fixtures", {}).items() if isinstance(fx, dict)}
    geometry = [layout.get("volumes"), layout.get("regions"), layout.get("routes"), fixtures]
    return hashlib.sha1(json.dumps(geometry, sort_keys=True, default=str).encode()).hexdigest()


def leg_sig(layout: dict, a: str, b: str) -> str:
    """The geometry hash one stored leg depends on. A leg that touches no microscope station leaves every microscope out of the hash
    (its seat, its box, its region): re-teaching a microscope seat then drops only the legs to and from that microscope, and the hotel
    <-> module legs stay (operator, 2026-11-04). A microscope is any fixture with a pose other than the module and the hotel."""
    fixtures = layout.get("fixtures", {})
    scopes = [n for n, fx in fixtures.items() if isinstance(fx, dict) and "pose" in fx and n not in ("polar", "hotel")]
    drop = {s for s in scopes if s not in a and s not in b}
    if not drop:
        return layout_sig(layout)
    fx = {name: {k: v for k, v in f.items() if k in GEOMETRY_KEYS} for name, f in fixtures.items() if isinstance(f, dict) and name not in drop}
    volumes = dict(layout.get("volumes") or {})
    volumes["obstacles"] = {k: v for k, v in (volumes.get("obstacles") or {}).items() if k not in drop}
    regions = {k: v for k, v in (layout.get("regions") or {}).items() if k not in drop}
    return hashlib.sha1(json.dumps([volumes, regions, layout.get("routes"), fx], sort_keys=True, default=str).encode()).hexdigest()


def _route_key(layout, safety, here, target, holding):
    """What a planned route depends on: the start (to 1 mm, 1 deg, with the joints it was reached by), the target, what the claw
    holds, and the layout (every volume, region and pose; not the occupancy or the setup notes, which change every move).
    None when the joints cannot be read: then nothing is cached."""
    try:
        joints = tuple(round(v) for v in safety.joints()[:4]) if safety.joints else None
        if joints is None:
            return None
        sig = layout_sig(layout)
        return (sig, tuple(round(v) for v in here[:6]), joints, tuple(round(v, 2) for v in target[:6]), holding)
    except Exception:
        return None


def _plan(layout, safety, here, target, holding):
    return lift_travel_lower(layout, safety, here, target, holding)
