"""The arm moves, each checked against the speed profile, the workspace box and the Z floors. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

import math
import time
from typing import List, Optional

from devices.arm import MotionRefused
from procedures.corrector import StopSafe
from procedures.safety import SafetyViolation
from routes.plan import plan
from routes_opt import fk


def offset(p: List[float], d: List[float]) -> List[float]:
    """pose p moved by d in x, y, z."""
    return [p[0] + d[0], p[1] + d[1], p[2] + d[2]] + list(p[3:])


class Motion:
    """The arm moves, each checked against the speed profile, the workspace box and the Z floors."""

    # -------------------------------------------------------------- primitives
    def _wait_still(self, label: str):
        t0 = self.clock.time()
        while self.robot.state().moving:
            if self.clock.time() - t0 > self.lim["step_timeout_s"]:
                raise StopSafe(f"{label}: still moving after {self.lim['step_timeout_s']} s")
            self.clock.sleep(0.05)

    def _settle(self, label: str, target_xyz: List[float]) -> float:
        self._wait_still(label)
        after = self.robot.pose()
        dev = max(abs(after[i] - target_xyz[i]) for i in range(3))
        if dev > self.lim["max_pose_deviation_mm"]:
            self._fault(label, "pose_deviation", f"{dev:.2f} mm")
            raise StopSafe(f"{label}: arm stopped {dev:.1f} mm from target; possible collision or stall")
        if len(target_xyz) > 5:                       # a refused yaw is a refused move too (soft limit, collision)
            dyaw = abs((after[5] - target_xyz[5] + 180.0) % 360.0 - 180.0)
            if dyaw > self.lim.get("max_yaw_deviation_deg", 0.5):
                self._fault(label, "yaw_deviation", f"{dyaw:.2f} deg")
                raise StopSafe(f"{label}: yaw ended {dyaw:.1f} deg from target; wrist limit or contact")
        self.store.s.last_safe_pose = after
        return dev

    def _descend(self, target: List[float], label: str, allow_floor=()):
        """Lower onto a seat: approach speed until slow_zone_mm above it, contact speed for the rest."""
        prof = self.safety.profile
        if self.ask_before_lower and not self.ask_before_lower(label):
            raise StopSafe(f"{label}: the operator said no")
        here = self.robot.pose()
        slow = self.lim.get("slow_zone_mm", 10.0)
        if here[2] - target[2] > slow + 1.0:
            mid = list(target)
            mid[2] = target[2] + slow
            self._move("L", mid, f"{label} (fast part)", prof.approach, allow_floor)
        self._move("L", target, label, prof.contact, allow_floor)

    def _slide(self, target: List[float], label: str, allow_floor=()):
        """Horizontal slide into a rack: approach speed until slow_zone_mm before the end, then contact speed."""
        prof = self.safety.profile
        here = self.robot.pose()
        slow = self.lim.get("slow_zone_mm", 10.0)
        d = [target[i] - here[i] for i in range(3)]
        dist = math.sqrt(sum(v * v for v in d))
        if dist > slow + 1.0:
            f = (dist - slow) / dist
            mid = [here[i] + d[i] * f for i in range(3)] + list(target[3:])
            self._move("L", mid, f"{label} (fast part)", prof.approach, allow_floor)
        self._move("L", target, label, prof.contact, allow_floor)

    def _check_parked(self, p: List[float], label: str):
        """A region flagged needs_parked (the microscope) is entered only while its stage reports parked."""
        r = self.safety.region_of(p)
        if self.safety.regions.get(r, {}).get("needs_parked") and not self.parked():
            raise StopSafe(f"{label}: the {r} stage is not parked; the arm stays out of the {r} zone")

    def _move(self, kind: str, target: List[float], label: str, speed: float, allow_floor=()):
        here = self.robot.pose()
        if kind == "L":
            self.safety.check_segment(here, target, label, allow_floor)
        else:
            self.safety.check_target(target, label, allow_floor)
            self.safety.check_joint_move(here, target, label)
        self._check_parked(target, label)
        self.robot.set_speed(speed)
        t0 = time.time()
        try:
            (self.robot.move_linear if kind == "L" else self.robot.move_joint)(target)
        except MotionRefused as e:
            self._refused(label, e)
        dev = self._settle(label, target)
        dt = time.time() - t0
        self.tele.log("move", label, kind, target=[round(v, 2) for v in target],
                      actual=[round(v, 2) for v in self.robot.pose()], dt=round(dt, 2), speed=speed, dev=round(dev, 2))
        self._observe(f"dt:{label}", dt)

    def _refused(self, label: str, e: Exception):
        """A move that did not reach its target. Ask the controller why before naming the fault: on 2026-10-20 22:25 a
        wrist collision (3873) surfaced only as 'abs joint move did not reach', the supervisor read that as
        recoverable and would have relaunched into a cell with the plate somewhere unknown."""
        errs = self.robot.errors()
        if errs:
            self._fault(label, "controller_error", f"{errs} after: {e}")
            self.corr.alert("collision" if "3873" in str(errs) else "controller", f"{label}: {errs}")
            raise StopSafe(f"{label}: controller error {errs} (possible collision); move stopped: {e}")
        self._fault(label, "motion_refused", str(e))
        if not self.robot.state().enabled:
            self.corr.on_disabled(label)
        raise StopSafe(f"{label}: {e}")

    def _arrive(self, hover: List[float], label: str, speed: float):
        """Reach a fixture hover. In a no-swing zone the trajectory must already end there: no joint move."""
        here = self.robot.pose()
        lateral = max(abs(here[0] - hover[0]), abs(here[1] - hover[1]))
        allowed = self.safety.regions.get(self.safety.region_of(hover), {}).get("max_lateral_mm", 2.0)    # the zone's own limit
        if lateral <= allowed:
            self._move("L", hover, label, speed)        # vertical: allowed anywhere
            return
        if self.safety.regions.get(self.safety.region_of(hover), {}).get("no_swing"):
            raise SafetyViolation(f"{label}: trajectory must end at the hover inside a no-swing zone "
                                  f"(arm is {max(abs(here[i] - hover[i]) for i in range(3)):.0f} mm away)")
        self._move("J", hover, label, speed)

    def _waypoints(self, name: str, reverse: bool = False):
        wp = self.L["trajectories"].get(name)
        if not wp:
            raise SafetyViolation(f"trajectory '{name}' is not taught")
        pts = list(zip(wp["joints"], wp["poses"]))
        if reverse:
            pts.reverse()
        start = 0
        if self.resuming:
            here = self.robot.joints()[:4]
            dists = [max(abs(a - b) for a, b in zip(here, j[:4])) for j, _ in pts]
            start = int(min(range(len(pts)), key=lambda k: dists[k]))
            if dists[start] > self.lim["max_waypoint_joint_step_deg"]:
                raise StopSafe(f"{name}: arm is {dists[start]:.1f} deg from the nearest waypoint; not on the route")
            self.tele.log("move", name, f"resuming at waypoint {start + 1}/{len(pts)}")
            self.resuming = False
        self.robot.set_speed(self.safety.profile.travel)
        tol = self.lim.get("max_waypoint_pose_dev_mm", 6.0)
        prev = self.robot.joints()[:4]
        for i, (j, p) in enumerate(pts[start:], start + 1):
            label = f"{name}[{i}/{len(pts)}]"
            self.safety.check_target(p, label)
            self.safety.check_joint_path(prev, list(j[:4]), label)       # the swing between waypoints, elbow included
            self._check_parked(p, label)
            prev = list(j[:4])
            ext = self.robot.joints()[4:]
            t0 = time.time()
            try:
                self.robot.move_abs_joints_stepped(list(j[:4]) + list(ext))     # external axes are never driven here
            except MotionRefused as e:
                self._refused(label, e)
            self._wait_still(label)
            after = self.robot.pose()
            dev = max(abs(after[k] - p[k]) for k in range(3))
            if dev > tol:
                self._fault(label, "pose_deviation", f"{dev:.2f} mm")
                raise StopSafe(f"{label}: arm {dev:.1f} mm from the demo pose (limit {tol}); joints reached target, check the route")
            self.store.s.last_safe_pose = after
            self.tele.log("move", label, "AJ", dt=round(time.time() - t0, 2), dev=round(dev, 2))

    def _seat(self, pose: List[float], corr: List[float]) -> List[float]:
        """The pose the jaw acts at: taught seat + vision correction + the plate type's grab height
        (the vendor template adds VAR7 to the seat Z the same way)."""
        return offset(offset(pose, corr), [0.0, 0.0, self.plate.grab_height_mm])

    # ------------------------------------------------------------- go somewhere
    def _position(self, name: str) -> List[float]:
        """The taught pose of a position (microscope, polar.pose_at_0, ...), at its hover height."""
        node, hover = self.L["fixtures"], 0.0
        for part in name.split("."):
            node = node[part]
            hover = node.get("hover_mm", hover)
        pose = node.get("pose")
        if not pose:
            raise SafetyViolation(f"{name}: no taught pose")
        return offset(list(pose), [0.0, 0.0, hover])

    def _position_here(self, tol_mm: float = 5.0) -> Optional[str]:
        """The position the arm is at now, if any; names the taught route to use."""
        here = self.robot.pose()
        for name in self.L.get("routes", {}).get("positions", []):
            try:
                p = self._position(name)
            except (KeyError, SafetyViolation):
                continue
            if max(abs(here[i] - p[i]) for i in range(3)) <= tol_mm:
                return name
        return None

    def move_to(self, name: str) -> None:
        """Go to a position's hover: a taught route when one exists, else lift, travel at the safe height, lower.
        Never to the seat itself; the seat is the job of pick and place, behind their gates."""
        self.go_to(self._position(name), name)

    def _move_joints_to(self, target: List[float], label: str, speed: float, joints: Optional[List[float]] = None) -> None:
        """A joint move to a pose, by joint values on the elbow branch the arm is on now (the branch the planner checked). A joint
        move to a Cartesian pose lets the controller pick its own branch: on 2026-10-28 it chose the mirror one and swung the
        arm's body into the pillar."""
        ja = list(self.robot.joints())
        branch = fk.elbow_sign(ja)
        if joints:                                    # the pose's own joints (the nest): both elbow branches meet there
            jt = list(joints[:4])
        else:
            jt = fk.ik(target, elbow=branch, j2_ref=ja[1], j3_ref=ja[2], j4_ref=ja[3])
            if fk.elbow_sign(jt) != branch:
                raise SafetyViolation(f"{label}: the target needs the other elbow branch")
        self.safety.check_target(target, label)
        self.safety.check_joint_path(ja[:4], list(jt[:4]), label)
        self._check_parked(target, label)
        self.robot.set_speed(speed)
        t0 = time.time()
        try:
            self.robot.move_abs_joints_stepped(list(jt[:4]) + list(ja[4:]))
        except MotionRefused as e:
            self._refused(label, e)
        self._wait_still(label)
        after = self.robot.pose()
        dev = max(abs(after[k] - target[k]) for k in range(3))
        self.tele.log("move", label, "AJ", target=[round(v, 2) for v in target], actual=[round(v, 2) for v in after],
                      dt=round(time.time() - t0, 2), dev=round(dev, 2))
        if dev > 3.0:
            raise StopSafe(f"{label}: arm {dev:.1f} mm from the target after a joint move")
        self.store.s.last_safe_pose = after

    def _enter(self, trajectory: Optional[str], reverse: bool, hover: List[float], fixture_name: str) -> None:
        """The taught route to a fixture's hover when the layout has one; otherwise the planner's way to the hover (a layout built
        from the CAD alone has no taught route: bringup/scratch.py)."""
        if trajectory and trajectory in self.L["trajectories"]:
            self._waypoints(trajectory, reverse)
        else:
            self.go_to(hover, f"{fixture_name} hover")

    def go_nest(self) -> None:
        """The nest: routes.nest.pose, a pose in the clear in front of the hotel (the front camera sees it) that the planner can
        reach from anywhere, including from a pose where the arm stopped against something (it backs out along the claw first).
        For an empty claw: a plate in it would reach into the hotel from there."""
        nest = self.L.get("routes", {}).get("nest")
        if not nest:
            raise SafetyViolation("routes.nest is not set")
        grip = getattr(getattr(self, "store", None), "s", None) and self.store.s.grip
        if grip and grip != "none":
            raise SafetyViolation("the claw holds a plate; the nest is for an empty claw. Put the plate down by hand, then go to the nest")
        here = self._position_here()
        if here and not here.startswith("polar"):          # at a microscope hover: the taught route out of its zone first
            self.move_to("polar.pose_at_0")
        self.go_to(list(nest["pose"]), "nest")

    def go_to(self, target: List[float], name: str) -> None:
        """move_to for a pose that is not a taught position (the setup script's computed views)."""
        grip = getattr(getattr(self, "store", None), "s", None) and self.store.s.grip
        route = plan(self.L, self.safety, self.robot.pose(), target, self._position_here(), name,
                     holding=grip if grip and grip != "none" else None)
        self.tele.log("move", f"move_to {name}", route.name, taught=route.taught, waypoints=len(route.poses))
        if route.taught:
            self._waypoints(route.name, route.reverse)
            self._arrive(target, f"move_to {name} arrive", self.safety.profile.approach)
            return
        for i, p in enumerate(route.poses, 1):
            label = f"move_to {name} [{i}/{len(route.poses)}]"
            if (i == 1 and route.first_joint) or (i == len(route.poses) and route.last_joint):
                self._move_joints_to(p, label, self.safety.profile.travel,
                                     joints=route.last_joints if i == len(route.poses) and route.last_joint else None)
            else:
                self._move("L", p, label, self.safety.profile.travel)
