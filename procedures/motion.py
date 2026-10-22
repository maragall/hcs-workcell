"""The arm moves, each checked against the speed profile, the workspace box and the Z floors. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

import math
import time
from typing import List

from devices.arm import MotionRefused
from procedures.corrector import StopSafe
from procedures.safety import SafetyViolation


def offset(p: List[float], d: List[float]) -> List[float]:
    """pose p moved by d in x, y, z."""
    return [p[0] + d[0], p[1] + d[1], p[2] + d[2]] + list(p[3:])


class Motion:
    """The arm moves, each checked against the speed profile, the workspace box and the Z floors."""

    # -------------------------------------------------------------- primitives
    def _wait_still(self, label: str):
        t0 = time.time()
        while self.robot.state().moving:
            if time.time() - t0 > self.lim["step_timeout_s"]:
                raise StopSafe(f"{label}: still moving after {self.lim['step_timeout_s']} s")
            time.sleep(0.05)

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

    def _move(self, kind: str, target: List[float], label: str, speed: float, allow_floor=()):
        here = self.robot.pose()
        if kind == "L":
            self.safety.check_segment(here, target, label, allow_floor)
        else:
            self.safety.check_target(target, label, allow_floor)
            self.safety.check_joint_move(here, target, label)
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
        errs = self._controller_errors()
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
        if lateral <= 2.0:
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
        for i, (j, p) in enumerate(pts[start:], start + 1):
            label = f"{name}[{i}/{len(pts)}]"
            self.safety.check_target(p, label)
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

    def _controller_errors(self):
        """Controller error list over the documented protocol; empty when unreachable (the SDK path still guards)."""
        try:
            import json as _json
            import socket as _socket
            s = _socket.create_connection((self.L["robot"]["ip"], 56788), timeout=3)
            s.settimeout(3)
            s.sendall((_json.dumps({"BRCAPI": "1.0", "cmd": "getRobotErr", "id": 7}) + chr(10)).encode())
            buf = b""
            while not buf.endswith(b"\n"):
                c = s.recv(65536)
                if not c:
                    break
                buf += c
            s.close()
            ret = _json.loads(buf).get("ret") or []
            return [e for e in ret if e.get("errCode") is not None]
        except Exception:
            return []
