"""
The plate cycle.

    microscope (short-edge grip) -> polar stage at 0 deg -> rotate 90 -> long-edge grip
    -> hotel slot -> back to polar at 90 -> rotate 0 -> short-edge grip -> microscope

Every fixture visit follows the same pattern: travel, hover, vision gate, lower, jaw action, jaw
verification, lift, held-plate check.  The state file is updated before and after each step.
Motion is never retried; every motion is observed to completion and compared with its target.
"""
from __future__ import annotations

import math
import os
import shutil
import time
from typing import List, Optional

from .corrector import Corrector, StopSafe
from .lower_check import LowerCheck, SlotCheck
from .polar import PolarError, PolarStage
from .predictor import DriftMonitor, PreChecks
from .robot import MotionRefused, Robot
from .safety import Safety, SafetyViolation
from .state import StateStore
from .telemetry import Telemetry
from .vision import VisionClient, VisionError


def _add(p: List[float], d: List[float]) -> List[float]:
    return [p[0] + d[0], p[1] + d[1], p[2] + d[2]] + list(p[3:])


class Cycle:
    def __init__(self, layout: dict, robot: Robot, vision: VisionClient, tele: Telemetry, store: StateStore,
                 mode: str, run_dir: str, ref_dir: str):
        self.L, self.robot, self.vision, self.tele, self.store = layout, robot, vision, tele, store
        self.run_dir = run_dir
        self.supervised = mode == "supervised"
        self.safety = Safety(layout, mode)
        self.checks = PreChecks(layout)
        self.drift = DriftMonitor(run_dir + "/drift.json")
        self.corr = Corrector(layout, vision, robot, self.safety, tele, self.supervised)
        self.polar = PolarStage(layout["polar"], robot, tele.log, self.supervised, store.s.polar_angle)
        lc = layout["vision"]["lower_check"]
        self.lower = LowerCheck(ref_dir, lc["min_refs"], lc["margin"], lc["floor"])
        self.slot_cfg = layout["vision"].get("slot_check", {})
        self.slots = SlotCheck(ref_dir, self.slot_cfg.get("min_refs", 5), 0.0, self.slot_cfg.get("min_match", 0.75))
        self.g = layout["gripper"]
        self.lim = layout["limits"]
        self.down_cam = layout["vision"]["down_camera"]
        self.front_cam = layout["vision"]["front_camera"]
        self.led = layout["vision"]["led"]
        self.front_led = int(layout["vision"].get("front_led", self.led))
        self._last_health = 0.0
        self.commission = {"no_gates": False, "no_vision_correction": False}
        self.resuming = False

    # --------------------------------------------------------------- recording
    def _fault(self, step: str, kind: str, detail: str):
        self.store.fault(step, kind, detail)
        self.tele.log("fault", step, kind, detail=detail)

    def _observe(self, feature: str, value: float):
        w = self.drift.observe(feature, value)
        if w:
            self.corr.on_drift(w)

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

    def _jaw(self, value: float, label: str):
        try:
            self.robot.jaw_to(value)
        except MotionRefused as e:
            self._fault(label, "jaw_refused", str(e))
            raise StopSafe(f"{label}: {e}")

    def _jaw_open(self, grip: str, fixture: Optional[dict] = None) -> float:
        """Opening width: the grip's default, or the fixture's `open_width[grip]` when it has one (the stage opens
        29 for the short edges since 2026-10-20: the plate's position at 0 after a robot place at 90 and the turn is
        not known to better than the old 1.8 mm per side, see docs/INCIDENTS.md; the jaw's end stop is 29.3)."""
        want = (fixture or {}).get("open_width", {}).get(grip) or self.g["open"][grip]
        self._jaw(want, "jaw open")
        self._wait_still("jaw open")
        time.sleep(0.5)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "open", jaw=round(jaw, 2), grip=grip)
        if jaw < want - 1.0:
            raise StopSafe(f"jaw did not open to {want} for {grip} ({jaw:.2f})")
        return jaw

    def _jaw_close(self, grip: str) -> float:
        """One close command from open to the taught value, as the vendor's own software does (2026-10-20).

        Measured with the controller's torque readout (lua/torque_probe) on the production plate: the holding torque
        is 70 % of rated (the gripper's configured cap) after a one-command close AND after the old 0.5 mm stepping,
        through the SDK or through Lua, whatever the target below the plate. So stepping never weakened the grip;
        one command is kept because it is the vendor path and takes 3 s instead of 10. A stall before the target is
        still not a fault here: the fingers met the plate (or something) at that width, and `grasp_ok` judges it.
        What differs between fixtures is where the pads land (1.62 to 1.67 on a free plate, 2.2 to 2.5 in the stage
        nest and the hotel), which is geometry, not force."""
        closed = self.g["closed"][grip]
        start = self.robot.jaw()
        stall = None
        try:
            self.robot.jaw_to(closed)
        except MotionRefused as e:
            stall = str(e)
        self._wait_still("jaw close")
        time.sleep(0.5)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "close", "stalled" if stall else "", jaw=round(jaw, 2), start=round(start, 2), grip=grip,
                      stall=stall)
        self._observe(f"jaw:closed:{grip}", jaw)
        return jaw

    def _center_search(self, step: str, fixture_name: str, fixture: dict, target: List[float], grip: str):
        """Tactile centering. The jaw stalled wider than the plate: one pad touched first because the seat pose is a
        few tenths of a millimetre off the plate along the closing axis (a hand-taught seat, a clamped holder).
        Walk the seat sideways in small steps, re-closing each time, until the fingers meet the plate at its true
        width. The offset that works is written back to the taught pose. Returns (jaw, target) on success, None
        otherwise. Lateral moves stay far inside the no-swing allowance and the fingers are open throughout."""
        cfg = fixture.get("grasp_search")
        if not cfg:
            return None
        # Judge against the fixture's learned signature when it has one (microscope 22.50), not the free-plate width:
        # on 2026-10-20 17:07 y+0.2 read 22.54 twice and was rejected against 22.40, then the attempt failed.
        closed = fixture.get("grasp_width", {}).get(grip) or self.g["closed"][grip]
        tol = self.checks.jaw_tol
        s, m = float(cfg.get("step_mm", 0.2)), float(cfg.get("max_mm", 0.6))
        sd, md = float(cfg.get("step_deg", 0.3)), float(cfg.get("max_deg", 0.6))
        trials = []                                     # (pose index, offset, unit)
        for axis in cfg.get("axes", ["x", "y"]):
            if axis == "yaw":
                k = 1
                while k * sd <= md + 1e-9:
                    for sign in (1, -1):
                        trials.append((5, sign * k * sd, "deg"))
                    k += 1
            elif axis == "z":
                for d in cfg.get("z_offsets_mm", [0.5, 1.0, -0.5]):
                    trials.append((2, float(d), "mm"))
            else:
                i = {"x": 0, "y": 1}[axis]
                k = 1
                while k * s <= m + 1e-9:
                    for sign in (1, -1):
                        trials.append((i, sign * k * s, "mm"))
                    k += 1
        names = {0: "x", 1: "y", 2: "z", 5: "yaw"}
        self._search_widths = []
        for i, d, unit in trials:
            cand = list(target)
            cand[i] += d
            label = f"{step} search {names[i]}{d:+.1f}{unit}"
            self._jaw_open(grip, fixture)
            here = self.robot.pose()
            cap = 0.4                                   # yaw changes inside a no-swing zone are capped per move
            n = int(abs(cand[5] - here[5]) / cap) + 1   # fixed number of parts, targets taken from `cand`, never from readback
            for k in range(1, n):
                mid = list(cand)
                mid[5] = here[5] + (cand[5] - here[5]) * k / n
                self._move("L", mid, label + f" (yaw part {k}/{n})", self.safety.profile.contact, allow_floor=(fixture_name,))
            self._move("L", cand, label, self.safety.profile.contact, allow_floor=(fixture_name,))
            jaw = self._jaw_close(grip)
            self._search_widths.append(jaw)
            self.tele.log("correct", f"{step} search", f"{names[i]}{d:+.1f}{unit} -> jaw {jaw:.2f}", jaw=round(jaw, 2),
                          axis=names[i], offset=d)
            if abs(jaw - closed) <= tol:
                self._persist_pose(fixture_name, fixture, cand,
                                   f"grasp search {names[i]}{d:+.1f}{unit} on {time.strftime('%Y-%m-%d %H:%M')}")
                return jaw, cand
        return None

    def _adopt_grasp_width(self, step: str, fixture_name: str, fixture: dict, grip: str) -> Optional[float]:
        """When the search finds the same width everywhere, the fixture itself presents the plate that wide (a holder
        that clamps it, a plate that bulges under the clips). If that width is consistent and only a little above the
        hand-measured grip, store it as this fixture's grasp signature and accept it. The held-plate camera check
        after the lift remains the guard against a false grasp."""
        cfg = fixture.get("grasp_search", {})
        ws = sorted(getattr(self, "_search_widths", []))
        closed = self.g["closed"][grip]
        if len(ws) < 6:
            return None
        med = ws[len(ws) // 2]
        spread = ws[-1] - ws[0]
        over = med - closed
        if spread > float(cfg.get("adopt_max_spread", 0.25)) or not (0 < over <= float(cfg.get("adopt_max_mm", 0.5))):
            self.tele.log("correct", f"{step} grasp width", f"not adopted: median {med:.2f}, spread {spread:.2f}")
            return None
        width = round(med, 2)
        fixture.setdefault("grasp_width", {})[grip] = width
        fixture.setdefault("corrections", []).append({"grasp_width": {grip: width}, "why": f"search widths {ws} on {time.strftime('%Y-%m-%d %H:%M')}"})
        self.tele.log("correct", f"{step} grasp width", f"adopted {width} for {grip} at {fixture_name} (nominal {closed})", widths=ws)
        self._persist_fixture(fixture_name, fixture, ("grasp_width", "corrections"))
        return width

    def _persist_fixture(self, fixture_name: str, fixture: dict, keys):
        path = getattr(self, "layout_path", None)
        if not path:
            return
        try:
            import json as _json
            with open(path, encoding="utf-8") as f:
                disk = _json.load(f)
            node = disk["fixtures"]
            for key in fixture_name.split("."):
                node = node[key]
            for k in keys:
                node[k] = fixture[k]
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                _json.dump(disk, f, indent=2)
            os.replace(tmp, path)
        except Exception as e:
            self.tele.log("correct", f"{fixture_name}", f"not persisted: {e}")

    def _persist_pose(self, fixture_name: str, fixture: dict, pose: List[float], why: str):
        fixture["pose"] = [round(v, 4) for v in pose]
        fixture.setdefault("corrections", []).append({"pose": fixture["pose"], "why": why})
        self.tele.log("correct", f"{fixture_name} pose", why, pose=fixture["pose"])
        path = getattr(self, "layout_path", None)
        if not path:
            return
        try:
            import json as _json
            with open(path, encoding="utf-8") as f:
                disk = _json.load(f)
            node = disk["fixtures"]
            for key in fixture_name.split("."):
                node = node[key]
            node["pose"] = fixture["pose"]
            node["corrections"] = fixture["corrections"]
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                _json.dump(disk, f, indent=2)
            os.replace(tmp, path)
        except Exception as e:
            self.tele.log("correct", f"{fixture_name} pose", f"not persisted: {e}")

    def _health(self, force: bool = False):
        if not force and time.time() - self._last_health < self.lim["health_check_every_s"]:
            return
        self._last_health = time.time()
        st = self.robot.state()
        vs = None
        deadline = time.time() + self.lim.get("vision_retry_s", 300)
        while vs is None:
            try:
                vs = self.vision.status()
            except Exception as e:
                if time.time() > deadline:
                    self.corr.alert("health", f"vision box unreachable for {self.lim.get('vision_retry_s', 300)} s: {e}")
                    raise StopSafe("vision box unreachable")
                self.tele.log("health", "vision", f"unreachable, retrying: {e}")
                time.sleep(10)
        ctrl_err = self._controller_errors()
        if ctrl_err:
            self.corr.alert("health", f"controller error: {ctrl_err}")
            raise StopSafe(f"controller error {ctrl_err}")
        free_gb = shutil.disk_usage(self.run_dir).free / 1e9
        self.tele.log("health", "check", robot_enabled=st.enabled, drag=st.drag, vision=vs.state, clients=vs.clients,
                      disk_free_gb=round(free_gb, 1), sdk_stops=getattr(self.robot, "sdk_stops", 0))
        if st.drag:
            raise StopSafe("drag mode turned on during the run")
        if not st.enabled:
            # between steps the arm is stationary; one re-enable per health check, counted by the corrector
            if not self.corr.on_disabled("health") or not self.robot.state().enabled:
                raise StopSafe("controller lost enable between steps and could not be re-enabled")
        if free_gb < self.lim["min_disk_free_gb"]:
            raise StopSafe(f"disk nearly full ({free_gb:.1f} GB free)")

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

    # ------------------------------------------------------------ vision gates
    def _frame(self, cam: str, led: Optional[int] = None) -> Optional[bytes]:
        # 1280x720: the camera's 1080p mode is a different sensor crop, and all gate references are 720p.
        # The front camera is lit by its own LED so day and night frames match (vision.front_led).
        if led is None:
            led = self.front_led if cam == self.front_cam else self.led
        try:
            return self.vision.frame(cam, 1280, 720, 85, None, led)
        except Exception as e:
            self.tele.log("vision", "frame", f"{cam}: {e}")
            return None

    def _vision_correction(self, fixture: dict, label: str) -> List[float]:
        ref = fixture.get("aruco_reference")
        if not ref or self.commission["no_vision_correction"]:
            if ref:
                self.tele.log("vision", f"{label} offset", "COMMISSIONING: vision correction disabled")
            return [0.0] * 6
        try:
            off = self.vision.pose_offset(self.down_cam, ref, self.L["vision"]["workspace"],
                                          fixture.get("aruco_marker_id", 1), None, self.led)
        except (VisionError, OSError) as e:
            self.tele.image(f"{label}_vision_error", self._frame(self.down_cam))
            if self.corr.on_vision_error(label, e):
                return [0.0] * 6
            raise StopSafe(f"{label}: no vision correction available")
        self.tele.image(f"{label}_aruco", off.image)
        mag = math.sqrt(off.d[0] ** 2 + off.d[1] ** 2 + off.d[2] ** 2)
        self.tele.log("vision", f"{label} offset", d=[round(v, 2) for v in off.d], mag=round(mag, 2), frame=off.frame)
        self._observe(f"vision_mag:{label}", mag)
        if mag > self.safety.vision_jump_mm:
            self.corr.on_vision_too_far(label, mag)
        return [off.d[0], off.d[1], 0.0, 0.0, 0.0, 0.0]

    def _gate_lower(self, gate: str, state: str, label: str):
        if self.commission["no_gates"]:
            img = self._frame(self.down_cam)
            self.tele.image(f"{label}_lower_nogate", img)
            self.tele.log("vision", f"{label} lower_check", "COMMISSIONING: gate disabled, image recorded only", gate=gate)
            return
        led = self.led
        attempts = self.L["vision"]["lower_check"]["retries"] + 1
        for attempt in range(attempts):
            img = self._frame(self.down_cam)
            if img is None:
                raise StopSafe(f"{label}: no frame from {self.down_cam} for the lower gate")
            v = self.lower.verdict(gate, state, img)
            self.tele.image(f"{label}_lower_{attempt}", img)
            self.tele.log("vision", f"{label} lower_check", v.reason, score=round(v.best, 3), thr=round(v.threshold, 3),
                          refs=v.n_refs, safe=v.safe, gate=gate)
            self._observe(f"lower:{gate}:{state}", v.best)
            if v.safe:
                return
            led = self.corr.on_lower_blocked(gate, attempt, attempts, v)
        raise StopSafe(f"{label}: lower gate exhausted {attempts} attempts without a safe verdict")

    def _check_plate_in_gripper(self, step: str, grip: str):
        img = self._frame(self.front_cam)
        self.tele.image(f"{step}_lifted", img)
        if img is None:
            if self.supervised or self.commission["no_gates"]:
                return
            raise StopSafe(f"{step}: front camera frame failed after lift")
        n = self.lower.count("gripper", grip)
        if n < int(self.L["vision"].get("held_plate_auto_refs", 12)):   # first lifts become the held-plate references
            self.lower.add_reference("gripper", grip, img)
            self.tele.log("vision", f"{step} plate_in_gripper", f"reference {n + 1} collected", grip=grip)
            return
        if self.commission["no_gates"]:
            v = self.lower.verdict("gripper", grip, img)
            self.tele.log("vision", f"{step} plate_in_gripper", v.reason, score=round(v.best, 3), safe=v.safe,
                          refs=v.n_refs, shadow=True)
            self._observe(f"gripper:{grip}", v.best)
            return
        v = self.lower.verdict("gripper", grip, img)
        self.tele.log("vision", f"{step} plate_in_gripper", v.reason, score=round(v.best, 3), safe=v.safe, refs=v.n_refs)
        if v.n_refs < self.lower.min_refs:
            return
        self._observe(f"gripper:{grip}", v.best)
        if not v.safe:
            self._fault(step, "not_held", v.reason)
            if not self.corr.operator_or_stop(step, "plate in gripper does not match references",
                                              "Plate is held correctly?"):
                raise StopSafe("plate not held correctly after lift")

    # ---------------------------------------------------------- fixture visits
    def _pick(self, fixture_name: str, pose: List[float], fixture: dict, grip: str, trajectory_in: Optional[str],
              gate: Optional[str] = None):
        prof = self.safety.profile
        gate = gate or fixture_name
        step = f"pick:{fixture_name}"
        self.store.begin(step)
        self._health()
        if trajectory_in:
            self._waypoints(trajectory_in)
        hover = _add(pose, [0, 0, fixture["hover_mm"]])
        self._arrive(hover, f"{step} hover", prof.approach)
        self._jaw_open(grip, fixture)
        corr = self._vision_correction(fixture, step)
        target = _add(pose, corr)
        hover = _add(target, [0, 0, fixture["hover_mm"]])
        if any(abs(v) > 0.05 for v in corr[:3]):
            self._move("L", hover, f"{step} hover(corrected)", prof.approach)

        for attempt in range(1, self.corr.budget.regrasp + 2):
            self._gate_lower(gate, "plate_present", step)
            problems = self.checks.before_grasp(self.robot.jaw(), self.robot.state())
            if problems:
                raise StopSafe(f"{step}: " + "; ".join(problems))
            self._descend(target, f"{step} lower", allow_floor=(fixture_name,))
            jaw = self._jaw_close(grip)
            expected = fixture.get("grasp_width", {}).get(grip)
            err = self.checks.grasp_ok(jaw, grip, expected)
            if err and jaw > self.g["closed"][grip]:          # wider than the plate: pads off-centre, not an empty grasp
                found = self._center_search(step, fixture_name, fixture, target, grip)
                if found:
                    jaw, target = found
                    hover = _add(target, [0, 0, fixture["hover_mm"]])
                    err = None
                else:
                    adopted = self._adopt_grasp_width(step, fixture_name, fixture, grip)
                    if adopted is not None:
                        self._jaw_open(grip, fixture)
                        self._move("L", target, f"{step} back to seat", self.safety.profile.contact, allow_floor=(fixture_name,))
                        jaw = self._jaw_close(grip)
                        err = self.checks.grasp_ok(jaw, grip, adopted)
            if not err:
                break
            self._fault(step, "grasp", err)
            self._jaw_open(grip, fixture)
            self._move("L", hover, f"{step} lift(empty)", prof.approach)
            self.corr.on_grasp_failed(fixture_name, attempt, err)
            target = _add(pose, self._vision_correction(fixture, step))
            hover = _add(target, [0, 0, fixture["hover_mm"]])
            self._move("L", hover, f"{step} hover(recorrected)", prof.approach)

        self._move("L", hover, f"{step} lift", prof.approach)
        self._check_plate_in_gripper(step, grip)
        self.store.done(plate_at="gripper", grip=grip)

    def _place(self, fixture_name: str, pose: List[float], fixture: dict, at: str, trajectory_in: Optional[str],
               reverse: bool = False, gate: Optional[str] = None):
        prof = self.safety.profile
        gate = gate or fixture_name
        step = f"place:{fixture_name}"
        self.store.begin(step)
        self._health()
        if trajectory_in:
            self._waypoints(trajectory_in, reverse)
        hover = _add(pose, [0, 0, fixture["hover_mm"]])
        self._arrive(hover, f"{step} hover", prof.approach)
        corr = self._vision_correction(fixture, step)
        target = _add(pose, corr)
        hover = _add(target, [0, 0, fixture["hover_mm"]])
        if any(abs(v) > 0.05 for v in corr[:3]):
            self._move("L", hover, f"{step} hover(corrected)", prof.approach)
        problems = self.checks.before_release(self.robot.jaw(), self.store.s.grip)
        if problems:
            raise StopSafe(f"{step}: " + "; ".join(problems))
        self._gate_lower(gate, "empty", step)
        self._descend(target, f"{step} lower", allow_floor=(fixture_name,))
        self._jaw_open(self.store.s.grip, fixture)
        self._seated_front(step, gate)
        self._move("L", hover, f"{step} lift", prof.approach)
        img = self._frame(self.down_cam)
        self.tele.image(f"{step}_placed", img)
        if img is None:
            raise StopSafe(f"{step}: no frame for the seated check")
        v = self.lower.verdict(gate, "plate_present", img)
        self.tele.log("vision", f"{step} seated_check", v.reason, score=round(v.best, 3), safe=v.safe, gate=gate)
        if not v.safe and v.n_refs >= self.lower.min_refs and not self.commission["no_gates"]:
            self._fault(step, "not_seated", v.reason)
            if not self.corr.operator_or_stop(step, "plate does not look seated after release",
                                              "Plate is seated correctly?"):
                raise StopSafe("plate not seated")
        self.store.done(plate_at=at, grip="none")

    def _seated_front(self, step: str, gate: str):
        """Seated check from the front camera at the release moment, fingers open, before lifting.
        Uses the bottom band of the frame, where the plate's top edge and the holder sit. The first
        `seated_front.auto_refs` releases at each gate become its references; until then, and while gates are
        off, the verdict is only logged (shadow mode)."""
        img = self._frame(self.front_cam)
        self.tele.image(f"{step}_released_front", img)
        self.tele.image(f"{step}_released_down", self._frame(self.down_cam))
        if img is None:
            return
        cfg = self.L["vision"].get("seated_front", {})
        roi = tuple(cfg.get("roi", [0.0, 0.55, 1.0, 1.0]))
        name = "seated_front"
        n = self.lower.count(name, gate)
        if n < cfg.get("auto_refs", 12):
            self.lower.add_reference(name, gate, img)
            self.tele.log("vision", f"{step} seated_front", f"reference {n + 1} collected", gate=gate)
            return
        v = self.lower.verdict(name, gate, img, roi=roi)
        self._observe(f"seated_front:{gate}", v.best)
        enforce = cfg.get("enforce", False) and not self.commission["no_gates"]
        self.tele.log("vision", f"{step} seated_front", v.reason, score=round(v.best, 3), thr=round(v.threshold, 3),
                      safe=v.safe, gate=gate, enforce=enforce)
        if enforce and not v.safe:
            self._fault(step, "not_seated_front", v.reason)
            if not self.corr.operator_or_stop(step, "front camera: plate does not look seated", "Plate is seated correctly?"):
                raise StopSafe("plate not seated (front camera)")

    def _slot_view(self, step: str, slot: str, expect: str):
        """Front-camera view of the slot from the approach point, with the verdict of the two-class check.
        The cell labels its own references: before a place the slot is empty, before a pick it is occupied.
        The first `min_refs` of each class are taken on consecutive visits; later ones every `spacing_s`, so the
        set spans the day's lighting. A frame that confidently contradicts its label is an alarm, never a lesson."""
        img = self._frame(self.front_cam)
        self.tele.image(f"{step}_slot_view", img)
        if img is None:
            return None
        sv = self.slots.verdict(slot, img)
        other = "occupied" if expect == "empty" else "empty"
        n, cap = self.slots.count(slot, expect), self.slot_cfg.get("auto_refs", 24)
        spaced = n < self.slots.min_refs or self.slots.newest_age_s(slot, expect) >= self.slot_cfg.get("spacing_s", 1800)
        if n < cap and spaced and sv.decision != other:
            self.slots.add_reference(slot, expect, img)
            self.tele.log("vision", f"{step} slot_ref", f"{expect} reference {n + 1} collected", slot=slot)
        return sv

    # ------------------------------------------------------------------ hotel
    def hotel_slot_pose(self, slot: str) -> List[float]:
        h = self.L["fixtures"]["hotel"]
        col, row = slot[:1], slot[1:]
        if col not in h["columns"] or not row.isdigit() or int(row) not in h["odd_rows"]:
            raise SafetyViolation(f"slot {slot} does not exist")
        if slot in h["taught"]:
            return list(h["taught"][slot])
        a, b, n = h["row_vector_from"]
        rv = [(h["taught"][b][i] - h["taught"][a][i]) / n for i in range(3)]
        base = h["taught"][col + "1"]
        return [base[i] + (int(row) - 1) * rv[i] for i in range(3)] + list(base[3:])

    def _hotel_geometry(self, slot: str):
        h = self.L["fixtures"]["hotel"]
        p = self.hotel_slot_pose(slot)
        ax, mm, lift = h["approach"]["axis"], h["approach"]["mm"], h["lift_mm"]
        approach = _add(p, [ax[0] * mm, ax[1] * mm, lift])
        align = _add(p, [ax[0] * mm, ax[1] * mm, 0])
        carry = _add(p, [0, 0, lift])
        return p, approach, align, carry

    def _hotel_junction(self, slot: str) -> List[float]:
        """Where route A ends: the approach point of the column's row-1 slot (taught and validated for A1)."""
        return self._hotel_geometry(slot[0] + "1")[1]

    def _hotel_climb(self, slot: str):
        """For rows above 1 the arm may not climb at the approach x: the plate already overlaps the shelf depth there
        (A3, 2026-10-15). It retreats to `climb_x`, a column in front of the rack the operator cleared, climbs there
        with the column axis only, and slides in at carry height. Returns (foot, top): the retreat point at the
        junction height and the point in front of the slot at carry height; both None for row 1."""
        if slot[1:] == "1":
            return None, None
        h = self.L["fixtures"]["hotel"]
        x = float(h["climb_x"])
        junction = self._hotel_junction(slot)
        _, approach, _, _ = self._hotel_geometry(slot)
        foot = [x, junction[1], junction[2]] + list(junction[3:])
        top = [x, approach[1], approach[2]] + list(approach[3:])
        return foot, top

    def _hotel_go_to_approach(self, slot: str, step: str):
        """Route A end -> junction -> (retreat, climb) -> approach point of the slot, all at carry height."""
        prof = self.safety.profile
        _, approach, _, _ = self._hotel_geometry(slot)
        foot, top = self._hotel_climb(slot)
        junction = self._hotel_junction(slot)
        here = self.robot.pose()
        if max(abs(here[i] - approach[i]) for i in range(3)) <= 5.0:     # resuming at the approach point
            self.resuming = False
            return
        if max(abs(here[i] - junction[i]) for i in range(3)) > 5.0:
            self._waypoints("polar_to_hotel")
            self._move("J", junction, f"{step} junction", prof.approach)
        if foot is not None:
            self._move("L", foot, f"{step} retreat", prof.approach)
            self._move("L", top, f"{step} climb", prof.approach)
            self._move("L", approach, f"{step} approach", prof.approach)

    def _hotel_leave_approach(self, slot: str, step: str):
        """Approach point -> (descend at the climb column) -> junction, ready for route A reversed."""
        prof = self.safety.profile
        foot, top = self._hotel_climb(slot)
        if foot is not None:
            self._move("L", top, f"{step} back to column", prof.approach)
            self._move("L", foot, f"{step} descend", prof.approach)
            self._move("L", self._hotel_junction(slot), f"{step} junction", prof.approach)

    def _hotel_place(self, slot: str):
        prof = self.safety.profile
        p, approach, _, carry = self._hotel_geometry(slot)
        step = f"place:hotel:{slot}"
        if self.store.s.hotel.get(slot) is not None:
            raise StopSafe(f"{step}: state says {slot} is occupied")
        self.store.begin(step)
        self._health()
        self._hotel_go_to_approach(slot, step)
        sv = self._slot_view(step, slot, "empty")
        try:
            ready, _ = self.vision.live_detection(self.front_cam, aruco=False, led=self.led)
        except Exception:
            ready = None
        self.tele.log("vision", f"{step} slot_view", sv.decision if sv else "no frame",
                      score_empty=round(sv.score_empty, 3) if sv else None,
                      score_occupied=round(sv.score_occupied, 3) if sv else None, vendor_live=ready)
        if sv and sv.decision == "occupied":
            self._fault(step, "slot_occupied", f"empty {sv.score_empty:.3f} vs occupied {sv.score_occupied:.3f}")
            if not self.corr.operator_or_stop(step, f"{slot} looks occupied", "Slot is actually empty?"):
                raise StopSafe(f"{slot} occupied")
        if sv is None or sv.decision == "unknown":
            if ready and not self.corr.operator_or_stop(step, f"{slot} may be occupied (vendor live detection)",
                                                        "Slot is actually empty?"):
                raise StopSafe(f"{slot} occupied")
        if self.checks.before_release(self.robot.jaw(), "long_edges"):
            raise StopSafe(f"{step}: not holding a plate on its long edges")
        self._slide(carry, f"{step} slide in")
        self._descend(p, f"{step} lower")
        self._jaw_open("long_edges")
        self._seated_front(step, f"hotel_{slot}")
        self._move("L", carry, f"{step} lift off", prof.approach)
        self._move("L", approach, f"{step} slide out", prof.approach)
        self.store.s.hotel[slot] = self.store.s.plate_id
        self.store.done(plate_at=f"hotel:{slot}", grip="none")
        self._hotel_leave_approach(slot, step)

    def _hotel_pick(self, slot: str):
        prof = self.safety.profile
        p, approach, align, carry = self._hotel_geometry(slot)
        step = f"pick:hotel:{slot}"
        self.store.begin(step)
        self._health()
        junction = self._hotel_junction(slot)
        here = self.robot.pose()
        if max(abs(here[i] - junction[i]) for i in range(3)) > 5.0:
            self._move("J", junction, f"{step} junction", prof.approach)
        foot, top = self._hotel_climb(slot)
        if foot is not None:
            self._move("L", foot, f"{step} retreat", prof.approach)
            self._move("L", top, f"{step} climb", prof.approach)
            self._move("L", approach, f"{step} approach", prof.approach)
        sv = self._slot_view(step, slot, "occupied")
        if sv:
            self.tele.log("vision", f"{step} slot_view", sv.decision, score_empty=round(sv.score_empty, 3),
                          score_occupied=round(sv.score_occupied, 3))
        if sv and sv.decision == "empty":
            self._fault(step, "slot_empty", f"empty {sv.score_empty:.3f} vs occupied {sv.score_occupied:.3f}")
            if self.slot_cfg.get("enforce_pick", False) and not self.commission["no_gates"]:
                if not self.corr.operator_or_stop(step, f"{slot} looks empty", "Plate is in the slot?"):
                    raise StopSafe(f"{slot} looks empty")
        self._jaw_open("long_edges")
        self._slide(carry, f"{step} slide in")                        # same path as the place, reversed
        self._descend(p, f"{step} lower onto plate")
        jaw = self._jaw_close("long_edges")
        err = self.checks.grasp_ok(jaw, "long_edges")
        if err:
            self._fault(step, "grasp", err)
            raise StopSafe(f"{step}: {err}")
        self._move("L", carry, f"{step} lift", prof.approach)
        self._move("L", approach, f"{step} slide out", prof.approach)
        self._check_plate_in_gripper(step, "long_edges")
        expected = self.store.s.hotel.get(slot, "")
        if expected and self.store.s.plate_id and expected != self.store.s.plate_id:
            self.corr.alert(step, f"plate id mismatch: slot had {expected}, scanned {self.store.s.plate_id}")
        self.store.s.hotel.pop(slot, None)
        self.store.done(plate_at="gripper", grip="long_edges")
        self._hotel_leave_approach(slot, step)
        self._waypoints("polar_to_hotel", reverse=True)

    # ------------------------------------------------------------------ polar
    def _polar_rotate(self, deg: int, from_pose: List[float]):
        F = self.L["fixtures"]["polar"]
        self.store.begin(f"polar:{deg}")
        clear = _add(from_pose, [0, 0, F["hover_mm"] + F["clear_mm"]])
        self._move("L", clear, "polar clear", self.safety.profile.approach)
        while True:
            try:
                self.polar.rotate_to(deg)
                if self.polar.driver == "ext_axis":
                    err = self.polar.read_angle() - (90 if deg == 90 else 0)
                    self.tele.log("polar", "readback", deg=deg, err_deg=round(err, 3))
                    self._observe(f"polar_err:{deg}", err)
                break
            except PolarError as e:
                self._fault("polar", "rotate", str(e))
                self.corr.on_polar_failed(e)
        self.store.done(polar_angle=deg)

    def _scan_plate(self):
        try:
            codes, img = self.vision.scan(self.front_cam, 3.0, "QR;DATAMATRIX;CODE128", self.led, True)
        except (VisionError, OSError):
            codes, img = [], None
        self.tele.image("plate_barcode", img)
        self.tele.log("vision", "barcode", ",".join(codes) or "none")
        self.store.s.plate_id = codes[0] if codes else ""

    # --------------------------------------------------------------- the loop
    def _steps(self, slot: str):
        F = self.L["fixtures"]
        P0, P90 = F["polar"]["pose_at_0"]["pose"], F["polar"]["pose_at_90"]["pose"]
        out = [
            ("pick:microscope", lambda: self._pick("microscope", F["microscope"]["pose"], F["microscope"], "short_edges", None)),
            ("place:polar", lambda: self._place("polar", P0, F["polar"], "polar", "microscope_to_polar", gate="polar@0")),
            ("polar:90", lambda: self._polar_rotate(90, P0)),
            ("pick:polar", lambda: self._pick("polar", P90, F["polar"], "long_edges", None, gate="polar@90")),
            ("scan", self._scan_plate),
            (f"place:hotel:{slot}", lambda: self._hotel_place(slot)),
        ]
        back = [
            (f"pick:hotel:{slot}", lambda: self._hotel_pick(slot)),
            ("place:polar", lambda: self._place("polar", P90, F["polar"], "polar", None, gate="polar@90")),
            ("polar:0", lambda: self._polar_rotate(0, P90)),
            ("pick:polar", lambda: self._pick("polar", P0, F["polar"], "short_edges", None, gate="polar@0")),
            ("place:microscope", lambda: self._place("microscope", F["microscope"]["pose"], F["microscope"], "microscope",
                                                     "microscope_to_polar", reverse=True)),
        ]
        return out, back

    def _run_steps(self, steps, from_step: Optional[str] = None):
        started = from_step is None
        for name, fn in steps:
            if not started:
                if name == from_step:
                    started = True
                else:
                    continue
            fn()
            self.resuming = False          # only the first route after a resume may start mid-way

    def outbound(self, slot: str, from_step: Optional[str] = None):
        self._run_steps(self._steps(slot)[0], from_step)

    def inbound(self, slot: str, from_step: Optional[str] = None):
        self._run_steps(self._steps(slot)[1], from_step)

    def _check_resumable(self, slot: str, resume: bool = False):
        s = self.store.s
        if resume and not s.step_done and s.plate_at == "gripper" and s.step.startswith("place:"):
            return                                   # a carry was interrupted on a route; continue that step
        if resume and not s.step_done and s.grip == "none" and s.step.startswith("pick:"):
            return                                   # a pick was interrupted before the grasp; the step restarts
        if not s.step_done:
            raise StopSafe(f"last step '{s.step}' did not finish; plate_at={s.plate_at} grip={s.grip}. "
                           "Inspect the cell, fix state.json, or rerun with --resume if the arm is still on its route.")
        if s.plate_at not in ("microscope", f"hotel:{slot}"):
            raise StopSafe(f"plate_at={s.plate_at}: no automatic resume from here; move the plate by hand "
                           "and edit state.json")
        if s.last_safe_pose:
            here = self.robot.pose()
            far = max(abs(here[i] - s.last_safe_pose[i]) for i in range(3))
            if far > self.lim["max_restart_offset_mm"]:
                raise StopSafe(f"arm is {far:.0f} mm from where the last run left it; jog it back before restarting")

    def _sync_stage(self):
        """The stage is unpowered in drag mode and gets turned by hand: trust the readback, not the state file."""
        if self.polar.driver != "ext_axis":
            return
        a = self.polar.read_angle()
        for deg in (0, 90):
            if abs(a - deg) <= 3.0:
                if self.store.s.polar_angle != deg:
                    self.tele.log("polar", "sync", f"state said {self.store.s.polar_angle}, stage reads {a:.2f}")
                self.store.s.polar_angle = self.polar.angle = deg
                self.store.save()
                if abs(a - deg) > 0.3:                   # settled off its mark while unpowered: re-drive it (the arm is at a seat, not above the stage)
                    self.tele.log("polar", "sync", f"re-driving the stage from {a:.2f} to {deg}")
                    self.polar.rotate_to(deg)
                return
        raise StopSafe(f"rotating stage reads {a:.2f} deg, neither 0 nor 90; run.py polar goto 0 first")

    def _choose_slot(self, slot: str) -> str:
        """`random` draws from fixtures.hotel.active_slots, never a slot the state says is occupied."""
        if slot != "random":
            return slot
        h = self.L["fixtures"]["hotel"]
        free = [x for x in h.get("active_slots", ["A1"]) if self.store.s.hotel.get(x) is None]
        if not free:
            raise StopSafe("no free active slot")
        import random
        pick = random.choice(free)
        self.tele.log("run", "slot", f"random draw -> {pick}", free=free)
        return pick

    def _current_hotel_slot(self) -> Optional[str]:
        at = self.store.s.plate_at
        return at.split(":", 1)[1] if at.startswith("hotel:") else None

    def run(self, slot: str, cycles: int, resume: bool = False, stop_file: Optional[str] = None):
        s = self.store.s
        if slot != "random":
            self.hotel_slot_pose(slot)
        step_slot = s.step.split(":")[-1] if s.step.startswith(("place:hotel:", "pick:hotel:")) else None
        self._check_resumable(self._current_hotel_slot() or step_slot or (slot if slot != "random" else "A1"), resume)
        self._health(force=True)
        self._sync_stage()
        for _ in range(cycles):
            if stop_file and os.path.exists(stop_file):
                self.tele.log("run", "stop_file", f"{stop_file} present; ending between cycles")
                break
            self.corr.new_cycle()
            t0 = time.time()
            if resume and not s.step_done and s.grip == "none" and s.step.startswith("pick:"):
                self.tele.cycle = s.cycle
                self.tele.log("run", "resume", f"restarting step {s.step} (gripper empty)")
                cur = step_slot or self._choose_slot(slot)
                out, back = self._steps(cur)
                table = out if any(n == s.step for n, _ in out) else back
                self._run_steps(table, from_step=s.step)
                resume = False
            elif resume and not s.step_done and s.plate_at == "gripper":
                self.resuming = True
                self.tele.cycle = s.cycle
                self.tele.log("run", "resume", f"continuing step {s.step} with grip {s.grip}")
                cur = step_slot or self._choose_slot(slot)
                if s.grip == "short_edges" and s.step == "place:polar":
                    self.outbound(cur, from_step="place:polar")
                elif s.step.startswith("place:hotel"):
                    self.outbound(cur, from_step=s.step)
                elif s.step in ("place:polar", "place:microscope") and s.grip == "long_edges":
                    self.inbound(cur, from_step=s.step)
                elif s.step == "place:microscope":
                    self.inbound(cur, from_step="place:microscope")
                resume = False
            else:
                s.cycle += 1
                self.tele.cycle = s.cycle
                self.store.save()
                if s.plate_at == "microscope":
                    self.outbound(self._choose_slot(slot))
            cur = self._current_hotel_slot()
            if cur:
                self.inbound(cur)
            self.drift.save()
            dt = time.time() - t0
            self.tele.log("cycle", "done", dt=round(dt, 1), state=s.plate_at)
            self._observe("dt:cycle", dt)
