"""
The plate cycle.

    microscope (short-edge grip) -> polar stage at 0 deg -> rotate 90 -> long-edge grip
    -> hotel slot -> back to polar at 90 -> rotate 0 -> short-edge grip -> microscope

Every fixture visit follows the same pattern: travel, hover, vision check, lower, act on the jaw,
verify the jaw, lift.  The state file is updated before and after each step so the cycle can resume.
"""
from __future__ import annotations

import time
from typing import List, Optional

from .corrector import Corrector, StopSafe
from .polar import PolarError, PolarStage
from .predictor import DriftMonitor, PreChecks
from .robot import MotionRefused, Robot
from .safety import Safety, SafetyViolation
from .state import StateStore
from .telemetry import Telemetry
from .vision import VisionClient, VisionError
from .lower_check import LowerCheck


def _add(p: List[float], d: List[float]) -> List[float]:
    return [p[0] + d[0], p[1] + d[1], p[2] + d[2]] + list(p[3:])


class Cycle:
    def __init__(self, layout: dict, robot: Robot, vision: VisionClient, tele: Telemetry, store: StateStore,
                 mode: str, run_dir: str, ref_dir: str):
        self.L, self.robot, self.vision, self.tele, self.store = layout, robot, vision, tele, store
        self.supervised = mode == "supervised"
        self.safety = Safety(layout, mode)
        self.checks = PreChecks(layout)
        self.drift = DriftMonitor(run_dir + "/drift.json")
        self.corr = Corrector(layout, vision, robot, self.safety, tele, self.supervised)
        self.polar = PolarStage(layout["polar"], robot, tele.log, self.supervised)
        lc = layout["vision"]["lower_check"]
        self.lower = LowerCheck(ref_dir, lc["min_refs"], lc["margin"], lc["floor"])
        self.g = layout["gripper"]
        self.down_cam = layout["vision"]["down_camera"]
        self.front_cam = layout["vision"]["front_camera"]
        self._last_health = 0.0

    # ------------------------------------------------------------- primitives
    def _move(self, kind: str, target: List[float], label: str, speed: float, allow_floor=()):
        here = self.robot.pose()
        if kind == "L":
            self.safety.check_segment(here, target, label, allow_floor)
        else:
            self.safety.check_target(target, label, allow_floor)
        self.robot.set_speed(speed)
        t0 = time.time()
        try:
            (self.robot.move_linear if kind == "L" else self.robot.move_joint)(target)
        except MotionRefused as e:
            st = self.robot.state()
            if not st.enabled and self.corr.on_disabled(label):
                (self.robot.move_linear if kind == "L" else self.robot.move_joint)(target)
            else:
                self.store.fault(label, "motion_refused", str(e))
                self.corr.on_motion_refused(label, e)
        dt = time.time() - t0
        after = self.robot.pose()
        dev = max(abs(after[i] - target[i]) for i in range(3))
        self.tele.log("move", label, kind, target=[round(v, 2) for v in target], actual=[round(v, 2) for v in after],
                      dt=round(dt, 2), speed=speed, dev=round(dev, 2))
        if dev > self.L["limits"]["max_pose_deviation_mm"]:
            self.store.fault(label, "pose_deviation", f"{dev:.2f} mm")
            raise StopSafe(f"{label}: arm stopped {dev:.1f} mm from target; possible collision or stall")
        w = self.drift.observe(f"dt:{label}", dt)
        if w:
            self.corr.on_drift(w)
        self.store.s.last_safe_pose = after

    def _waypoints(self, name: str, reverse: bool = False):
        wp = self.L["trajectories"].get(name)
        if not wp:
            raise SafetyViolation(f"trajectory '{name}' is not taught; record it with run.py trace/normalise")
        pts = list(zip(wp["joints"], wp["poses"]))
        if reverse:
            pts.reverse()
        self.robot.set_speed(self.safety.profile.travel)
        for i, (j, p) in enumerate(pts, 1):
            self.safety.check_target(p, f"{name} waypoint {i}")
            t0 = time.time()
            try:
                self.robot.move_abs_joints(j)
            except MotionRefused as e:
                self.store.fault(name, "motion_refused", str(e))
                self.corr.on_motion_refused(f"{name} waypoint {i}", e)
            self.tele.log("move", f"{name}[{i}/{len(pts)}]", "AJ", dt=round(time.time() - t0, 2))

    def _jaw_open(self):
        self.robot.jaw_to(self.g["open"])
        time.sleep(1.0)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "open", jaw=round(jaw, 2))
        return jaw

    def _jaw_close(self) -> float:
        prof = self.safety.profile
        v = self.robot.jaw()
        curve = []
        while v > self.g["closed_on_plate"] + 1e-6:
            v = max(self.g["closed_on_plate"], v - prof.jaw_step)
            self.robot.jaw_to(v)
            time.sleep(prof.jaw_pause)
            curve.append(round(self.robot.jaw(), 3))
        time.sleep(0.5)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "close", jaw=round(jaw, 2), curve=curve)
        w = self.drift.observe("jaw:closed", jaw)
        if w:
            self.corr.on_drift(w)
        return jaw

    def _health(self):
        if time.time() - self._last_health < self.L["limits"]["health_check_every_s"]:
            return
        self._last_health = time.time()
        st = self.robot.state()
        vs = None
        try:
            vs = self.vision.status()
        except Exception as e:
            self.corr.alert("health", f"vision box unreachable: {e}")
            raise StopSafe("vision box unreachable")
        self.tele.log("health", "check", robot_enabled=st.enabled, drag=st.drag, vision=vs.state, clients=vs.clients)
        if st.drag:
            raise StopSafe("drag mode turned on during the run")

    # ------------------------------------------------------------ vision gates
    def _vision_correction(self, fixture: dict, label: str) -> List[float]:
        ref = fixture.get("aruco_reference")
        if not ref:
            return [0.0] * 6
        try:
            off = self.vision.pose_offset(self.down_cam, ref, self.L["vision"]["workspace"],
                                          fixture.get("aruco_marker_id", 1), None, self.L["vision"]["led"])
        except VisionError as e:
            self.tele.image(f"{label}_vision_error", self._safe_frame(self.down_cam))
            if self.corr.on_vision_error(label, e):
                return [0.0] * 6
            raise StopSafe(f"{label}: no vision correction available")
        self.tele.image(f"{label}_aruco", off.image)
        mag = (off.d[0] ** 2 + off.d[1] ** 2 + off.d[2] ** 2) ** 0.5
        self.tele.log("vision", f"{label} offset", d=[round(v, 2) for v in off.d], mag=round(mag, 2), frame=off.frame)
        w = self.drift.observe(f"vision_mag:{label}", mag)
        if w:
            self.corr.on_drift(w)
        if mag > self.safety.vision_jump_mm:
            self.corr.on_vision_too_far(label, mag)
        return [off.d[0], off.d[1], 0.0, 0.0, 0.0, 0.0]

    def _safe_frame(self, cam: str) -> Optional[bytes]:
        try:
            return self.vision.frame(cam, 1920, 1080, 80, None, self.L["vision"]["led"])
        except Exception:
            return None

    def _gate_lower(self, fixture_name: str, state: str, label: str):
        led = self.L["vision"]["led"]
        for attempt in range(self.L["vision"]["lower_check"]["retries"] + 1):
            img = self.vision.frame(self.down_cam, 1920, 1080, 85, None, led)
            v = self.lower.verdict(fixture_name, state, img)
            self.tele.image(f"{label}_lower_{attempt}", img)
            self.tele.log("vision", f"{label} lower_check", v.reason, score=round(v.best, 3), thr=round(v.threshold, 3),
                          refs=v.n_refs, safe=v.safe)
            w = self.drift.observe(f"lower:{fixture_name}:{state}", v.best)
            if w:
                self.corr.on_drift(w)
            if v.safe:
                return
            led = self.corr.on_lower_blocked(fixture_name, attempt, v)

    # ---------------------------------------------------------- fixture visits
    def _pick(self, fixture_name: str, pose: List[float], fixture: dict, grip: str, trajectory_in: Optional[str]):
        prof = self.safety.profile
        hover = _add(pose, [0, 0, fixture["hover_mm"]])
        step = f"pick:{fixture_name}"
        self.store.begin(step)
        self._health()
        if trajectory_in:
            self._waypoints(trajectory_in)
        self._move("J", hover, f"{step} hover", prof.approach)
        self._jaw_open()
        target = _add(pose, self._vision_correction(fixture, step))
        hover = _add(target, [0, 0, fixture["hover_mm"]])
        self._move("L", hover, f"{step} hover(corrected)", prof.approach)

        for attempt in range(2):
            self._gate_lower(fixture_name, "plate_present", step)
            problems = self.checks.before_grasp(self.robot.jaw(), self.robot.state(), None)
            if problems:
                raise StopSafe(f"{step}: " + "; ".join(problems))
            self._move("L", target, f"{step} lower", prof.contact, allow_floor=(fixture_name,))
            jaw = self._jaw_close()
            err = self.checks.grasp_ok(jaw)
            if not err:
                break
            self.store.fault(step, "grasp", err)
            self._jaw_open()
            self._move("L", hover, f"{step} lift(empty)", prof.approach)
            if not self.corr.on_grasp_failed(fixture_name, attempt + 1, err):
                raise StopSafe(err)
            target = _add(pose, self._vision_correction(fixture, step))
            hover = _add(target, [0, 0, fixture["hover_mm"]])

        self.store.done(plate_at="gripper", grip=grip)
        self._move("L", hover, f"{step} lift", prof.approach)
        self._check_plate_in_gripper(step, grip)

    def _place(self, fixture_name: str, pose: List[float], fixture: dict, at: str, trajectory_in: Optional[str],
               reverse: bool = False):
        prof = self.safety.profile
        hover = _add(pose, [0, 0, fixture["hover_mm"]])
        step = f"place:{fixture_name}"
        self.store.begin(step)
        self._health()
        if trajectory_in:
            self._waypoints(trajectory_in, reverse)
        self._move("J", hover, f"{step} hover", prof.approach)
        target = _add(pose, self._vision_correction(fixture, step))
        hover = _add(target, [0, 0, fixture["hover_mm"]])
        self._move("L", hover, f"{step} hover(corrected)", prof.approach)
        problems = self.checks.before_release(self.robot.jaw())
        if problems:
            raise StopSafe(f"{step}: " + "; ".join(problems))
        self._gate_lower(fixture_name, "empty", step)
        self._move("L", target, f"{step} lower", prof.contact, allow_floor=(fixture_name,))
        self._jaw_open()
        self.store.done(plate_at=at, grip="none")
        self._move("L", hover, f"{step} lift", prof.approach)
        img = self.vision.frame(self.down_cam, 1920, 1080, 85, None, self.L["vision"]["led"])
        self.tele.image(f"{step}_placed", img)
        v = self.lower.verdict(fixture_name, "plate_present", img)
        self.tele.log("vision", f"{step} seated_check", v.reason, score=round(v.best, 3), safe=v.safe)
        if not v.safe and v.n_refs >= self.lower.min_refs:
            if not self.corr.operator_or_stop(step, "plate does not look seated after release",
                                              "Plate is seated correctly?"):
                raise StopSafe("plate not seated")

    def _check_plate_in_gripper(self, step: str, grip: str):
        img = self._safe_frame(self.front_cam)
        self.tele.image(f"{step}_lifted", img)
        if img is None:
            return
        v = self.lower.verdict("gripper", grip, img)
        self.tele.log("vision", f"{step} plate_in_gripper", v.reason, score=round(v.best, 3), safe=v.safe, refs=v.n_refs)
        if v.n_refs < self.lower.min_refs:
            return
        w = self.drift.observe(f"gripper:{grip}", v.best)
        if w:
            self.corr.on_drift(w)
        if not v.safe and not self.corr.operator_or_stop(step, "plate in gripper does not match references",
                                                         "Plate is held correctly?"):
            raise StopSafe("plate not held correctly after lift")

    # ------------------------------------------------------------------ hotel
    def hotel_slot_pose(self, slot: str) -> List[float]:
        h = self.L["fixtures"]["hotel"]
        if slot in h["taught"]:
            return list(h["taught"][slot])
        a, b, n = h["row_vector_from"]
        rv = [(h["taught"][b][i] - h["taught"][a][i]) / n for i in range(3)]
        col, row = slot[0], int(slot[1:])
        base = h["taught"][col + "1"]
        return [base[i] + (row - 1) * rv[i] for i in range(3)] + list(base[3:])

    def _hotel_place(self, slot: str):
        prof = self.safety.profile
        h = self.L["fixtures"]["hotel"]
        p = self.hotel_slot_pose(slot)
        ax, mm, lift = h["approach"]["axis"], h["approach"]["mm"], h["lift_mm"]
        approach = _add(p, [ax[0] * mm, ax[1] * mm, lift])
        carry = _add(p, [0, 0, lift])
        step = f"place:hotel:{slot}"
        self.store.begin(step)
        self._health()
        self._waypoints("polar_to_hotel")
        self._move("J", approach, f"{step} approach", prof.approach)
        ready, img = self.vision.live_detection(self.front_cam, aruco=False, led=self.L["vision"]["led"])
        self.tele.image(f"{step}_slot_view", img)
        self.tele.log("vision", f"{step} slot_view", "object seen" if ready else "slot looks empty", ready=ready)
        if ready and not self.corr.operator_or_stop(step, f"{slot} may be occupied", "Slot is actually empty?"):
            raise StopSafe(f"{slot} occupied")
        if self.checks.before_release(self.robot.jaw()):
            raise StopSafe(f"{step}: not holding a plate")
        self._move("L", carry, f"{step} slide in", prof.contact)
        self._move("L", p, f"{step} lower", prof.contact)
        self._jaw_open()
        self.store.s.hotel[slot] = self.store.s.plate_id
        self.store.done(plate_at=f"hotel:{slot}", grip="none")
        self._move("L", carry, f"{step} lift off", prof.approach)
        self._move("L", approach, f"{step} slide out", prof.approach)

    def _hotel_pick(self, slot: str):
        prof = self.safety.profile
        h = self.L["fixtures"]["hotel"]
        p = self.hotel_slot_pose(slot)
        ax, mm, lift = h["approach"]["axis"], h["approach"]["mm"], h["lift_mm"]
        approach = _add(p, [ax[0] * mm, ax[1] * mm, lift])
        carry = _add(p, [0, 0, lift])
        step = f"pick:hotel:{slot}"
        self.store.begin(step)
        self._health()
        self._move("J", approach, f"{step} approach", prof.approach)
        self._jaw_open()
        self._move("L", _add(p, [ax[0] * mm, ax[1] * mm, 0]), f"{step} align", prof.approach)
        self._move("L", p, f"{step} slide in", prof.contact)
        jaw = self._jaw_close()
        err = self.checks.grasp_ok(jaw)
        if err:
            self.store.fault(step, "grasp", err)
            raise StopSafe(f"{step}: {err}")
        self.store.s.hotel.pop(slot, None)
        self.store.done(plate_at="gripper", grip="long_edges")
        self._move("L", carry, f"{step} lift", prof.approach)
        self._move("L", approach, f"{step} slide out", prof.approach)
        self._waypoints("polar_to_hotel", reverse=True)

    # --------------------------------------------------------------- the loop
    def _polar_rotate(self, deg: int):
        self.store.begin(f"polar:{deg}")
        clear = _add(self.robot.pose(), [0, 0, self.L["fixtures"]["polar"]["clear_mm"]])
        self._move("L", clear, "polar clear", self.safety.profile.approach)
        while True:
            try:
                self.polar.rotate_to(deg)
                break
            except PolarError as e:
                self.store.fault("polar", "rotate", str(e))
                self.corr.on_polar_failed(e)
        self.store.done(polar_angle=deg)

    def _scan_plate(self):
        try:
            codes, img = self.vision.scan(self.front_cam, 3.0, "QR;DATAMATRIX;CODE128", self.L["vision"]["led"], True)
        except VisionError:
            codes, img = [], None
        self.tele.image("plate_barcode", img)
        self.tele.log("vision", "barcode", ",".join(codes) or "none")
        self.store.s.plate_id = codes[0] if codes else ""

    def outbound(self, slot: str):
        F = self.L["fixtures"]
        self._pick("microscope", F["microscope"]["pose"], F["microscope"], "short_edges", None)
        self._place("polar", F["polar"]["pose_at_0"]["pose"], F["polar"], "polar", "microscope_to_polar")
        self._polar_rotate(90)
        self._pick("polar", F["polar"]["pose_at_90"]["pose"], F["polar"], "long_edges", None)
        self._scan_plate()
        self._hotel_place(slot)

    def inbound(self, slot: str):
        F = self.L["fixtures"]
        self._hotel_pick(slot)
        self._place("polar", F["polar"]["pose_at_90"]["pose"], F["polar"], "polar", None)
        self._polar_rotate(0)
        self._pick("polar", F["polar"]["pose_at_0"]["pose"], F["polar"], "short_edges", None)
        self._place("microscope", F["microscope"]["pose"], F["microscope"], "microscope", "microscope_to_polar",
                    reverse=True)

    def run(self, slot: str, cycles: int):
        s = self.store.s
        if not s.step_done:
            raise StopSafe(f"last step '{s.step}' did not finish; plate_at={s.plate_at} grip={s.grip}. "
                           "Inspect the cell, fix the state file, then rerun.")
        for _ in range(cycles):
            self.corr.new_cycle()
            s.cycle += 1
            self.tele.cycle = s.cycle
            self.store.save()
            t0 = time.time()
            if s.plate_at == "microscope":
                self.outbound(slot)
            if s.plate_at == f"hotel:{slot}":
                self.inbound(slot)
            self.drift.save()
            dt = time.time() - t0
            self.tele.log("cycle", "done", dt=round(dt, 1), state=s.plate_at)
            w = self.drift.observe("dt:cycle", dt)
            if w:
                self.corr.on_drift(w)
