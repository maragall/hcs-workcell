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
import shutil
import time
from typing import List, Optional

from .corrector import Corrector, StopSafe
from .lower_check import LowerCheck
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
        self.g = layout["gripper"]
        self.lim = layout["limits"]
        self.down_cam = layout["vision"]["down_camera"]
        self.front_cam = layout["vision"]["front_camera"]
        self.led = layout["vision"]["led"]
        self._last_health = 0.0

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
        self.store.s.last_safe_pose = after
        return dev

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
            self._fault(label, "motion_refused", str(e))
            if not self.robot.state().enabled:
                self.corr.on_disabled(label)
            raise StopSafe(f"{label}: {e}")
        dev = self._settle(label, target)
        dt = time.time() - t0
        self.tele.log("move", label, kind, target=[round(v, 2) for v in target],
                      actual=[round(v, 2) for v in self.robot.pose()], dt=round(dt, 2), speed=speed, dev=round(dev, 2))
        self._observe(f"dt:{label}", dt)

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
        self.robot.set_speed(self.safety.profile.travel)
        for i, (j, p) in enumerate(pts, 1):
            label = f"{name}[{i}/{len(pts)}]"
            self.safety.check_target(p, label)
            ext = self.robot.joints()[4:]
            t0 = time.time()
            try:
                self.robot.move_abs_joints(list(j[:4]) + list(ext))     # external axes are never driven here
            except MotionRefused as e:
                self._fault(label, "motion_refused", str(e))
                raise StopSafe(f"{label}: {e}")
            dev = self._settle(label, p)
            self.tele.log("move", label, "AJ", dt=round(time.time() - t0, 2), dev=round(dev, 2))

    def _jaw(self, value: float, label: str):
        try:
            self.robot.jaw_to(value)
        except MotionRefused as e:
            self._fault(label, "jaw_refused", str(e))
            raise StopSafe(f"{label}: {e}")

    def _jaw_open(self) -> float:
        self._jaw(self.g["open"], "jaw open")
        self._wait_still("jaw open")
        time.sleep(0.5)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "open", jaw=round(jaw, 2))
        if jaw < self.g["open"] - 1.0:
            raise StopSafe(f"jaw did not open ({jaw:.2f})")
        return jaw

    def _jaw_close(self, grip: str) -> float:
        prof = self.safety.profile
        closed = self.g["closed"][grip]
        v = self.robot.jaw()
        curve = []
        while v > closed + 1e-6:
            v = max(closed, v - prof.jaw_step)
            self._jaw(v, "jaw close")
            time.sleep(prof.jaw_pause)
            curve.append(round(self.robot.jaw(), 3))
        self._wait_still("jaw close")
        time.sleep(0.5)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "close", jaw=round(jaw, 2), curve=curve, grip=grip)
        self._observe(f"jaw:closed:{grip}", jaw)
        return jaw

    def _health(self, force: bool = False):
        if not force and time.time() - self._last_health < self.lim["health_check_every_s"]:
            return
        self._last_health = time.time()
        st = self.robot.state()
        try:
            vs = self.vision.status()
        except Exception as e:
            self.corr.alert("health", f"vision box unreachable: {e}")
            raise StopSafe("vision box unreachable")
        free_gb = shutil.disk_usage(self.run_dir).free / 1e9
        self.tele.log("health", "check", robot_enabled=st.enabled, drag=st.drag, vision=vs.state, clients=vs.clients,
                      disk_free_gb=round(free_gb, 1))
        if st.drag:
            raise StopSafe("drag mode turned on during the run")
        if not st.enabled:
            raise StopSafe("controller lost enable between steps")
        if free_gb < self.lim["min_disk_free_gb"]:
            raise StopSafe(f"disk nearly full ({free_gb:.1f} GB free)")

    # ------------------------------------------------------------ vision gates
    def _frame(self, cam: str) -> Optional[bytes]:
        try:
            return self.vision.frame(cam, 1920, 1080, 85, None, self.led)
        except Exception as e:
            self.tele.log("vision", "frame", f"{cam}: {e}")
            return None

    def _vision_correction(self, fixture: dict, label: str) -> List[float]:
        ref = fixture.get("aruco_reference")
        if not ref:
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
            if self.supervised:
                return
            raise StopSafe(f"{step}: front camera frame failed after lift")
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
        self._jaw_open()
        target = _add(pose, self._vision_correction(fixture, step))
        hover = _add(target, [0, 0, fixture["hover_mm"]])
        self._move("L", hover, f"{step} hover(corrected)", prof.approach)

        for attempt in range(1, self.corr.budget.regrasp + 2):
            self._gate_lower(gate, "plate_present", step)
            problems = self.checks.before_grasp(self.robot.jaw(), self.robot.state())
            if problems:
                raise StopSafe(f"{step}: " + "; ".join(problems))
            self._move("L", target, f"{step} lower", prof.contact, allow_floor=(fixture_name,))
            jaw = self._jaw_close(grip)
            err = self.checks.grasp_ok(jaw, grip)
            if not err:
                break
            self._fault(step, "grasp", err)
            self._jaw_open()
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
        target = _add(pose, self._vision_correction(fixture, step))
        hover = _add(target, [0, 0, fixture["hover_mm"]])
        self._move("L", hover, f"{step} hover(corrected)", prof.approach)
        problems = self.checks.before_release(self.robot.jaw(), self.store.s.grip)
        if problems:
            raise StopSafe(f"{step}: " + "; ".join(problems))
        self._gate_lower(gate, "empty", step)
        self._move("L", target, f"{step} lower", prof.contact, allow_floor=(fixture_name,))
        self._jaw_open()
        self._move("L", hover, f"{step} lift", prof.approach)
        img = self._frame(self.down_cam)
        self.tele.image(f"{step}_placed", img)
        if img is None:
            raise StopSafe(f"{step}: no frame for the seated check")
        v = self.lower.verdict(gate, "plate_present", img)
        self.tele.log("vision", f"{step} seated_check", v.reason, score=round(v.best, 3), safe=v.safe, gate=gate)
        if not v.safe and v.n_refs >= self.lower.min_refs:
            self._fault(step, "not_seated", v.reason)
            if not self.corr.operator_or_stop(step, "plate does not look seated after release",
                                              "Plate is seated correctly?"):
                raise StopSafe("plate not seated")
        self.store.done(plate_at=at, grip="none")

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

    def _hotel_place(self, slot: str):
        prof = self.safety.profile
        p, approach, _, carry = self._hotel_geometry(slot)
        step = f"place:hotel:{slot}"
        if self.store.s.hotel.get(slot) is not None:
            raise StopSafe(f"{step}: state says {slot} is occupied")
        self.store.begin(step)
        self._health()
        self._waypoints("polar_to_hotel")
        self._move("J", approach, f"{step} approach", prof.approach)
        ready, img = self.vision.live_detection(self.front_cam, aruco=False, led=self.led)
        self.tele.image(f"{step}_slot_view", img)
        self.tele.log("vision", f"{step} slot_view", "object seen" if ready else "slot looks empty", ready=ready)
        if ready and not self.corr.operator_or_stop(step, f"{slot} may be occupied", "Slot is actually empty?"):
            raise StopSafe(f"{slot} occupied")
        if self.checks.before_release(self.robot.jaw(), "long_edges"):
            raise StopSafe(f"{step}: not holding a plate on its long edges")
        self._move("L", carry, f"{step} slide in", prof.contact)
        self._move("L", p, f"{step} lower", prof.contact)
        self._jaw_open()
        self._move("L", carry, f"{step} lift off", prof.approach)
        self._move("L", approach, f"{step} slide out", prof.approach)
        self.store.s.hotel[slot] = self.store.s.plate_id
        self.store.done(plate_at=f"hotel:{slot}", grip="none")

    def _hotel_pick(self, slot: str):
        prof = self.safety.profile
        p, approach, align, carry = self._hotel_geometry(slot)
        step = f"pick:hotel:{slot}"
        self.store.begin(step)
        self._health()
        self._move("J", approach, f"{step} approach", prof.approach)
        self._jaw_open()
        self._move("L", align, f"{step} align", prof.approach)
        self._move("L", p, f"{step} slide in", prof.contact)
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
    def outbound(self, slot: str):
        F = self.L["fixtures"]
        P0, P90 = F["polar"]["pose_at_0"]["pose"], F["polar"]["pose_at_90"]["pose"]
        self._pick("microscope", F["microscope"]["pose"], F["microscope"], "short_edges", None)
        self._place("polar", P0, F["polar"], "polar", "microscope_to_polar", gate="polar@0")
        self._polar_rotate(90, P0)
        self._pick("polar", P90, F["polar"], "long_edges", None, gate="polar@90")
        self._scan_plate()
        self._hotel_place(slot)

    def inbound(self, slot: str):
        F = self.L["fixtures"]
        P0, P90 = F["polar"]["pose_at_0"]["pose"], F["polar"]["pose_at_90"]["pose"]
        self._hotel_pick(slot)
        self._place("polar", P90, F["polar"], "polar", None, gate="polar@90")
        self._polar_rotate(0, P90)
        self._pick("polar", P0, F["polar"], "short_edges", None, gate="polar@0")
        self._place("microscope", F["microscope"]["pose"], F["microscope"], "microscope", "microscope_to_polar",
                    reverse=True)

    def _check_resumable(self, slot: str):
        s = self.store.s
        if not s.step_done:
            raise StopSafe(f"last step '{s.step}' did not finish; plate_at={s.plate_at} grip={s.grip}. "
                           "Inspect the cell, fix state.json, then rerun.")
        if s.plate_at not in ("microscope", f"hotel:{slot}"):
            raise StopSafe(f"plate_at={s.plate_at}: no automatic resume from here; move the plate by hand "
                           "and edit state.json")
        if s.last_safe_pose:
            here = self.robot.pose()
            far = max(abs(here[i] - s.last_safe_pose[i]) for i in range(3))
            if far > self.lim["max_restart_offset_mm"]:
                raise StopSafe(f"arm is {far:.0f} mm from where the last run left it; jog it back before restarting")

    def run(self, slot: str, cycles: int):
        s = self.store.s
        self.hotel_slot_pose(slot)
        self._check_resumable(slot)
        self._health(force=True)
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
            self._observe("dt:cycle", dt)
