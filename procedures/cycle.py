"""
The plate cycle.

    microscope (short-edge grip) -> polar stage at 0 deg -> rotate 90 -> long-edge grip
    -> hotel slot -> back to polar at 90 -> rotate 0 -> short-edge grip -> microscope

Every fixture visit follows the same pattern: travel, hover, vision gate, lower, jaw action, jaw
verification, lift, held-plate check.  The state file is updated before and after each step.
Motion is never retried; every motion is observed to completion and compared with its target.
"""
from __future__ import annotations

import os
import threading
import time
from typing import Callable, List, Optional

from devices.arm import Robot
from devices.stage import PolarError, PolarStage
from devices.vision_box import VisionClient
from perception import hub
from perception.lower_check import LowerCheck
from perception.occupancy import Occupancy
from perception.predictor import DriftMonitor, PreChecks
from procedures import plates
from procedures.corrector import Corrector, StopSafe
from procedures.gates import Gates
from procedures.grasp import Grasp
from procedures.hotel import Hotel
from procedures.motion import Motion, offset
from procedures.safety import Safety
from procedures.state import StateStore
from procedures.telemetry import Telemetry


class Cycle(Motion, Grasp, Gates, Hotel):
    """One plate cycle on one microscope. The four parents only split the methods over files by topic
    (moves, jaw, checks, hotel); every attribute they use is set here.

    scope:       this cycle's microscope, a fixture name in the positions file
    layout_path: where learned poses and grasp widths are written back; None keeps them in memory
    stop_event:  set by another task (run.py) when the cell must stop at the next step
    parked:      the microscope's stage-parked report; the arm enters a `needs_parked` region only while it is True.
                 None: no microscope device to ask (cli runs, exercise), the region is entered as before
    """

    def __init__(self, layout: dict, robot: Robot, vision: VisionClient, tele: Telemetry, store: StateStore,
                 mode: str, run_dir: str, ref_dir: str, scope: str = "microscope",
                 layout_path: Optional[str] = None, stop_event: Optional[threading.Event] = None,
                 parked: Optional[Callable[[], bool]] = None):
        self.L, self.robot, self.vision, self.tele, self.store = layout, robot, vision, tele, store
        self.scope, self.run_dir, self.layout_path = scope, run_dir, layout_path
        self.stop_event = stop_event or threading.Event()
        self.parked = parked or (lambda: True)
        self.supervised = mode == "supervised"
        self.safety = Safety(layout, mode, robot.joints)
        self.plate = plates.default(layout)      # jaw widths and seat Z offset of the plate being handled
        self.checks = PreChecks(layout, self.plate.jaw_open, self.plate.jaw_closed)
        self.drift = DriftMonitor(os.path.join(run_dir, f"drift_{scope}.json"))
        self.corr = Corrector(layout, vision, robot, self.safety, tele, self.supervised)
        self.polar = PolarStage(layout["polar"], robot, tele.log, self.supervised, store.s.polar_angle)
        lc = layout["vision"]["lower_check"]
        self.lower = LowerCheck(ref_dir, lc["min_refs"], lc["margin"], lc["floor"])
        self.slot_cfg = layout["vision"].get("slot_check", {})
        self.slots = Occupancy(ref_dir, self.slot_cfg.get("min_refs", 5), self.slot_cfg.get("min_match", 0.75))
        self.lim = layout["limits"]
        self.down_cam = layout["vision"]["down_camera"]
        self.front_cam = layout["vision"]["front_camera"]
        self.led = layout["vision"]["led"]
        self.front_led = int(layout["vision"].get("front_led", self.led))
        self._last_health = 0.0
        self.commission = {"no_gates": False, "no_vision_correction": False}
        self.resuming = False
        self._search_widths: List[float] = []        # jaw widths seen during a centering search
        self.hub_cfg = layout["vision"].get("hub", {})   # record: every gate verdict and outcome go to the outbox
        if self.hub_cfg.get("record"):
            store.on_done = lambda step: hub.outcome(step, True, self.hub_cfg.get("outbox", hub.OUTBOX))

    # --------------------------------------------------------------- recording
    def _fault(self, step: str, kind: str, detail: str):
        self.store.fault(step, kind, detail)
        self.tele.log("fault", step, kind, detail=detail)
        if self.hub_cfg.get("record"):
            hub.outcome(step, False, self.hub_cfg.get("outbox", hub.OUTBOX))

    def _observe(self, feature: str, value: float):
        w = self.drift.observe(feature, value)
        if w:
            self.corr.on_drift(w)

    def _pick(self, fixture_name: str, pose: List[float], fixture: dict, grip: str, trajectory_in: Optional[str],
              gate: Optional[str] = None):
        prof = self.safety.profile
        gate = gate or fixture_name
        step = f"pick:{fixture_name}"
        self.store.begin(step)
        self._health()
        if trajectory_in:
            self._waypoints(trajectory_in)
        hover = offset(pose, [0, 0, fixture["hover_mm"]])
        self._arrive(hover, f"{step} hover", prof.approach)
        self._jaw_open(grip, fixture)
        corr = self._vision_correction(fixture, step)
        target = self._seat(pose, corr)
        hover = offset(target, [0, 0, fixture["hover_mm"]])
        if any(abs(v) > 0.05 for v in corr[:3]):
            self._move("L", hover, f"{step} hover(corrected)", prof.approach)

        for attempt in range(1, self.corr.budget.regrasp + 2):
            self._gate_lower(gate, "plate_present", step)
            problems = self.checks.before_grasp(self.robot.jaw(), self.robot.state())
            if problems:
                raise StopSafe(f"{step}: " + "; ".join(problems))
            self._descend(target, f"{step} lower", allow_floor=(fixture_name,))
            jaw = self._jaw_close(grip)
            regrip = fixture.get("regrip_above", {}).get(grip)
            for k in range(int(fixture.get("regrip_max", 2))):
                # A wide first close at the microscope means the plate sits skewed in the holder's clips (robot-placed
                # plates read 22.6 to 22.7, hand-placed 22.4 to 22.55). Opening and closing again squares it up: the
                # 17:05 search saw 22.89 -> 22.64 -> 22.54 from the re-closes alone. A skewed plate carried to the
                # stage does not drop into the nest at 0 (wrist collision 2026-10-20 23:25).
                if regrip is None or jaw <= regrip + 1e-6:
                    break
                self.tele.log("jaw", "regrip", f"{jaw:.2f} above {regrip}: open and close again ({k + 1})", jaw=round(jaw, 2))
                self._jaw_open(grip, fixture)
                jaw = self._jaw_close(grip)
            expected = fixture.get("grasp_width", {}).get(grip)
            err = self.checks.grasp_ok(jaw, grip, expected)
            if err and jaw > self.plate.jaw_closed[grip]:          # wider than the plate: pads off-centre, not an empty grasp
                found = self._center_search(step, fixture_name, fixture, target, grip)
                if found:
                    jaw, target = found
                    hover = offset(target, [0, 0, fixture["hover_mm"]])
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
            target = self._seat(pose, self._vision_correction(fixture, step))
            hover = offset(target, [0, 0, fixture["hover_mm"]])
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
        hover = offset(pose, [0, 0, fixture["hover_mm"]])
        self._arrive(hover, f"{step} hover", prof.approach)
        corr = self._vision_correction(fixture, step)
        target = self._seat(pose, corr)
        hover = offset(target, [0, 0, fixture["hover_mm"]])
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

    # ------------------------------------------------------------------ polar
    def _polar_rotate(self, deg: int, from_pose: List[float]):
        F = self.L["fixtures"]["polar"]
        self.store.begin(f"polar:{deg}")
        clear = offset(from_pose, [0, 0, F["hover_mm"] + F["clear_mm"]])
        self._move("L", clear, "polar clear", self.safety.profile.approach)
        self._stage_clear(f"polar:{deg}", deg)
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

    # --------------------------------------------------------------- the loop
    def _steps(self, slot: str):
        """The two routes of one plate: microscope -> hotel slot (out) and hotel slot -> microscope (back).
        The microscope's route to the stage is fixtures.<scope>.trajectory."""
        scope, F = self.scope, self.L["fixtures"]
        P0, P90 = F["polar"]["pose_at_0"]["pose"], F["polar"]["pose_at_90"]["pose"]
        route = F[scope].get("trajectory", "microscope_to_polar")
        out = [
            (f"pick:{scope}", lambda: self._pick(scope, F[scope]["pose"], F[scope], "short_edges", None)),
            ("place:polar", lambda: self._place("polar", P0, F["polar"], "polar", route, gate="polar@0")),
            ("polar:90", lambda: self._polar_rotate(90, P0)),
            ("pick:polar", lambda: self._pick("polar", P90, F["polar"], "long_edges", None, gate="polar@90")),
            ("scan", self._scan_plate),
            (f"place:hotel:{slot}", lambda: self._hotel_place_free(slot)),
        ]
        back = [
            (f"pick:hotel:{slot}", lambda: self._hotel_pick(slot)),
            ("place:polar", lambda: self._place("polar", P90, F["polar"], "polar", None, gate="polar@90")),
            ("polar:0", lambda: self._polar_rotate(0, P90)),
            ("pick:polar", lambda: self._pick("polar", P0, F["polar"], "short_edges", None, gate="polar@0")),
            (f"place:{scope}", lambda: self._place(scope, F[scope]["pose"], F[scope], scope, route, reverse=True)),
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
            if self.stop_event.is_set():
                raise StopSafe(f"{name}: not started, another microscope's task stopped the cell")
            fn()
            self.resuming = False          # only the first route after a resume may start mid-way

    def outbound(self, slot: str, from_step: Optional[str] = None):
        """Carry the plate from the microscope to hotel slot `slot`."""
        self._run_steps(self._steps(slot)[0], from_step)

    def inbound(self, slot: str, from_step: Optional[str] = None):
        """Carry the plate from hotel slot `slot` to the microscope."""
        self._run_steps(self._steps(slot)[1], from_step)

    def _check_resumable(self, slot: str, resume: bool = False):
        s, scope = self.store.s, self.scope
        if resume and not s.step_done and s.plate_at == "gripper" and s.step.startswith("place:"):
            return                                   # a carry was interrupted on a route; continue that step
        if resume and not s.step_done and s.grip == "none" and s.step.startswith("pick:"):
            return                                   # a pick was interrupted before the grasp; the step restarts
        if not s.step_done:
            raise StopSafe(f"last step '{s.step}' did not finish; plate_at={s.plate_at} grip={s.grip}. "
                           "Inspect the cell, fix state.json, or rerun with --resume if the arm is still on its route.")
        if s.plate_at != scope and not s.plate_at.startswith("hotel:"):
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
        raise StopSafe(f"rotating stage reads {a:.2f} deg, neither 0 nor 90; cli.py polar goto 0 first")

    def run(self, slot: str, cycles: int, resume: bool = False, stop_file: Optional[str] = None):
        """`cycles` plate changes on this microscope. Each change returns the plate on the microscope to the slot it
        came from (its home; a plate with no known home goes to `slot`), then brings the plate in `slot` to the
        microscope. `random` draws a free slot. So run(A1) then run(C2) is: load A1, then A1 home and C2 in."""
        s, scope = self.store.s, self.scope
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
                elif s.step in ("place:polar", f"place:{scope}") and s.grip == "long_edges":
                    self.inbound(cur, from_step=s.step)
                elif s.step == f"place:{scope}":
                    self.inbound(cur, from_step=f"place:{scope}")
                resume = False
            else:
                s.cycle += 1
                self.tele.cycle = s.cycle
                self.store.save()
                if s.plate_at == scope:
                    self.outbound(s.home_slot or self._choose_slot(slot))
            nxt = self._current_hotel_slot() if slot == "random" else slot
            if nxt and (s.plate_at.startswith("hotel:") or s.plate_at == "gripper" and not s.step_done):
                self.inbound(nxt)
            self.drift.save()
            dt = time.time() - t0
            self.tele.log("cycle", "done", dt=round(dt, 1), state=s.plate_at)
            self._observe("dt:cycle", dt)
