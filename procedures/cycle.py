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
    clock:       time() and sleep() for every fixed wait (jaw settle, stage settle, retry pauses); the
                 time module, or a fake the tests inject so the simulator does not wait in real seconds
    frame_source: where the checks take their camera frames; anything with vision_box's frame() signature.
                  Default: the box itself. The GUI passes its always-on frame buffer.
    parked:      the microscope's stage-parked report; the arm enters a `needs_parked` region only while it is True.
                 None: no microscope device to ask (cli runs, exercise), the region is entered as before
    """

    def __init__(self, layout: dict, robot: Robot, vision: VisionClient, tele: Telemetry, store: StateStore,
                 mode: str, run_dir: str, ref_dir: str, scope: str = "microscope",
                 layout_path: Optional[str] = None, stop_event: Optional[threading.Event] = None, clock=time,
                 parked: Optional[Callable[[], bool]] = None, frame_source=None):
        self.L, self.robot, self.vision, self.tele, self.store = layout, robot, vision, tele, store
        self.scope, self.run_dir, self.layout_path, self.clock = scope, run_dir, layout_path, clock
        self.frames = frame_source or vision
        self.stop_event = stop_event or threading.Event()
        self.ask_before_lower = None             # callable(label) -> bool; set by a supervisor that wants a word before every lowering
        self.parked = parked or (lambda: True)
        self.supervised = mode == "supervised"
        self.safety = Safety(layout, mode, robot.joints)
        from routes import plan as _plan_mod
        from routes.graph import StationGraph
        _plan_mod.GRAPH = StationGraph(self.stations)       # legs between stations are planned once, stored, and replayed (routes/graph.py)
        self.plate = plates.default(layout)      # jaw widths and seat Z offset of the plate being handled
        self.checks = PreChecks(layout, self.plate.jaw_open, self.plate.jaw_closed)
        self.drift = DriftMonitor(os.path.join(run_dir, f"drift_{scope}.json"))
        self.corr = Corrector(layout, vision, robot, self.safety, tele, self.supervised, clock)
        self.polar = PolarStage(layout["polar"], robot, tele.log, self.supervised, store.s.polar_angle, clock)
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
            store.on_done = lambda step: self._hub(hub.outcome, step, True)

    # --------------------------------------------------------------- recording
    def _hub(self, write, *args):
        """Opt-in (vision.hub.record) rows for cephla-hub. Training data never stops the arm: a failed write is logged."""
        if not self.hub_cfg.get("record"):
            return
        try:
            write(*args, self.hub_cfg.get("outbox", hub.OUTBOX))
        except OSError as e:
            self.tele.log("hub", write.__name__, f"not recorded: {e}")

    def _fault(self, step: str, kind: str, detail: str):
        self.store.fault(step, kind, detail)
        self.tele.log("fault", step, kind, detail=detail)
        self._hub(hub.outcome, step, False)

    def _observe(self, feature: str, value: float):
        w = self.drift.observe(feature, value)
        if w:
            self.corr.on_drift(w)

    def _pick(self, fixture_name: str, pose: List[float], fixture: dict, grip: str, trajectory_in: Optional[str],
              gate: Optional[str] = None):
        prof = self.safety.profile
        step = f"pick:{fixture_name}"
        self.store.begin(step)
        self._health()
        hover = offset(pose, [0, 0, fixture["hover_mm"]])
        if trajectory_in:
            self._enter(trajectory_in, False, hover, fixture_name)
        self._arrive(hover, f"{step} hover", prof.approach)
        self._polar_ensure(fixture_name, gate, pose, hover)
        self._jaw_open(grip, fixture)
        target = self._seat(pose, [0.0] * 6)
        hover = offset(target, [0, 0, fixture["hover_mm"]])
        problems = self.checks.before_grasp(self.robot.jaw(), self.robot.state())
        if problems:
            raise StopSafe(f"{step}: " + "; ".join(problems))
        self._descend(target, f"{step} lower", allow_floor=(fixture_name,))
        for attempt in (1, 2):
            r, why = self._clamp(grip, step)
            if not why:
                break
            self._fault(step, "grasp", why)
            self._jaw_open(grip, fixture)
            if attempt == 2:
                raise StopSafe(f"{step}: nothing clamped in two closes: {why}")
        self._move("L", hover, f"{step} lift", prof.approach)
        self._carry_check(step, grip)
        self.store.done(plate_at="gripper", grip=grip)

    def _place(self, fixture_name: str, pose: List[float], fixture: dict, at: str, trajectory_in: Optional[str],
               reverse: bool = False, gate: Optional[str] = None, check: bool = True):
        prof = self.safety.profile
        step = f"place:{fixture_name}"
        self.store.begin(step)
        self._health()
        hover = offset(pose, [0, 0, fixture["hover_mm"]])
        if trajectory_in:
            self._enter(trajectory_in, reverse, hover, fixture_name)
        if not (check and self._check_seat_from_view(fixture_name, pose, fixture, hover, step)):      # check=False: the seat proof, the plate was just lifted off it
            self._arrive(hover, f"{step} hover", prof.approach)
        self._polar_ensure(fixture_name, gate, pose, hover)
        target = self._seat(pose, [0.0] * 6)
        hover = offset(target, [0, 0, fixture["hover_mm"]])
        # a place stops this far above the height a pick lowers to: the plate is let go just over its seat and drops the last millimetres.
        # Lowered to the pick height the plate bottoms out in the clips and the jaw cannot open (module, 2026-11-02: 24.66 against 24.84)
        lift = float((fixture.get("place_lift_mm") or {}).get(self.store.s.grip, 0.0)) if isinstance(fixture.get("place_lift_mm"), dict) else float(fixture.get("place_lift_mm") or 0.0)
        if lift:
            target = [target[0], target[1], target[2] + lift] + list(target[3:])
        self._carry_check(step, self.store.s.grip)
        # No camera on the claw can see a plate already in the seat while another is carried over it: the seated plate's edges project below the
        # carried plate's and are hidden by it (front camera at the hover, 2026-11-04 01:50). The seat is checked with the empty claw instead.
        self._descend(target, f"{step} lower", allow_floor=(fixture_name,))
        self._collect("seat", fixture_name, "held", "front")           # the pair the tilt check will use: level in the fingers, then as it lands
        self._jaw_open(self.store.s.grip, fixture)
        self._collect("seat", fixture_name, "flat", "front")
        self._move("L", hover, f"{step} lift", prof.approach)
        self.store.done(plate_at=at, grip="none")

    def _check_seat_from_view(self, fixture_name: str, pose: List[float], fixture: dict, hover: List[float], step: str) -> bool:
        """Before a plate is lowered onto the module or a microscope: is the seat already occupied? With the plate in hand the arm goes to the
        station's viewing pose first (fixtures.<station>.view, taught with tools/view_pose.py; the carried plate is outside the down camera's
        field there, 2026-11-04 01:50), the down camera is judged against the seat's empty pictures, then the arm comes over the seat at
        that height and down to the hover. Returns True when it ended at the hover. vision.check_before_lower switches it; vision.gate_before_lower
        makes a confident "full" a stop (off until the false-positive rate is known); otherwise the verdict is logged and shown."""
        v = self.L["vision"]
        view = (fixture.get("view") or {}).get("tool_offset")
        if not v.get("check_before_lower", True) or not view or fixture_name == "hotel":
            return False
        from tools.view_pose import from_tool_frame
        target = from_tool_frame(pose, *view)
        zmax = float(self.L["workspace_box"]["z"][1]) - 3.0
        if target[2] > zmax:
            target = [target[0], target[1], zmax] + list(target[3:])
        prof = self.safety.profile
        self._arrive(hover, f"{step} hover", prof.approach)                 # the stored leg to the hover; from there straight up and sideways: nothing new to plan
        self._move("L", [hover[0], hover[1], target[2]] + list(hover[3:]), f"{step} up to the occupancy view", prof.approach)
        self._move("L", target, f"{step} occupancy view", prof.approach)
        # judged against this seat's own pictures from this very situation, a plate in hand at the view (reference_images/occupancy_drop); the
        # empty-claw pictures differ (closed fingers, the plate's edge) and read 0.44 on an empty module (02:20). Every drop-in picture whose
        # seat the ledger calls empty becomes a reference after the verdict; the first one has none.
        got = self._collect("occupancy_drop", fixture_name, "empty", "down", judge=True, save=(self.store.s.contents.get("module" if fixture_name == "polar" else fixture_name) != "full"))
        word = got[1] if got else "no verdict (no reference yet)"
        self.tele.log("seat", fixture_name, f"before lowering: the camera says {word}")
        print(f"   {fixture_name}: occupancy before lowering: {word}", flush=True)
        if got and got[0] == "full" and v.get("gate_before_lower", False):
            raise StopSafe(f"{step}: the camera says the seat is occupied ({word}); not lowering")
        self._move("L", [hover[0], hover[1], target[2]] + list(hover[3:]), f"{step} over the seat", prof.approach)
        self._move("L", hover, f"{step} hover", prof.approach)
        return True

    # ------------------------------------------------------------------ polar
    def _polar_ensure(self, fixture_name: str, gate: Optional[str], pose: List[float], hover: List[float]) -> None:
        """At the module's hover, before the lower: the module at the angle this seat is taught for (`gate` polar@0 for the short-edge seat,
        polar@90 for the long-edge one), read from the axis, never assumed. Set down at the other angle the plate lies across the seat
        (A7's plate, 2026-11-03 19:52: the test had turned the module to 0 and the long-edge place did not look). Off: up to the clear
        height, turn, back to the hover."""
        if fixture_name != "polar" or not gate or "@" not in gate or self.polar.driver != "ext_axis":
            return
        deg = int(gate.split("@")[1])
        real = self.polar.read_angle()
        real = real - 360.0 * round(real / 360.0)
        # 0.3 deg, not 1: a plate seated 0.6 deg off at 90 is gripped 0.6 deg off by the long edges and its corner meets a post in the rack
        # (B3, 2026-11-04 01:45: held width 3.65 against 3.0, wrist torque stop 13 mm into the slide)
        if abs(real - deg) <= 0.3 and self.polar.angle == deg:
            return
        self.tele.log("polar", "angle", f"the module reads {real:.1f} deg, this seat needs {deg}: turning it first")
        F = self.L["fixtures"]["polar"]
        prof = self.safety.profile
        clear = offset(pose, [0, 0, F["hover_mm"] + F["clear_mm"]])
        self._move("L", clear, "polar clear", prof.approach)
        self.polar.rotate_to(deg)
        err = (self.polar.read_angle() - deg + 180.0) % 360.0 - 180.0
        self.tele.log("polar", "readback", deg=deg, err_deg=round(err, 3))
        self.store.s.polar_angle = self.polar.angle = deg
        self.store.save()
        self._move("L", hover, "back to the hover", prof.approach)

    def _polar_rotate(self, deg: int, from_pose: List[float]):
        F = self.L["fixtures"]["polar"]
        self.store.begin(f"polar:{deg}")
        clear = offset(from_pose, [0, 0, F["hover_mm"] + F["clear_mm"]])
        self._move("L", clear, "polar clear", self.safety.profile.approach)
        while True:
            try:
                self.polar.rotate_to(deg)
                if self.polar.driver == "ext_axis":
                    err = (self.polar.read_angle() - deg + 180.0) % 360.0 - 180.0
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

    def _to_hotel(self):
        """From the microscope hover or the stage hover to the hotel junction, along the taught routes. With the hotel's
        poses from the CAD nothing is replayed: the move to a slot plans its own way."""
        if self._planned():
            return
        here = self._position_here()
        if here and not here.startswith("polar"):          # at a microscope hover (this one, or one sharing its pose)
            self._waypoints(self.L["fixtures"][here].get("trajectory", "microscope_to_polar"))
            here = "polar"
        if here and here.startswith("polar"):
            self._waypoints("polar_to_hotel")

    def _check_resumable(self, slot: str, resume: bool = False):
        s, scope = self.store.s, self.scope
        self.hotel_slot_pose(slot)                   # a ledger from another hotel layout may name a slot this one lacks
        if resume and not s.step_done and s.plate_at == "gripper" and s.step.startswith("place:"):
            return                                   # a carry was interrupted on a route; continue that step
        if resume and not s.step_done and s.grip == "none" and s.step.startswith("pick:"):
            problems = self.checks.before_grasp(self.robot.jaw(), self.robot.state())
            if problems:                             # the ledger learns of a grasp only after it; the jaw knows now
                raise StopSafe(f"{s.step} was interrupted and cannot restart: {'; '.join(problems)}. "
                               "The fingers may hold the plate; inspect the cell and open them by hand first.")
            return                                   # a pick was interrupted before the grasp; the step restarts
        if not s.step_done:
            raise StopSafe(f"last step '{s.step}' did not finish; plate_at={s.plate_at} grip={s.grip}. "
                           "Inspect the cell, fix state.json, or rerun with --resume if the arm is still on its route.")
        if s.plate_at not in (scope, "none") and not s.plate_at.startswith("hotel:"):
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
            if s.plate_at == "none":
                self._to_hotel()                         # an empty holder: travel out before the first fetch
            nxt = self._current_hotel_slot() if slot == "random" else slot
            if nxt and (s.plate_at in ("none",) or s.plate_at.startswith("hotel:") or s.plate_at == "gripper" and not s.step_done):
                self.inbound(nxt)
            self.drift.save()
            dt = time.time() - t0
            self.tele.log("cycle", "done", dt=round(dt, 1), state=s.plate_at)
            self._observe("dt:cycle", dt)
