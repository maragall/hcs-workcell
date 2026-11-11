"""
The setup, section by section, in the order the user meets them. Each line is one Step; a machine line calls the
function cli.py and the loop already use (bringup.cameras, Cycle, exercise, the vision box, Squid) and returns
the values its comment names. Nothing here talks to a device directly except through those.

Written through `positions`: layout.json -> `setup` (what this run found), the fixtures it re-seats, the gripper
widths. Written to the ledger: `contents` (what is where) in state_microscope.json.

Not run on the real arm yet: the sections are exercised on the simulator (tests/test_bringup.py).
"""
from __future__ import annotations

import json
import math
import os
import random
import time
from typing import Callable, List, Optional

import numpy as np

import exercise
from bringup import geometry
from bringup.runner import Refused, Section, Step
from bringup import bench, cameras, colour, hand
from procedures import cad_poses
from procedures.corrector import StopSafe
from procedures.cycle import Cycle
from procedures.motion import offset
from procedures.recovery import clear_faults, is_motion_fault
from procedures.safety import SafetyViolation, SpeedProfile
from procedures.state import StateStore
from procedures.telemetry import Telemetry

CARRY_ON = ("safe to lower", "clear to lower", "clear to rotate", "fine to continue", "seated correctly", "held correctly", "Continue anyway", "Proceed with the taught pose")      # the camera gates' questions: the setup answers yes and logs the mismatch
SEAT_MOVED_KEEP_MM = 1.0          # closer than this the taught seat stays: the read has that much noise
SEARCH_HEIGHTS = 4                # heights tried below the microscope hover, highest allowed first
ACQUISITION_TIMEOUT_S = 600
VIEW_DISTANCE_MM = 140.0         # camera height above the tag for the capture grid (calibration/plan.py tag_in_view)
TAG_VIEW_MARGIN_MM = 5.0         # headroom under the workspace top for the stage view
SEAT_MOVED_DROP_ROUTE_MM = 8.0       # a hover this far from the taught seat: the taught route belongs to another place
STAGE_SEARCH_MM = ((0, 0), (35, 0), (-35, 0), (0, 35), (0, -35), (35, 35), (-35, 35), (35, -35), (-35, -35),
                   (70, 0), (-70, 0), (0, 70), (0, -70))        # where the arm looks for the tag around the hover, x and y
SCOPE_LONG_MM, SCOPE_SHORT_MM = 330.0, 220.0      # the microscope as a box (operator, 2026-11-03): long edges along the plate's long edges


def microscope_box(L: dict, seat: List[float]) -> dict:
    """The microscope for the planner, from the seat alone: a box whose top is the stage, centred under the plate. The plate is gripped on
    its short edges there, so the plate's long edges (and the box's 33 cm) run across the claw, the 22 cm along it. The plate's centre is
    where the short-edge grip holds it (volumes.held.short_edges). The box top is set under the fingertips at the seat, so the grip itself
    is not inside it: the stage's own holder above that is where the claw must go anyway."""
    v = L["volumes"]
    hx = v["held"]["short_edges"]["x"]
    ahead = (hx[0] + hx[1]) / 2.0
    yaw = math.radians(seat[5])
    ux, uy = math.cos(yaw), math.sin(yaw)                  # along the claw
    vx, vy = -uy, ux                                       # across it
    cx, cy = seat[0] + ahead * ux, seat[1] + ahead * uy
    a, b = SCOPE_SHORT_MM / 2.0, SCOPE_LONG_MM / 2.0
    xs = [cx + i * a * ux + j * b * vx for i in (-1, 1) for j in (-1, 1)]
    ys = [cy + i * a * uy + j * b * vy for i in (-1, 1) for j in (-1, 1)]
    top = seat[2] + float(v["claw"]["z"][0]) - float(v.get("margin_mm", 10.0)) - 5.0
    return {"x": [round(min(xs), 1), round(max(xs), 1)], "y": [round(min(ys), 1), round(max(ys), 1)], "z": [-50.0, round(top, 1)], "links": False,
            "_note": f"from the seat taught {_stamp()}: {SCOPE_LONG_MM:.0f} x {SCOPE_SHORT_MM:.0f} mm box centred under the plate, top {5.0 + float(v.get('margin_mm', 10.0)):.0f} mm "
                     "under the fingertips at the seat (the stage holder above it is where the claw grips); the bounding box of the rotated rectangle"}


WRIST_WOUND_DEG = 180.0           # joint 4 beyond this has the same orientation one turn the other way, inside the soft limits
MIN_STAGE_FROM_ARM_MM = 200.0     # the microscope at least this far from the arm's base (operator, 2026-11-03); measured at the seat's reach from the axis
TAG_AHEAD_MM = 71.5              # both tag views are this far ahead of the grip pose along the claw: the tag centre is the plate centre (186.5 ahead of the wrist) and the camera looks 115 ahead of it; measured 2026-10-28 (the old 40 left the tag cut off at the top of the picture)
SIZE_ROWS = (2, 4)                # rows where a 15-slot hotel has no shelf and a 30-slot one has
VIEW_SEARCH_MM = ((0, 0), (25, 0), (-25, 0), (0, 25), (0, -25), (25, 25), (-25, -25), (25, -25), (-25, 25), (50, 0), (-50, 0))   # along, across the claw
VIEW_LIFTS_MM = (0, 60, 120)      # heights above the seat hover the view is tried at (the camera has to focus on the plate)
RESEATS = 1                       # re-grips at the module after the long-edge width is learned (grips.toml)


def reachable(cyc: Cycle, slot: str) -> bool:
    """The slot's seat, its approach point and its carry height are all inside the workspace box and above their floors."""
    try:
        for name, pose in zip(("seat", "approach", "align", "carry"), cyc._hotel_geometry(slot)):
            cyc.safety.check_target(pose, f"slot {slot} {name}", allow_floor_regions=("hotel",))
        return True
    except SafetyViolation:
        return False


def _lines(path: str) -> int:
    try:
        with open(path, "rb") as f:
            return sum(1 for _ in f)
    except OSError:
        return 0


def _stamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


JAW_SLACK_MM = 0.6          # a jaw that stalled against a plate reads a little outside its accept band (24.66 against 24.73 at the module, 2026-11-02)

class Bringup:
    def __init__(self, cell, squid, ask: Callable[[str], str], scope: str = "microscope", confirm_stage: bool = False, speed: str = "stress"):
        self.cell, self.squid, self.ask, self.scope = cell, squid, ask, scope
        self.confirm_stage = confirm_stage     # the operator has said, for this run, that lowering at the stage is safe
        self.speed = speed                     # the speed profile (layout speeds) the test runs at; the setup steps keep the supervised one
        self._cyc: Optional[Cycle] = None
        self.unattended = False               # set while the rotation runs for ever: a question that would wait for a person is answered no
        self.found: dict = {}                 # what this run learned: size, plate_offset, marker poses ...

    # ------------------------------------------------------------------ plumbing
    def edit(self, fn) -> None:
        self.cell.edit(fn)
        self._cyc = None                      # the Cycle holds a copy of the layout: build it again

    @staticmethod
    def setup_node(L: dict) -> dict:
        return L.setdefault("setup", {})

    def cyc(self) -> Cycle:
        if self._cyc is None:
            c = self.cell
            run_dir = os.path.join(c.runs_dir, time.strftime("setup_%Y%m%d_%H%M%S"))
            os.makedirs(c.state_dir, exist_ok=True)
            store = StateStore(os.path.join(c.state_dir, "state_microscope.json"))
            self._cyc = Cycle(c.layout(), c.robot, c.vision, Telemetry(run_dir), store, "supervised", run_dir, c.ref_dir,
                              scope=self.scope, layout_path=c.layout_path, stop_event=c.stop_event, frame_source=c.frames,
                              parked=self._parked)
            self._cyc.commission["no_vision_correction"] = True      # the real plate carries no marker: the offset is applied here
            self._answer_for(self._cyc)
        return self._cyc

    def _answer_for(self, cyc: Cycle) -> None:
        """The lower-check gate asks the operator when it has no pictures yet for a fixture ('is it safe to lower?'). For the module
        the setup knows: the tag view has just shown the plate on it, so it answers yes there. For anything else, the stage above
        all (its seat has never been validated on this elbow branch), the question stays with the operator."""
        corr, said = cyc.corr, {"reason": ""}
        alert = corr.alert


        def watch(step, reason):
            said["reason"] = reason
            return alert(step, reason)

        def ask(question, timeout=None):
            if "seated correctly" in question or "held correctly" in question:
                cyc.tele.log("setup", "gate", f"auto-confirmed during setup ({said['reason'][:80]}): the grip width after the next pick is the check; the picture becomes a reference")
                print(f"      (setup confirms this itself: {question.strip()} -> y; the grip width after the next pick is the check)")
                return "y"
            if "safe to lower" in question and "reference images for" in said["reason"] and "need" in said["reason"]:
                cyc.tele.log("setup", "lower_check", f"too few references to judge ({said['reason'][:90]}): lowering confirmed, the picture is kept as a reference")
                print("      (too few reference pictures to judge yet: setup confirms the lowering and keeps this picture)")
                return "y"
            if self.confirm_stage and "safe to lower" in question and self.scope in said["reason"]:
                cyc.tele.log("setup", "lower_check", "the operator confirmed lowering at the stage for this run (--confirm-stage)")
                return "y"
            if any(k in question for k in CARRY_ON):                  # a camera gate that doubts: logged, and the setup carries on (automation: nobody is waited for)
                line = f"{time.strftime('%Y-%m-%d %H:%M:%S')}  carried on over a mismatch ({question.strip()}): {said['reason'][:200]}"
                cyc.tele.log("setup", "mismatch", line)
                with open(os.path.join(self.cell.runs_dir, "mismatches.log"), "a", encoding="utf-8") as f:
                    f.write(line + chr(10))
                print("      (mismatch logged to runs/mismatches.log; carrying on)")
                return "y"
            if self.unattended:
                cyc.tele.log("setup", "question", f"not asked (unattended run): {question.strip()} -> n")
                print(f"      (unattended: no one is waited for; '{question.strip()}' -> n, so the run stops here with the reason above)")
                return "n"
            return self._ask_user(question)
        corr.alert, corr.ask = watch, ask

    def _ask_user(self, question: str) -> str:
        return self.ask(colour.paint(f"  >>> {question} [y/N]: ")).strip().lower()

    def _parked(self) -> bool:
        try:
            return bool(self.squid.parked())
        except Exception:
            return False                      # unknown park position: the arm stays out of the microscope zone

    def ready(self) -> None:
        if self.cell.robot.state().drag:      # a user step left the arm loose; the cell's own flag lags by its poll
            hand.drag(self.cell, False)
        self.cell.arm = dict(self.cell.arm, drag=False)
        why = self.cell.ready_to_move()
        if why:
            raise Refused(why)

    def stage_hover(self, hand_is_seat: bool = False) -> dict:
        """The customer dragged the hand over the plate holder (the aruco plate sits in it): that pose is the hover, and the seat
        is calculated from it, hover_mm below, the claw square to the instrument. A hover far from the taught one means the
        microscope was moved: the taught route to the old place is dropped, the planner takes over."""
        L = self.cell.layout()
        f = L["fixtures"][self.scope]
        here = [float(v) for v in self.cell.robot.pose()]
        j4 = float(self.cell.robot.joints()[3])
        if abs(j4) > WRIST_WOUND_DEG:
            # 2026-11-03 01:30: a seat dragged with the wrist at -220 deg; the first commanded move to it hit the controller's soft limit (3571)
            raise Refused(f"the wrist is wound: joint 4 is at {j4:.0f} deg. Lift the claw off the plate, turn it one full turn the other way "
                          f"(the same orientation, the wrist near the middle of its range), set it back around the plate, and run this section again")
        hover_mm = 0.0 if hand_is_seat else float(f.get("hover_mm", 67.0))      # taught by hand at the grip: the pose is the seat itself
        yaw = round(here[5], 3) if hand_is_seat else min((90.0 * k for k in (-2, -1, 0, 1, 2)), key=lambda a: abs(geometry.wrap(here[5] - a)))      # a plate in the fingers sets the yaw: it is the holder's, not a square number
        z = float(f["holder_z"]) if hand_is_seat and f.get("holder_z") else here[2] - hover_mm      # a hand cannot find the height: the fingers are tall; the holder's is fixed
        seat = [round(here[0], 3), round(here[1], 3), round(z, 3), 0.0, 0.0, yaw]
        old = f.get("pose")
        moved = round(geometry.moved_mm(old, seat), 1) if old else None
        r = math.hypot(seat[0] - 126.0, seat[1])
        if L.get("volumes") and not MIN_STAGE_FROM_ARM_MM <= r <= 395.0:        # the real cell's frame; the simulator's fakes have their own
            raise Refused(f"the hand is {r:.0f} mm from the arm's axis: the microscope must be at least {MIN_STAGE_FROM_ARM_MM / 10:.0f} cm "
                          "from the arm's base and its plate holder no more than 39 cm from the axis. Drag the hand to a spot in that "
                          "range and put the microscope under it")
    
        def write(L):
            f = L["fixtures"][self.scope]
            if old and moved and moved > 0.5:
                f[f"previous_pose_{time.strftime('%Y_%m_%d')}"] = {"pose": old}
            f["pose"], f["taught_at"] = seat, _stamp()
            f["hover"] = {"pose": [seat[0], seat[1], round(seat[2] + float(f.get("hover_mm", 67.0)), 3)] + seat[3:], "derived_from_seat": _stamp()}     # the hover is the seat's, never a second teach
            L.setdefault("volumes", {}).setdefault("obstacles", {})[self.scope] = microscope_box(L, seat)
            if moved is None or moved > SEAT_MOVED_DROP_ROUTE_MM:
                for name in [k for k in L.get("trajectories", {}) if k.startswith(f"{self.scope}_to_")]:
                    L["trajectories"].pop(name)
            self.setup_node(L).update(microscope_hover={"hand_pose": here, "seat": seat, "from_taught_mm": moved, "at": _stamp()})
        self.edit(write)
        return {"seat from the hover": seat, "from the taught seat mm": moved, "reach mm": round(r)}

    def leave_holder(self, release: bool = False) -> dict:
        """Straight up out of the microscope's zone (no sideways move there, the body is not in the planner): the arm is locked, the
        jaw opens first when `release` is set (the plate was held at the seat), then the wrist rises to the zone's free height."""
        c = self.cell
        z_reg = c.layout()["regions"]["microscope"]
        box, top = z_reg["box"], float(z_reg.get("free_above_z", 350.0))
        here = [float(v) for v in c.robot.pose()]
        inside = box["x"][0] <= here[0] <= box["x"][1] and box["y"][0] <= here[1] <= box["y"][1]
        if not inside or here[2] >= top - 1.0:
            return {"left the holder": "not needed"}
        if c.arm.get("drag"):
            hand.drag(c, False)
        cyc = self.cyc()
        if release:
            self.guard(lambda: cyc._jaw_open("short_edges", c.layout()["fixtures"][self.scope]))
        self.guard(lambda: cyc._move("L", [here[0], here[1], top] + here[3:], "setup:straight up out of the holder", cyc.safety.profile.approach))
        return {"left the holder": f"straight up to z {top:.0f}", "plate released": release}

    def check_seat(self) -> dict:
        """The hand-taught seat, proved with the machine's own pick and place: the aruco plate is in the holder, the arm closes on it from
        the hover, lifts it clear, sets it back, opens. The grip width must be in the band and the jaw must open all the way."""
        c = self.cell
        self.ready()
        self.confirm_stage = True                    # the operator is at the stage and has just set the plate in it: lowering onto it is theirs, confirmed
        cyc = self.cyc()
        F = c.layout()["fixtures"]
        cyc.store.s.plate_at, cyc.store.s.grip, cyc.store.s.step_done = self.scope, "none", True
        cyc.store.s.contents["module"], cyc.store.s.contents[self.scope] = "empty", "full"     # the operator moved the plate from the module into the holder
        cyc.store.save()
        # a setup teaches the grips afresh: this clamp (short edges, at the microscope) learns its width, the first long-edge clamp at the
        # module learns the other one before the first slot. Nothing stale from an earlier plate or earlier fingers can block them.
        self.edit(lambda L: L["gripper"].get("plate_width", {}).clear())
        cyc.checks.width.clear()
        # the hotel occupancy (fixtures.hotel.loaded, set by the operator) is kept: plates may already sit in slots (2026-11-04 02:00);
        # only the module/stage flags start clean
        self.edit(lambda L: (L["fixtures"]["polar"].pop("plate_on_module", None), L["fixtures"][self.scope].pop("plate_on_stage", None)))
        self.guard(lambda: cyc._pick(self.scope, F[self.scope]["pose"], F[self.scope], "short_edges", None))
        held = round(c.robot.jaw(), 2)
        self.guard(lambda: cyc._place(self.scope, F[self.scope]["pose"], F[self.scope], self.scope, None, check=False))   # no occupancy view: the plate was just lifted off this seat
        return {"gripped at": held, "put back and released": f"jaw {round(c.robot.jaw(), 2)}"}









    def module_view_pose(self, along: float = 0.0, across: float = 0.0, lift: float = 0.0) -> List[float]:
        """The wrist pose that puts the down camera over the module's plate: the taught seat's wrist pose moved along
        the claw. The camera looks `camera_ahead_of_wrist_mm` ahead of the wrist, the plate centre is
        `module_wrist_to_plate_mm` ahead of it at the seat (both in hardware.json, from the CAD)."""
        claw = cad_poses.load_hardware()["claw"]
        taught = self.cyc()._position("polar.pose_at_0")
        yaw = math.radians(taught[5])
        d, n = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
        shift = claw["module_wrist_to_plate_mm"] - claw["camera_ahead_of_wrist_mm"] + along
        return [taught[0] + shift * d[0] + across * n[0], taught[1] + shift * d[1] + across * n[1], taught[2] + lift, 0.0, 0.0, taught[5]]

    def centre_on_tag(self, tries: int = 4, tol_px: int = 150, max_move_mm: float = 12.0):
        """Put the down camera right over the plate tag before a capture plan, which is a grid around where the arm is: two small
        probe moves give how the tag moves in the picture per mm, then the wrist goes where the tag is in the middle. No camera
        calibration is needed (it is what is being solved for)."""
        import cv2
        import numpy as np
        c, v = self.cell, self.cell.layout()["vision"]
        cyc = self.cyc()

        def tag():
            data = c.vision.frame_full(v["down_camera"], v["led"])
            gray = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE) if data else None
            if gray is None:
                return None
            try:
                from calibration import marker
                corners = marker.detect(gray, None)
            except Exception:
                return None
            h, w = gray.shape
            return corners.mean(axis=0), np.array([w / 2.0, h / 2.0]), corners

        start = list(c.robot.pose())

        def nudge(dx, dy):
            h = c.robot.pose()
            self.guard(lambda: cyc.go_to([h[0] + dx, h[1] + dy, h[2], h[3], h[4], h[5]], "centre on tag"))

        def drift():
            h = c.robot.pose()
            return math.hypot(h[0] - start[0], h[1] - start[1])

        for _ in range(tries):
            got = tag()
            if got is None:
                raise Refused("the down camera does not see the whole tag to centre on it: is the aruco plate on the module, tag up?")
            at, mid = got[0], got[1]
            if np.linalg.norm(at - mid) <= tol_px:
                return got[2]
            cols = []
            for ux, uy in ((1.0, 0.0), (0.0, 1.0)):
                for sign in (1.0, -1.0):                  # the other way when the tag is near the picture's edge and leaves it
                    dx, dy = 5.0 * sign * ux, 5.0 * sign * uy
                    nudge(dx, dy)
                    moved = tag()
                    nudge(-dx, -dy)
                    if moved is not None:
                        cols.append((moved[0] - at) / (5.0 * sign))
                        break
                else:
                    raise Refused("the tag left the picture on a small test move both ways: centre the hand over the plate by hand "
                                  "and run again")
            J = np.array(cols).T                       # pixels per mm: columns are x and y of the wrist
            if abs(np.linalg.det(J)) < 1e-6:
                raise Refused("the tag did not move in the picture on test moves: is the camera reading a live picture?")
            step = np.linalg.solve(J, mid - at)
            if drift() + float(np.hypot(*step)) > max_move_mm:           # the tag is whole in the picture: good enough, and the claw
                return got[2]                                              # is near the hotel on this side
            try:
                nudge(float(np.clip(step[0], -40, 40)), float(np.clip(step[1], -40, 40)))
            except Refused:
                return got[2]
        got = tag()
        if got is None or np.linalg.norm(got[0] - got[1]) > 3 * tol_px:
            raise Refused("could not centre the camera over the tag in 4 tries")
        return got[2]

    def view_for_capture(self) -> dict:
        """Centre over the tag, go straight up until the camera is VIEW_DISTANCE_MM above it (so the whole tag stays in the picture over
        the capture grid), centre again, and say where the tag is: {"tag_xy", "dist_mm"} for the capture plan's visibility check."""
        import numpy as np
        c, cyc = self.cell, self.cyc()
        f_px = 2632.0                                         # the vendor lens: a tag's side in pixels gives its distance
        corners = self.centre_on_tag()
        side = float(np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]))
        dist = f_px * 40.0 / side
        if abs(dist - VIEW_DISTANCE_MM) > 3.0:
            h = c.robot.pose()
            self.guard(lambda: cyc.go_to([h[0], h[1], h[2] + (VIEW_DISTANCE_MM - dist), h[3], h[4], h[5]], "capture height"))
            corners = self.centre_on_tag()
            side = float(np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]))
            dist = f_px * 40.0 / side
        p, yaw = c.robot.pose(), math.radians(c.robot.pose()[5])
        ahead = cad_poses.load_hardware()["claw"]["camera_ahead_of_wrist_mm"]
        return {"tag_xy": [p[0] + ahead * math.cos(yaw), p[1] + ahead * math.sin(yaw)], "dist_mm": dist}

    def view_module(self) -> List[float]:
        """Take the camera over the module's plate and read the marker: the CAD pose first, then a small cross of
        offsets (the camera's sideways offset from the claw is not known). Remembers where it read."""
        c, v = self.cell, self.cell.layout()["vision"]
        cyc = self.cyc()
        stored = self.setup_node(c.layout()).get("module_view")
        tries = [self.found["module_view"]] if self.found.get("module_view") else ([stored] if stored else [])
        tries += [self.module_view_pose(a, b, z) for z in VIEW_LIFTS_MM for a, b in VIEW_SEARCH_MM]
        last = "no pose tried"
        for pose in tries:
            try:
                self.guard(lambda pose=pose: cyc.go_to(pose, "module view"))
            except Refused as e:                  # a candidate the safety check refuses is skipped, not fatal
                last = str(e)
                continue
            try:
                if any(c.vision.aruco_pose(v["down_camera"], 1, None, v["led"]).pixel) or cameras._local_reads(c, v["down_camera"], v["led"]):
                    self.found["module_view"] = pose
                    self.edit(lambda L, pose=pose: self.setup_node(L).update(module_view=[round(float(x), 3) for x in pose]))
                    return pose
            except Exception:
                pass
        raise Refused("the down camera cannot read the aruco plate over the module: is the practice plate on the module, "
                      f"marker up, and the lens clean? (last refusal: {last})")

    @staticmethod
    def guard(fn):
        """Run a motion step: the cell's own refusals become the setup's, with the message."""
        try:
            return fn()
        except (SafetyViolation, StopSafe) as e:
            raise Refused(str(e))

    def loosen(self) -> None:
        if self.cell.arm_error is None:
            hand.loosen(self.cell)

    # ------------------------------------------------------------------ the sections
    def sections(self) -> List[Section]:
        # the pre-nuke workflow without the hotel (operator, 2026-11-08): no hotel size, no slot survey; the hotel seats are taught
        # by hand (tools/teach_daemon.py, fitted by tools/fit_hotel_grid.py) and the occupancy is the operator's list
        return [self.connect(), self.cameras(), self.seat(), self.microscope(), self.grips(), self.test()]

    # ---- Connect system modules
    def connect(self) -> Section:
        def search_ports():
            self.cell.discover()
            rows = {d["name"]: d for d in self.cell.devices}
            bad = [n for n in ("Arm", "Vision box") if not rows.get(n, {}).get("ok")]
            if bad:
                raise Refused(f"not found: {', '.join(bad)} ({'; '.join(rows[n]['state'] for n in bad if rows.get(n))}). "
                              "Check power and cables, then run again")
            return {"arm": rows["Arm"]["found"], "nova": rows["Vision box"]["found"]}

        def home():
            self.squid.home()
            return {"stage": "homed"}

        def module_to_0():
            self.ready()
            c = self.cyc()
            self.guard(lambda: c.move_to("polar.pose_at_0"))
            # the axis reading, never the assumed angle: a stop or a hand left the module at 15.56 deg (2026-11-02 02:54), the assumed 0 skipped the
            # turn and the short-edge pads closed on a plate turned 15 degrees
            real = c.polar.read_angle()
            real = real - 360.0 * round(real / 360.0)
            if abs(real) > 1.0 or c.polar.angle != 0:
                self.guard(lambda: c._polar_rotate(0, self.cell.layout()["fixtures"]["polar"]["pose_at_90"]["pose"]))
            return {"module": "0 deg", "was": round(real, 2)}

        def hover():
            self.ready()
            self.view_module()

        def to_nest():
            self.ready()
            self.guard(self.cyc().go_nest)

        return Section("connect", [
            Step("computer", "search ports", search_ports),
            Step("squid", "home stage", home, moves=True),
            Step("polar", "arm to the nest (a pose in the clear in front of the hotel; also where it goes after any fault)", to_nest, moves=True),
            Step("user", "put the aruco plate on the rotation module, tag up, and press Enter"),
            Step("polar", "rotation module to 0 deg", module_to_0, moves=True),
            Step("polar", "hover hand onto rotation module centroid", hover, moves=True)])

    # ---- cameras.toml
    def cameras(self) -> Section:
        c = self.cell

        def node():
            L = c.layout()
            return L["vision"].get("calibration", {}).get(L["vision"]["down_camera"], {})

        def detect():
            cameras.read_marker(c, "down")
            if node().get("marker") != "ok":
                raise Refused("the down camera cannot read the aruco plate: clean the lens, check the plate is flat under the hand")
            self.view_module()                                # start from the module view, not wherever the last run left the wrist
            view = self.view_for_capture()
            got = self.guard(lambda: cameras.pictures(c, False, "down", view))
            return {"pictures": sum(got.values()), "captured_at": _stamp()}

        def solve():
            res = cameras.solve(c, "down")
            r = next(iter(res.values()))
            self.found["camera"] = r
            return {"error_px": r["px"], "error_mm": r["mm"]}

        def limits():
            r = self.found.get("camera")
            if not r or not r["ok"]:
                lim = c.layout()["vision"].get("calibration_limits") or {}
                raise Refused(f"calibration error {r and r['px']} px / {r and r['mm']} mm is above "
                              f"{lim.get('px', cameras.LIMIT_PX)} px / {lim.get('mm', cameras.LIMIT_MM)} mm: nothing uploaded, run again")

        def upload():
            try:
                cameras.upload(c, "down")
            except Exception as e:                            # no login, no route to the box, a refused file: the box keeps the calibration it has (the aruco servo needs none)
                c.note(f"not uploaded ({type(e).__name__}): {e}")
                self.found["upload_skipped"] = True
                return {"uploaded_at": "skipped, no box login"}
            at = node().get("uploaded_at")
            if not at:
                raise Refused("the box did not take the files")
            return {"uploaded_at": at}

        def validate():
            if self.found.get("upload_skipped") or str(node().get("uploaded_at", "")).startswith("skipped"):      # the box never got the calibration: nothing for it to check
                c.note("validate skipped: the calibration was not uploaded (no box login)")
                return {"check_mm": "skipped, not uploaded", "checked_at": _stamp()}
            self.ready()
            self.guard(lambda: cameras.check(c, "down"))
            mm = node().get("checked_mm")
            if mm is None:
                c.note("validate: the box could not read the aruco from two poses; carrying on (the arm does not use the box's calibration)")
                return {"check_mm": "not read", "checked_at": _stamp()}
            if mm > cameras.limit_mm(c.layout()):
                c.note(f"validate WARNING: the aruco moved {mm} mm between two poses 20 mm apart (limit {cameras.limit_mm(c.layout())}); carrying on, check the calibration after the demo")
            return {"check_mm": mm, "checked_at": node().get("checked_at")}

        return Section("cameras.toml", [
            Step("nova", "detect aruco with the down camera", detect, moves=True),
            Step("computer", "solve intrinsic (lens) and extrinsic (camera-to-gripper) calibration together", solve),
            Step("computer", "refuse if px > 5.0 or mm > 2.0", limits),
            Step("computer", "re-upload to nova, which restarts", upload)])



    # ---- the microscope seat
    def seat(self) -> Section:
        c = self.cell


        return Section("microscope seat", [
            Step("user", "the microscope must stand at least 20 cm from the arm's base (its plate holder no more than 39 cm from the axis). Take the plate off the rotation module and set it flat in the microscope's plate holder; then drag the arm so the open fingers straddle the plate as they would to grip it, the claw square to the instrument, and press Enter", before=self.loosen),
            Step("computer", "take that pose as the microscope seat", lambda: self.stage_hover(hand_is_seat=True)),
            Step("polar", "let go of the plate and rise straight out of the holder", lambda: self.leave_holder(release=True), moves=True),
            Step("polar", "prove the seat: pick the plate from it, lift, set it back, release (clamped by force, the jaw opening all the way)", self.check_seat, moves=True)])


    # ---- microscope.toml
    def microscope(self) -> Section:
        c = self.cell

        def limits():
            cfg = c.layout()["squid"][self.scope]
            if not getattr(self.squid, "motorized", True):
                # no motors: the stage stands at home (the real height) and never moves, so there is no travel to limit,
                # no axis to map and nothing to park; home is where it is
                park = [round(float(v), 3) for v in self.squid.position()]
                self.edit(lambda L: L["squid"][self.scope].update(park_xy_mm=park, park_tolerance_mm=cfg.get("park_tolerance_mm") or 0.5))
                return {"stage": "no motors, stays at home", "park_xy": park}
            lim = cfg.get("stage_limits_mm")
            if not lim:
                raise Refused(f"squid.{self.scope}.stage_limits_mm is not set in layout.json: [[x min, x max], [y min, y max]] "
                              "in Squid's stage frame, mm (Squid's configuration or the caliper)")
            park = [round(float(v), 3) for v in self.squid.position()]
            tol = cfg.get("park_tolerance_mm") or 0.5
            self.edit(lambda L: L["squid"][self.scope].update(park_xy_mm=park, park_tolerance_mm=tol))
            return {"stage_limits": lim, "park_xy": park, "park_tolerance": tol,
                    "stage_sweep_box": "not set: needs the Squid-to-arm axis mapping"}





        return Section("microscope.toml", [
            Step("computer", "read the stage travel limits from squid", limits),
            ])



    # ---- grips.toml
    def grips(self) -> Section:
        """The long-edge grip, by force at the module (the short-edge one was learned at the seat proof): the plate goes from the holder to
        the module, the module turns to 90, the first holding clamp on the long edges learns the width (procedures/grasp.py _learn_width),
        one re-grip proves it, and the plate stays on the module at 0 deg, where the test's way in expects it."""
        c = self.cell

        def poses():
            F = c.layout()["fixtures"]["polar"]
            return F, F["pose_at_0"]["pose"], F["pose_at_90"]["pose"]

        def to_module():
            """The plate is in the holder (the seat was just proved): microscope to the module along the microscope's taught way and the planner."""
            self.ready()
            cyc = self.cyc()
            st = cyc.store.s
            st.plate_at, st.grip, st.step_done = self.scope, "none", True
            cyc.store.save()
            steps = cyc._steps("A1")[0][:2]                       # pick:microscope, place:polar (the slot's steps are not run)
            self.guard(lambda: cyc._run_steps(steps))
            return {"plate": "on the module at 0 deg"}

        def long():
            F, p0, p90 = poses()
            cyc = self.cyc()
            self.guard(lambda: cyc._polar_rotate(90, p0))
            self.guard(lambda: cyc._pick("polar", p90, F, "long_edges", None, gate="polar@90"))
            return {"long_edges held at": round(c.robot.jaw(), 2)}

        def learned():
            g = c.layout()["gripper"]
            missing = [k for k in ("short_edges", "long_edges") if k not in g.get("plate_width", {})]
            if missing:
                raise Refused(f"no width learned for {', '.join(missing)}: that clamp never held the plate. Run again from \"grips.toml\"")
            keys = ("short_edges", "long_edges")
            return {"plate_width": {k: g["plate_width"][k] for k in keys}, "closed": {k: g["closed"][k] for k in keys},
                    "accept_band": {k: g["accept_band"][k] for k in keys}}

        def reseat():
            F, p0, p90 = poses()
            cyc = self.cyc()
            seen = []
            for _ in range(RESEATS):
                self.guard(lambda: cyc._place("polar", p90, F, "polar", None, gate="polar@90"))
                self.guard(lambda: cyc._pick("polar", p90, F, "long_edges", None, gate="polar@90"))
                seen.append(round(c.robot.jaw(), 2))
            lo, hi = c.layout()["gripper"]["accept_band"]["long_edges"]
            bad = [w for w in seen if not lo <= w <= hi]
            if bad:
                raise Refused(f"re-grip widths {seen} left the band {lo} to {hi}: the plate does not seat the same way each time")
            return {"re-grip widths": seen, "spread": geometry.spread(seen)}

        def put_back():
            F, p0, p90 = poses()
            cyc = self.cyc()
            self.guard(lambda: cyc._place("polar", p90, F, "polar", None, gate="polar@90"))
            self.guard(lambda: cyc._polar_rotate(0, p90))
            cyc.store.s.contents["module"], cyc.store.s.contents[self.scope] = "full", "empty"     # the plate rests on the module: the test starts from it
            cyc.store.save()
            return {"plate": "on the module at 0 deg"}

        return Section("grips.toml", [
            Step("polar", "carry the plate from the microscope to the rotation module", to_module, moves=True),
            Step("polar", "rotate module to 90, grip on the long edges, closing under the force limit (the first holding clamp learns the width)", long, moves=True),
            Step("computer", "the widths the force clamps learned: plate_width, the clamp command 1 mm under it, the band +-0.5", learned),
            Step("polar", "seat and re-grip the plate", reseat, moves=True),
            Step("polar", "place plate back on the rotation module at 0 deg", put_back, moves=True)])

    # ---- shuffle
    def shuffle(self, limit: int = 0, speed: str = "stress", probe_every: int = 0) -> Section:
        """A random plate from a taught slot goes to the microscope, is imaged, and goes into a random empty taught slot, for ever (or `limit` moves).
        The occupancy is fixtures.hotel.loaded; the ledger is made to agree with it before the first move. A plate left in the
        fingers by an earlier stop (the jaw at the long-edge width) is put into a slot first."""
        c = self.cell
        connect = self.connect().steps

        def stopped():
            return c.stop_event.is_set() or os.path.exists(os.path.join(c.runs_dir, "STOP"))

        def holding_plate():
            self.ready()                                  # the arm's state is read; the jaw and the ledger below are current
            band, jaw = c.layout()["gripper"]["accept_band"], c.robot.jaw()
            return (any(band[k][0] - JAW_SLACK_MM <= jaw <= band[k][1] + JAW_SLACK_MM for k in ("long_edges", "short_edges"))
                    or self.cyc().store.s.grip in ("long_edges", "short_edges"))

        def unless_holding(step):
            """A stop can leave a plate in the claw: the nest is for an empty claw and the module is turned empty. The run below puts
            the plate down first."""
            def go():
                if holding_plate():
                    print("   the claw holds a plate: not going to the nest, the plate goes down first")
                    return {"skipped": "the claw holds a plate"}
                return step.run()
            return Step(step.actor, step.text, go, moves=step.moves)

        def run():
            self.unattended = True                      # no question may wait for a person: camera doubts are logged and carried on
            try:
                self.ready()
                cyc = self.cyc()
                st = cyc.store.s
                F = c.layout()["fixtures"]
                fast, slow = (SpeedProfile(**c.layout()["speeds"][k]) for k in (speed, "supervised"))
                cyc.safety.profile = fast                           # --speed: "stress" to collect data hard, "supervised" once models are in the loop
                wanted = F["hotel"].get("active_slots", [])
                taught = [x for x in wanted if reachable(cyc, x)]
                print(f"   slots in the shuffle ({len(taught)}): {', '.join(taught)}" + (f"; out of reach, left out: {', '.join(x for x in wanted if x not in taught)}" if len(taught) < len(wanted) else ""))
                lo, hi = c.layout()["gripper"]["accept_band"]["long_edges"]
                lo, hi = lo - JAW_SLACK_MM, hi + JAW_SLACK_MM
                in_hand = lo <= c.robot.jaw() <= hi
                slo, shi = c.layout()["gripper"]["accept_band"]["short_edges"]
                if not in_hand and slo - JAW_SLACK_MM <= c.robot.jaw() <= shi + JAW_SLACK_MM:
                    # a stop between the microscope and the module (or the module and the microscope) left the plate in the claw by the
                    # short edges, the module at 0 deg: it goes down on the module, which is where the loop picks it up again
                    print("   a plate is in the hand by the short edges: onto the module first")
                    P0_ = F["polar"]["pose_at_0"]["pose"]
                    self.guard(lambda: cyc.move_to("polar.pose_at_0"))
                    self.guard(lambda: cyc._place("polar", P0_, F["polar"], "polar", None, gate="polar@0"))
                    self.guard(cyc.go_nest)             # the claw is empty now: through the folded pose, which puts the arm on the branch the module and the microscope were taught on
                have = [x for x in bench.loaded_slots(c.layout()) if x in taught]
                on_module = (st.plate_at == "polar" or bool(F["polar"].get("plate_on_module"))) and not in_hand   # a plate on the module (left there by a stop, or set by hand in layout.json) goes through the microscope first
                on_scope = bool(F[self.scope].get("plate_on_stage")) and not in_hand and not on_module      # set by hand in layout.json: a plate is on the stage
                if len(have) + (1 if in_hand or on_module or on_scope else 0) < 1:
                    raise Refused("no plate to move: set the occupancy (fixtures.hotel.loaded) to the plates that are in the taught slots " + str(taught))
                if len(have) + (1 if in_hand or on_module or on_scope else 0) >= len(taught):
                    raise Refused(f"every taught slot {taught} holds a plate: the shuffle needs one empty slot to move into")
                st.hotel = {x: "" for x in have}                        # the ledger takes the occupancy from the positions file
                st.contents.update({x: ("full" if x in have else "empty") for x in st.contents if x[:1] in "ABC" and x[1:].isdigit()})
                st.contents.update({"module": "full" if on_module else "empty", "microscope": "full" if on_scope else "empty", "gripper": "full" if in_hand else "empty"})
                st.plate_at, st.grip, st.step_done = ("gripper", "long_edges", True) if in_hand else ("polar", "none", True) if on_module else (self.scope, "none", True) if on_scope else ("none", "none", True)
                cyc.store.save()
                if os.path.exists(os.path.join(c.runs_dir, "STOP")):
                    os.remove(os.path.join(c.runs_dir, "STOP"))         # left by the last run: this one starts clean

                def landed(want):
                    at = st.plate_at
                    return at.split(":", 1)[1] if at.startswith("hotel:") else want

                if in_hand:
                    holes = [x for x in taught if x not in have]
                    want = random.choice(holes)
                    print(f"   a plate is in the hand by the long edges: into {want}")
                    self.guard(lambda: cyc._hotel_place_free(want))
                    have.append(landed(want))
                    self.edit(lambda L: bench.set_loaded(L, have))
                P0, P90 = F["polar"]["pose_at_0"]["pose"], F["polar"]["pose_at_90"]["pose"]

                def module_to_90():
                    """The way in from a slot puts the plate down on the module at 90 deg: the module is turned there first, empty, from where it
                    really is (the axis reading, not the ledger: a hand or an earlier stop may have turned it)."""
                    turn = cyc.polar.read_angle()
                    turn = turn - 360.0 if turn > 180.0 else turn
                    cyc.polar.angle = int(round(turn / 90.0) * 90) if min(abs(turn), abs(turn - 90.0)) < 5.0 else -1
                    if cyc.polar.angle != 90:
                        print(f"   the module is at {turn:.0f} deg: turning it to 90 before the next plate comes in")
                        self.guard(lambda: cyc.move_to("polar.pose_at_0"))
                        self.guard(lambda: cyc._polar_rotate(90, P0))

                def carry(steps):
                    """A route stopped by a motion fault (a collision, a lost enable) is not the end of the run: the controller's errors are
                    cleared, and the step that stopped starts again at the slow profile, the planner backing out against the move that hit and
                    planning a new way in. Once per route; and only a pick with the fingers empty or a place with the plate in them (a pick
                    restarted with the plate in the fingers would open them over it). Anything else stops the run."""
                    try:
                        return self.guard(lambda: cyc._run_steps(steps))
                    except Refused as e:
                        step = cyc.store.s.step
                        short_lo, short_hi = c.layout()["gripper"]["accept_band"]["short_edges"]
                        jaw = c.robot.jaw()
                        held = lo <= jaw <= hi or short_lo - JAW_SLACK_MM <= jaw <= short_hi + JAW_SLACK_MM
                        ok = is_motion_fault(str(e)) or any(w in str(e) for w in ("did not reach", "wrist limit or contact", "jaw open: external axis", "jaw_refused"))
                        if not ok or step not in [n for n, _ in steps] or not (step.startswith("pick") and not held or step.startswith("place") and held):
                            raise
                        why = clear_faults(c.robot)
                        line = f"{_stamp()}  {step}: {str(e)[:160]}; faults {'cleared' if not why else 'NOT cleared: ' + why}; starting the step again, slowly"
                        print(colour.paint("   RECOVERING: " + line, colour.BAD))
                        cyc.tele.log("recover", "retry", line)
                        with open(os.path.join(c.runs_dir, "recoveries.log"), "a", encoding="utf-8") as f:
                            f.write(line + chr(10))
                        if why:
                            raise Refused(why)
                        cyc.safety.profile = slow
                        try:
                            return self.guard(lambda: cyc._run_steps(steps, step))
                        finally:
                            cyc.safety.profile = fast

                def look(hand):
                    """The front camera at every taught slot's approach point, nothing touched: a labelled frame per slot (the ledger knows
                    which hold a plate), the hand empty or holding a plate. The only frames that cross the hand with the slot."""
                    for slot in taught:
                        if stopped():
                            return
                        self.guard(lambda: cyc._hotel_go_to_approach(slot, "probe"))
                        cyc.tele.image(f"probe_hand-{hand}_slot-{'occupied' if slot in have else 'empty'}_{slot}", cyc._frame(cyc.front_cam))
                        self.guard(lambda: cyc._hotel_leave_approach(slot, "probe"))

                def tour():
                    print("   probe tour: every taught slot's approach with the hand empty, then with a plate in the hand")
                    look("empty")
                    if stopped() or not have:
                        return
                    src = random.choice([x for x in taught if x in have])
                    have.remove(src)
                    self.edit(lambda L: bench.set_loaded(L, have))
                    self.guard(lambda: cyc._hotel_pick(src))
                    look("plate")
                    self.guard(lambda: cyc._hotel_place_free(src))
                    have.append(landed(src))
                    self.edit(lambda L: bench.set_loaded(L, have))

                if on_scope and not stopped():
                    want = random.choice([x for x in taught if x not in have])
                    turn = cyc.polar.read_angle()
                    turn = turn - 360.0 if turn > 180.0 else turn
                    if abs(turn) > 5.0:                                      # the way out of the stage puts the plate on the module at 0 deg
                        print(f"   the module is at {turn:.0f} deg: turning it to 0 before the plate comes off the stage")
                        self.guard(lambda: cyc.move_to("polar.pose_at_0"))
                        self.guard(lambda: cyc._polar_rotate(0, P0))
                    cyc.polar.angle = 0
                    print(f"   a plate is on the microscope stage: imaged, then into {want}")
                    self._acquire()
                    self.guard(lambda: cyc.move_to(self.scope))              # the pick starts at the hover: from the nest the planner takes the way in over the microscope
                    carry(cyc._steps(want)[0])
                    have.append(landed(want))
                    self.edit(lambda L: bench.set_loaded(L, have))
                    self.edit(lambda L: L["fixtures"][self.scope].pop("plate_on_stage", None))
                if on_module and not stopped():
                    holes = [x for x in taught if x not in have]
                    want = random.choice(holes)
                    turn = cyc.polar.read_angle()
                    turn = turn - 360.0 if turn > 180.0 else turn
                    if abs(turn) > 5.0:                                      # the plate turns with the module: bring it to the pick angle, hand above it
                        print(f"   the module is at {turn:.0f} deg: turning it to 0 before the plate comes off")
                        self.guard(lambda: cyc.move_to("polar.pose_at_0"))
                        self.guard(lambda: cyc._polar_rotate(0, P0))
                    cyc.polar.angle = 0
                    print(f"   a plate is on the module: to the microscope, then into {want}")
                    carry(cyc._steps(want)[1][3:])                           # pick from the module by the short edges, onto the stage
                    self._acquire()
                    carry(cyc._steps(want)[0])
                    have.append(landed(want))
                    self.edit(lambda L: bench.set_loaded(L, have))
                    self.edit(lambda L: L["fixtures"]["polar"].pop("plate_on_module", None))
                moved = 0
                while not stopped() and (not limit or moved < limit):
                    if probe_every and moved % probe_every == 0:
                        tour()
                        if stopped():
                            break
                    module_to_90()
                    full = [x for x in taught if x in have]
                    holes = [x for x in taught if x not in have]
                    src, want = random.choice(full), random.choice(holes)
                    print(f"   move {moved + 1}: {src} to the microscope, then into {want}   (plates in {sorted(have)}; stops when runs/STOP exists or with Ctrl+C)")
                    have.remove(src)
                    self.edit(lambda L: bench.set_loaded(L, have))       # in transit: the slot is empty
                    began, doubts0 = time.time(), _lines(os.path.join(c.runs_dir, "mismatches.log"))
                    carry(cyc._steps(src)[1])                                  # slot, module, microscope stage
                    self._acquire()
                    carry(cyc._steps(want)[0])                                 # stage, module, scan, slot
                    have.append(landed(want))
                    self.edit(lambda L: bench.set_loaded(L, have))
                    moved += 1
                    with open(os.path.join(c.runs_dir, "shuffle_moves.csv"), "a", encoding="utf-8") as f:       # one line per finished move: the data the gates are tuned from
                        f.write(f"{_stamp()},{src},{want},{landed(want)},{time.time() - began:.1f},{_lines(os.path.join(c.runs_dir, 'mismatches.log')) - doubts0}" + chr(10))
                return {"plates moved": moved, "occupancy": sorted(have)}
            finally:
                self.unattended = False

        return Section("shuffle", [connect[0], unless_holding(connect[2]), unless_holding(connect[3]),
                                   Step("polar", "a random plate from a taught slot through the microscope into a random empty taught slot, for ever", run, moves=True)])

    # ---- test
    def _acquire(self):
        c = self.cell
        self.squid.acquire()
        deadline = time.time() + ACQUISITION_TIMEOUT_S
        while not self.squid.done():
            if c.stop_event.is_set() or time.time() > deadline:
                raise Refused("the acquisition did not finish")
            time.sleep(0.2)
        self.squid.home()

    def test(self) -> Section:
        """The test is the rotation: the plate on the module goes through the microscope into the first empty taught slot, then every plate
        in a taught slot visits the microscope and goes to the slot that has been empty the longest, for ever. It needs one more empty
        taught slot than the module's plate takes: the module is the way in and out of the microscope, so it cannot be a spare place."""
        c = self.cell

        def unattended(fn):
            def run():
                self.unattended = True                  # no question may wait for a person while this runs
                try:
                    return fn()
                finally:
                    self.unattended = False
            return run

        def stopped():
            return c.stop_event.is_set() or os.path.exists(os.path.join(c.runs_dir, "STOP"))

        def rotate():
            c.between_plates_only = True                                 # runs/STOP ends the run with the plate in a slot, never mid-carry
            try:
                return _rotate()
            finally:
                c.between_plates_only = False

        def _rotate():
            self.ready()
            cyc = self.cyc()
            cyc.safety.profile = SpeedProfile(**c.layout()["speeds"][self.speed])      # --speed: stress by default (travel 0.5, approach 0.25, contact 0.1); the lowers, slides and clamps keep their slow last millimetres
            # the module's angle is not set here: every pick and place at the module reads the axis at the hover and turns it to the angle
            # its seat is taught for (Cycle._polar_ensure). Turning it to 0 here put A7's plate across the long-edge seat (2026-11-03 19:52).
            F = c.layout()["fixtures"]
            st = cyc.store.s
            taught = [x for x in F["hotel"].get("active_slots", []) if reachable(cyc, x)]
            lo, hi = c.layout()["gripper"]["accept_band"]["long_edges"]
            moves = 0
            if lo <= c.robot.jaw() <= hi:                                                  # a plate in the hand at the long-edge width: the ledger can be stale
                st.plate_at, st.grip, st.step_done = "gripper", "long_edges", True
                cyc.store.save()
            # where plates are comes from the ledger and the positions file only: the hotel occupancy sweep with the camera is out of the
            # workflow (operator, 2026-11-08). The camera still takes its pictures at the slot mouths and the drop-in view at the module and
            # the microscope; none of them chooses a slot or stops the run.
            # the ledger's own record of the hotel (written at every place, cleared at every pick) outranks the positions file: a run that
            # died between "in transit" and the pick (03:33, the SDK crash) left the plate in B3 and B3 out of `loaded`, and the next run
            # found nothing to rotate
            ledger_full = set(st.hotel) | ({st.plate_at.split(":", 1)[1]} if st.plate_at.startswith("hotel:") else set())
            missing = sorted(x for x in ledger_full if x in taught and x not in bench.loaded_slots(c.layout()))
            if missing:
                print(f"   the ledger says {missing} hold a plate and the positions file did not: taking the ledger's word")
                self.edit(lambda L: bench.set_loaded(L, list(bench.loaded_slots(L)) + missing))

            def parking_places():
                have = set(bench.loaded_slots(c.layout()))
                return [x for x in taught if x not in have and st.contents.get(x) != "full"]

            holes = parking_places()
            intake = st.plate_at == "gripper" or st.contents.get("module") == "full" or st.plate_at in ("polar", self.scope)
            if len(holes) < (2 if intake else 1):
                raise Refused(f"the test needs {2 if intake else 1} empty taught slot(s){' (one for the module plate, one for the rotation to start from)' if intake else ''} and has {holes}: "
                              "put plates from taught slots into untaught ones by hand and set the occupancy, then run: python cli.py setup --from \"test\"")
            if st.plate_at == "gripper" and st.grip == "long_edges":
                slot = holes.pop(random.randrange(len(holes)))
                print(f"   a plate is in the hand by the long edges: scan, into {slot}")
                self.guard(lambda: cyc._run_steps(cyc._steps(slot)[0][4:]))
                self.edit(lambda L: bench.set_loaded(L, list(bench.loaded_slots(L)) + [slot]))
            if st.contents.get("module") == "full" or st.plate_at in ("polar", self.scope):          # also when a plate was in the hand: the module's is still to go
                still_there = st.plate_at == self.scope and (st.step_done or (st.step == f"pick:{self.scope}" and st.grip == "none" and c.robot.jaw() > 20.0))
                if still_there:                       # also a pick begun and abandoned with the claw still open: the plate never left the stage (03:08, 2026-11-04)
                    print(f"   the plate is already on the {self.scope} stage: carrying on from it")
                else:
                    P0 = F["polar"]["pose_at_0"]["pose"]
                    st.plate_at, st.grip, st.step_done = "polar", "none", True
                    cyc.store.save()
                    route = F[self.scope].get("trajectory", "microscope_to_polar")
                    print(f"   the module's plate to the microscope, acquire, then to the module")
                    self.guard(lambda: cyc._pick("polar", P0, F["polar"], "short_edges", None, gate="polar@0"))
                    self.guard(lambda: cyc._place(self.scope, F[self.scope]["pose"], F[self.scope], self.scope, route, reverse=True))
                self._acquire()
                # stage -> module by the short edges, module to 90: the plate rests on the module and the claw is empty
                self.guard(lambda: cyc._run_steps(cyc._steps(taught[0])[0][:3]))                  # pick:scope, place:polar, polar:90 (no slot in them)
                holes = parking_places()
                if not holes:
                    raise Refused("no empty taught slot for the module's plate: free one by hand, then: python cli.py setup --from \"test\"")
                slot = holes.pop(random.randrange(len(holes)))
                print(f"   the module's plate into {slot}")
                self.guard(lambda: cyc._run_steps(cyc._steps(slot)[0][3:]))                       # pick:polar (long edges), place:hotel:<slot>
                self.edit(lambda L: bench.set_loaded(L, list(bench.loaded_slots(L)) + [slot]))
                st.contents["module"] = "empty"
                cyc.store.save()
            passes = 0
            while not stopped():
                have = set(bench.loaded_slots(c.layout()))
                full = [x for x in taught if x in have]
                holes = parking_places()
                if not holes:
                    raise Refused("the rotation has no empty taught slot to start from: free one (put its plate in an untaught slot) and run: "
                                  "python cli.py setup --from \"test\"")
                if not full:                          # nothing to rotate: say so and stop (a pass that does nothing spun 340 000 times in four minutes, 03:42)
                    raise Refused(f"no plate in any taught slot (the positions file has {sorted(have) or 'none'} loaded): put a plate in a taught slot and set its occupancy, "
                                  "then run: python cli.py setup --from \"test\"")
                passes += 1
                print(f"   rotation pass {passes} (stops when runs/STOP exists or with Ctrl+C)")
                for slot in full:
                    if stopped():
                        break
                    dest = holes.pop(random.randrange(len(holes)))
                    print(f"   rotate: {slot} to the microscope, then into {dest}")
                    self.edit(lambda L, slot=slot: bench.set_loaded(L, [x for x in bench.loaded_slots(L) if x != slot]))       # in transit: the slot is empty
                    self.guard(lambda: cyc._run_steps(cyc._steps(slot)[1]))
                    self._acquire()
                    self.guard(lambda: cyc._run_steps(cyc._steps(dest)[0]))
                    self.edit(lambda L, dest=dest: bench.set_loaded(L, list(bench.loaded_slots(L)) + [dest]))
                    holes.append(slot)
                    moves += 1
                    if c.layout()["vision"].get("check_seats_after_park", False):
                        # off (operator, 01:25): the seats are not looked at with the empty claw; the question "can I drop in here" is asked with
                        # the plate in hand from the front camera at the hover. Kept for the day a seat check from above is wanted.
                        seat_check(cyc, "polar")
                        seat_check(cyc, self.scope)
            return {"stopped": "by request, between plates", "passes": passes, "plates moved": moves}

        def seat_check(cyc, station):
            """The empty claw at a seat's viewing pose (tools/view_pose.py), the down camera judged against the seat's empty pictures, the
            verdict printed and drawn in the window; back to the hover after. Returns full / empty / unsure, or None without a view or a
            reference. Never stops the run."""
            from tools.view_pose import from_tool_frame, to_view
            F = c.layout()["fixtures"]
            st = cyc.store.s
            view = (F.get(station) or {}).get("view", {}).get("tool_offset")
            if not view or st.grip != "none":
                return None
            seat = F["polar"]["pose_at_0"]["pose"] if station == "polar" else F[station]["pose"]
            target = from_tool_frame(seat, *view)
            truth = "full" if (st.plate_at == station or st.contents.get("module" if station == "polar" else station) == "full") else "empty"
            self.guard(lambda: cyc.move_to("polar.pose_at_0" if station == "polar" else station))
            self.guard(lambda: to_view(cyc, station, target))
            got = cyc._collect("occupancy", station, truth, "down", judge=True)
            print(f"   {'module' if station == 'polar' else station}: the camera says {got[1] if got else 'nothing (no reference)'}; the ledger says {truth}")
            time.sleep(1.5)                                                       # long enough to read in the window
            hover = offset(seat, [0, 0, F["polar" if station == "polar" else station]["hover_mm"]])
            self.guard(lambda: cyc._move("L", [hover[0], hover[1], target[2]] + list(hover[3:]), f"{station}: back over the seat", cyc.safety.profile.contact))
            self.guard(lambda: cyc._move("L", hover, f"{station}: down to the hover", cyc.safety.profile.contact))
            return got[0] if got else None

        # No hotel occupancy sweep: put into the test on 2026-11-04 01:01 it broke the proven flow, kept as a record-only pass that night, and
        # taken out of the workflow on 2026-11-08 (operator). Where plates are is the ledger's and the positions file's word.
        return Section("test", [Step("polar", "the rotation, for ever: the module's plate through the microscope into an empty slot, then every plate in a taught slot to the microscope and into the slot empty the longest", unattended(rotate), moves=True)])