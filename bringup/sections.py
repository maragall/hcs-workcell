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
from bringup import bench, cameras, colour, hand, servo
from perception.hotel_rows import HotelRows
from procedures import cad_poses
from procedures.corrector import StopSafe
from procedures.cycle import Cycle
from procedures.motion import offset
from procedures.safety import SafetyViolation
from procedures.state import StateStore
from procedures.telemetry import Telemetry

CARRY_ON = ("safe to lower", "clear to lower", "clear to rotate", "fine to continue", "seated correctly", "held correctly")      # the camera gates' questions: the setup answers yes and logs the mismatch
SEAT_MOVED_KEEP_MM = 1.0          # closer than this the taught seat stays: the read has that much noise
SEARCH_HEIGHTS = 4                # heights tried below the microscope hover, highest allowed first
BAND_BELOW, BAND_ABOVE = 0.1, {"short_edges": 0.5, "long_edges": 1.05}     # as gripper.accept_band was set 2026-10-22
RESEATS = 1
ACQUISITION_TIMEOUT_S = 600
VIEW_DISTANCE_MM = 140.0         # camera height above the tag for the capture grid (calibration/plan.py tag_in_view)
TAG_VIEW_MARGIN_MM = 5.0         # headroom under the workspace top for the stage view
SEAT_MOVED_DROP_ROUTE_MM = 8.0       # a hover this far from the taught seat: the taught route belongs to another place
STAGE_SEARCH_MM = ((0, 0), (35, 0), (-35, 0), (0, 35), (0, -35), (35, 35), (-35, 35), (35, -35), (-35, -35),
                   (70, 0), (-70, 0), (0, 70), (0, -70))        # where the arm looks for the tag around the hover, x and y
MIN_STAGE_FROM_ARM_MM = 250.0     # the Squid base must be at least this far from the arm's base
TAG_AHEAD_MM = 71.5              # both tag views are this far ahead of the grip pose along the claw: the tag centre is the plate centre (186.5 ahead of the wrist) and the camera looks 115 ahead of it; measured 2026-10-28 (the old 40 left the tag cut off at the top of the picture)
SIZE_ROWS = (2, 4)                # rows where a 15-slot hotel has no shelf and a 30-slot one has
VIEW_SEARCH_MM = ((0, 0), (25, 0), (-25, 0), (0, 25), (0, -25), (25, 25), (-25, -25), (25, -25), (-25, 25), (50, 0), (-50, 0))   # along, across the claw
VIEW_LIFTS_MM = (0, 60, 120)      # heights above the seat hover the view is tried at (the camera has to focus on the plate)


def reachable(cyc: Cycle, slot: str) -> bool:
    """The slot's seat, its approach point and its carry height are all inside the workspace box and above their floors."""
    try:
        for name, pose in zip(("seat", "approach", "align", "carry"), cyc._hotel_geometry(slot)):
            cyc.safety.check_target(pose, f"slot {slot} {name}", allow_floor_regions=("hotel",))
        return True
    except SafetyViolation:
        return False


def _stamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


class Bringup:
    def __init__(self, cell, squid, ask: Callable[[str], str], scope: str = "microscope", confirm_stage: bool = False):
        self.cell, self.squid, self.ask, self.scope = cell, squid, ask, scope
        self.confirm_stage = confirm_stage     # the operator has said, for this run, that lowering at the stage is safe
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
        cyc.store.save()
        self.guard(lambda: cyc._pick(self.scope, F[self.scope]["pose"], F[self.scope], "short_edges", None))
        held = round(c.robot.jaw(), 2)
        self.guard(lambda: cyc._place(self.scope, F[self.scope]["pose"], F[self.scope], self.scope, None))
        return {"gripped at": held, "put back and released": f"jaw {round(c.robot.jaw(), 2)}"}

    def tag_height(self) -> float:
        """Where the module's tag reference is taken from: straight above the module's grip pose, as high as the same view over
        the stage still fits under the workspace top (the stage seat is higher than the module's by SEAT_RISE). Both pictures
        are then taken from straight above a grip pose at the same height over the plate, which the microscope zone allows
        (only vertical moves there), and the tag, 71 mm ahead of the camera's axis at the seat, is in the picture."""
        L = self.cell.layout()
        rise = L["fixtures"][self.scope]["pose"][2] - L["fixtures"]["polar"]["pose_at_0"]["pose"][2]
        return round(L["workspace_box"]["z"][1] - rise - TAG_VIEW_MARGIN_MM, 1)

    @staticmethod
    def tag_view_xy(pose: List[float]) -> List[float]:
        """Where the camera is taken to see the tag: TAG_AHEAD_MM ahead of a grip pose along the claw."""
        a = math.radians(pose[5])
        return [pose[0] + TAG_AHEAD_MM * math.cos(a), pose[1] + TAG_AHEAD_MM * math.sin(a)]

    def _servo(self) -> "servo.Servo":
        """The aruco servo (bringup/servo.py): the down camera's picture of the plate tag and small checked moves of the wrist. It
        needs no camera calibration: the tag's place is the arm's pose when the tag looks as it did at the reference."""
        c, v = self.cell, self.cell.layout()["vision"]
        cyc = self.cyc()
        def keep(frame, corners):                              # every picture the servo looks at, for the page that shows them
            d = os.path.join(c.runs_dir, "servo_last")
            os.makedirs(d, exist_ok=True)
            n = len(os.listdir(d)) // 2
            open(os.path.join(d, f"{n:03d}.jpg"), "wb").write(frame)
            json.dump(None if corners is None else [[float(a), float(b)] for a, b in corners], open(os.path.join(d, f"{n:03d}.json"), "w"))
        read = servo.corner_reader(lambda: c.vision.frame_full(v["down_camera"], v["led"]), keep=keep)

        def move(dx, dy, dyaw):
            h = c.robot.pose()
            self.guard(lambda: cyc.go_to([h[0] + dx, h[1] + dy, h[2], h[3], h[4], h[5] + dyaw], "servo"))
            p = c.robot.pose()
            return (p[0], p[1], p[5])
        return servo.Servo(read, move, get_pose=lambda: (lambda p: (p[0], p[1], p[5]))(c.robot.pose()))

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

    @staticmethod
    def _tag_orientation(corners) -> str:
        """Which way round the plate sits, as the nearest quarter turn of the tag's first edge in the picture ("0", "90", "180", "270").
        The plate turned half a turn on the module is its twin: a different reference."""
        d = np.asarray(corners, float)[1] - np.asarray(corners, float)[0]
        return str(int(round(math.degrees(math.atan2(d[1], d[0])) / 90.0)) * 90 % 360)

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
        return [self.connect(), self.cameras(), self.hotel(), self.swap(), self.grips(), self.microscope(),
                self.clear_stage(), self.system_state(), self.test()]

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
            if c.polar.angle != 0:
                self.guard(lambda: c._polar_rotate(0, self.cell.layout()["fixtures"]["polar"]["pose_at_90"]["pose"]))
            return {"module": "0 deg"}

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
            except cameras.calib_upload_errors() as e:        # no box login: the box keeps the calibration it has (the aruco servo needs none)
                c.note(f"not uploaded: {e}")
                return {"uploaded_at": "skipped, no box login"}
            at = node().get("uploaded_at")
            if not at:
                raise Refused("the box did not take the files")
            return {"uploaded_at": at}

        def validate():
            if str(node().get("uploaded_at", "")).startswith("skipped"):      # the box never got the calibration: nothing for it to check
                c.note("validate skipped: the calibration was not uploaded (no box login)")
                return {"check_mm": "skipped, not uploaded", "checked_at": _stamp()}
            self.ready()
            self.guard(lambda: cameras.check(c, "down"))
            mm = node().get("checked_mm")
            if mm is None:
                raise Refused("the check could not read the aruco from two poses")
            if mm > cameras.LIMIT_MM:
                raise Refused(f"the aruco moved {mm} mm between two poses 20 mm apart (limit {cameras.LIMIT_MM}): calibrate again")
            return {"check_mm": mm, "checked_at": node().get("checked_at")}

        return Section("cameras.toml", [
            Step("nova", "detect aruco with the down camera", detect, moves=True),
            Step("computer", "solve intrinsic (lens) and extrinsic (camera-to-gripper) calibration together", solve),
            Step("computer", "refuse if px > 5.0 or mm > 2.0", limits),
            Step("computer", "re-upload to nova, which restarts", upload),
            Step("nova", "validate. detect aruco from two poses", validate, moves=True)])

    # ---- hotel.toml
    def hotel(self) -> Section:
        c = self.cell

        def to_nest():
            self.ready()
            self.guard(self.cyc().go_nest)

        def look():
            cyc = self.cyc()
            img = cyc._frame(cyc.front_cam)
            if img is None:
                raise Refused("the front camera gave no picture")
            cyc.tele.image("hotel_view", img)
            return {"front camera": "1 picture from the nest, for the log"}

        def rows():
            """15 or 30 slots by looking, one row at a time: the camera is taken to the slot position of an even row (a 15-slot
            hotel has no shelf there) and the cell is asked 'is there a slot here, yes or no' until it has seen enough to tell
            by itself. One picture of the whole hotel cannot tell."""
            cyc = self.cyc()
            seen = HotelRows(os.path.join(c.ref_dir, "hotel_rows"))
            present = {}
            for row in SIZE_ROWS:
                slot = f"B{row}"
                self.ready()
                self.guard(lambda slot=slot: cyc._hotel_go_to_approach(slot, "setup:hotel size"))
                img = cyc._frame(cyc.front_cam)
                if img is None:
                    raise Refused("the front camera gave no picture")
                cyc.tele.image(f"hotel_row_{row}", img)
                verdict = seen.verdict(row, img)
                if verdict is None:
                    said = self.ask(f"      row {row}: is there a slot in front of the camera? yes or no: ").strip().lower()
                    if said not in ("yes", "no"):
                        raise Refused(f"row {row}: {said!r} is neither yes nor no")
                    seen.learn(row, img, said == "yes")
                    verdict = said == "yes"
                present[row] = verdict
            if all(present.values()):
                size = 30
            elif not any(present.values()):
                size = 15
            else:
                raise Refused(f"the rows disagree ({present}): the hotel has slots in some even rows and not in others")
            at = _stamp()
            self.found["hotel_size"] = size
            self.edit(lambda L: (bench.set_hotel_size(L, size),
                                 self.setup_node(L).update(hotel={"size": size, "rows_seen": {str(k): v for k, v in present.items()},
                                                                  "detected_at": at})))
            return {"size": size, "rows with a slot": ", ".join(str(r) for r, v in present.items() if v) or "none", "detected_at": at}

        return Section("hotel.toml", [
            Step("polar", "arm to the nest, the hotel view", to_nest, moves=True),
            Step("nova", "image the hotel with the front camera", look),
            Step("polar", "camera to the slot position of each even row, one at a time; is there a slot here, yes or no", rows, moves=True)])

    # ---- swap the plates
    def swap(self) -> Section:
        c = self.cell

        def read_plate():
            """The aruco plate on the module, by servo: its tag as the camera sees it at the view pose is the reference for the
            stage; on a later run the servo finds the tag again and the difference is how far the plate moved."""
            self.ready()
            L = c.layout()
            P0 = L["fixtures"]["polar"]["pose_at_0"]["pose"]
            grip = (P0[0], P0[1], P0[5])
            self.view_module()                                           # finds the plate; then straight above its grip pose, high
            cyc = self.cyc()
            for turned in (False, True):
                vx, vy = self.tag_view_xy(P0)
                self.guard(lambda: cyc.go_to([vx, vy, self.tag_height(), P0[3], P0[4], P0[5]], "tag view"))
                s = self._servo()
                try:
                    corners, view = s.observe()
                    break
                except servo.ServoError as e:
                    if turned:
                        raise Refused(f"the tag is not whole in the picture from straight above the module's grip pose, in either "
                                      f"orientation of the plate: {e}")
                    # the tab with the tag is at the far end of the plate: the module turns the plate half a turn (the arm clears it)
                    self.guard(lambda: cyc._polar_rotate(180, P0))
            node = self.setup_node(L)
            refs = dict(node.get("module_tags") or {})
            if node.get("module_tag"):                                                   # the single reference of earlier runs
                refs.setdefault(self._tag_orientation(np.array(node["module_tag"]["corners"], float)), node["module_tag"])
            bucket = self._tag_orientation(corners)           # which way round the plate sits, from how the tag looks (0/90/180/270)
            prev = refs.get(bucket)
            if prev:                                    # a reference taken from another view can never be matched: take a new one
                pv, now_z = prev.get("view") or [0, 0, 0], float(c.robot.pose()[2])
                if (math.hypot(pv[0] - view[0], pv[1] - view[1]) > 15.0 or abs(((pv[2] - view[2] + 180) % 360) - 180) > 8.0
                        or abs(float(prev.get("view_z", now_z)) - now_z) > 3.0):
                    c.note("the stored tag reference was taken from a different view: taking a new one")
                    prev = None
            try:
                if prev:
                    got = s.align(np.array(prev["corners"], float), start_pose=view)
                    offset_ = [round(v, 3) for v in servo.plate_offset(tuple(prev["view"]), got.pose, grip)]
                    ref, ref_view = np.array(prev["corners"], float), tuple(prev["view"])
                else:
                    offset_, ref, ref_view = [0.0, 0.0, 0.0], corners, view
            except servo.ServoError as e:
                raise Refused(f"the servo could not find the plate tag again on the module: {e}")
            tag = {"corners": ref.tolist(), "view": list(ref_view), "grip": list(grip),
                   "view_z": round(float(c.robot.pose()[2]), 3), "grip_z": float(P0[2])}
            self.found.update(module_tag=tag, plate_offset=offset_)

            def write(L):
                n = self.setup_node(L)
                n.update(plate_offset=offset_, plate_offset_at=_stamp())
                held = dict(n.get("module_tags") or {})
                for k, v in refs.items():
                    held.setdefault(k, v)                      # every orientation seen keeps its own reference
                if not prev:
                    held[bucket] = dict(tag, taken_at=_stamp())
                n["module_tags"] = held
                n["module_tag"] = held[bucket]                 # the one the stage steps carry over
            self.edit(write)
            return {"plate_offset": offset_, "tag": "reference taken" if not prev else "found again"}

        return Section("swap", [
            Step("user", "take the plate off the rotation module and set it flat in the microscope's plate holder (the aruco plate or a real one); then drag the arm "
                         "so the open fingers straddle the plate as they would to grip it, the claw square to the instrument, and press Enter", before=self.loosen),
            Step("computer", "take that pose as the microscope seat", lambda: self.stage_hover(hand_is_seat=True)),
            Step("polar", "let go of the plate and rise straight out of the holder", lambda: self.leave_holder(release=True), moves=True),
            Step("polar", "prove the seat: pick the plate from it, lift, set it back, release (the grip width in its band, the jaw opening all the way)", self.check_seat, moves=True),
            Step("user", "place a real plate on the rotation module")])

    # ---- grips.toml
    def grips(self) -> Section:
        c = self.cell
        widths: dict = {}

        def poses():
            F = c.layout()["fixtures"]["polar"]
            return F, F["pose_at_0"]["pose"], F["pose_at_90"]["pose"]

        def corrected(p0):
            dx, dy, dyaw = self.found.get("plate_offset") or c.layout().get("setup", {}).get("plate_offset") or (0, 0, 0)
            p = offset(p0, [dx, dy, 0.0])
            p[5] = p0[5] + dyaw
            return p

        def short():
            self.ready()
            F, p0, _ = poses()
            cyc = self.cyc()
            self.leave_holder(release=True)                      # a plate may still be in the fingers at the microscope seat
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            if cyc.polar.driver == "ext_axis" and abs(((cyc.polar.read_angle() + 180.0) % 360.0) - 180.0) > 3.0:
                print(f"   the module reads {cyc.polar.read_angle():.1f} deg: turning it to the short-edge grip (0) first")
                self.guard(lambda: cyc._polar_rotate(0, p0))
                self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            self.guard(lambda: cyc._pick("polar", corrected(p0), F, "short_edges", None, gate="polar@0"))
            widths["short_edges"] = round(c.robot.jaw(), 2)
            return {"short_closed": widths["short_edges"]}

        def grip_offset():
            size = self.found.get("hotel_size") or c.layout().get("setup", {}).get("hotel", {}).get("size")
            back = c.layout()["fixtures"]["hotel"].get("grip_back_mm", {}).get(str(size))
            if back is None:
                raise Refused(f"the {size}-slot hotel has no grip_back_mm in layout.json (fixtures.hotel). Measure how far "
                              "back the plate is gripped for it with the caliper and enter it")
            self.found["grip_offset"] = back
            return {"grip_offset": back}

        def long():
            F, p0, p90 = poses()
            cyc = self.cyc()
            self.guard(lambda: cyc._place("polar", corrected(p0), F, "polar", None, gate="polar@0"))
            self.guard(lambda: cyc._polar_rotate(90, p0))
            self.guard(lambda: cyc._pick("polar", p90, F, "long_edges", None, gate="polar@90"))
            widths["long_edges"] = round(c.robot.jaw(), 2)
            return {"long_closed": widths["long_edges"]}

        def band():
            def write(L):
                g = L["gripper"]
                for grip, w in widths.items():
                    g["closed"][grip] = w
                    g["accept_band"][grip] = geometry.accept_band(w, BAND_BELOW, BAND_ABOVE[grip])
                self.setup_node(L).update(grips={"short_closed": widths["short_edges"], "long_closed": widths["long_edges"],
                                                 "grip_offset": self.found["grip_offset"], "measured_at": _stamp()})
            self.edit(write)
            g = c.layout()["gripper"]["accept_band"]
            return {"accept_band": {k: g[k] for k in widths}, "measured_at": _stamp()}

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

        return Section("grips.toml", [
            Step("polar", "grip plate on the short edges at the module pose corrected by plate_offset, close until stall", short, moves=True),
            Step("computer", "choose the long-edge grip offset for the hotel size", grip_offset),
            Step("polar", "rotate module to 90, grip on the long edges, close until stall", long, moves=True),
            Step("computer", "accept band around each closed width", band),
            Step("polar", "carry, seat and re-grip the plate three times", reseat, moves=True),
            Step("polar", "place plate back on the rotation module at 0 deg", put_back, moves=True)])

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

        def find_marker():
            """The stage tag, by the servo's measure-only mode: from the taught hover the arm goes straight up or down (the only
            moves the microscope zone allows) to the height where the camera sees the tag at the scale it had at the module,
            reads it, and the offset from the module's picture says where the plate stands, to a fraction of a millimetre."""
            self.ready()
            L = c.layout()
            tag = self.found.get("module_tag") or self.setup_node(L).get("module_tag")
            if not tag:
                raise Refused("needs the aruco tag taken on the module first (the swap section)")
            cyc = self.cyc()
            if not self._parked():
                raise Refused("the stage is not parked: home Squid first")
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))          # the taught route starts at the module hover
            self.guard(lambda: cyc.move_to(self.scope))
            hx, hy, hz, rx, ry, hyaw = cyc._position(self.scope)
            seat = L["fixtures"][self.scope]["pose"]
            z_same = seat[2] + (tag["view_z"] - tag["grip_z"])        # the module view's height above the plate, over the stage plate
            top = L["workspace_box"]["z"][1]
            heights = [z_same] if hz - 1 <= z_same <= top else []     # the same height over the plate as at the module: the same picture scale
            if not heights:
                raise Refused(f"the module view's height over the plate would be z {z_same:.0f} over the stage, outside {hz:.0f}..{top:.0f}")
            s = self._servo()
            ref = np.array(tag["corners"], float)
            got, last = None, "the tag is not in the picture"
            self.guard(lambda: cyc._move("L", [hx, hy, heights[0], rx, ry, hyaw], f"setup:up to z{heights[0]:.0f}", cyc.safety.profile.approach))
            a = math.radians(hyaw)                                    # forward is the way the camera looks: along the claw, as at the module
            ahead, side = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
            bx, by = seat[0] + TAG_AHEAD_MM * ahead[0], seat[1] + TAG_AHEAD_MM * ahead[1]     # where the camera sees the plate centre, as at the module
            for dx, dy in STAGE_SEARCH_MM:                            # dx along the claw, dy across it
                x, y = bx + dx * ahead[0] + dy * side[0], by + dx * ahead[1] + dy * side[1]
                if L.get("volumes") and not 100.0 <= math.hypot(x - 126.0, y) <= 395.0:
                    continue                                          # out of the arm's reach: not a place to look
                wb = L["workspace_box"]
                if not (wb["x"][0] <= x <= wb["x"][1] and wb["y"][0] <= y <= wb["y"][1]):
                    continue                                          # outside the workspace box: the arm refuses it
                self.guard(lambda x=x, y=y: cyc._move("L", [x, y, heights[0], rx, ry, hyaw], f"setup:search {dx:+.0f} along {dy:+.0f} across", cyc.safety.profile.approach))
                try:
                    corners, view = s.observe()
                except servo.ServoError as e:
                    last = str(e)
                    continue
                try:
                    aligned = s.align(ref, start_pose=view)           # sideways moves are free this high above the instrument
                except servo.ServoError as e:
                    last = str(e)
                    print(f"      tag seen at {dx:+.0f} along {dy:+.0f} across, not aligned: {last}")
                    continue
                got = (heights[0], aligned)
                break
            self.guard(lambda: cyc._move("L", [hx, hy, heights[0], rx, ry, hyaw], "setup:back over the hover", cyc.safety.profile.approach))
            self.guard(lambda: cyc._move("L", [hx, hy, hz, rx, ry, hyaw], "setup:back down to the hover", cyc.safety.profile.approach))
            if got is None:
                raise Refused(f"no aruco tag seen over the stage ({last}): check the aruco plate is in the holder and the "
                              "microscope stands on the marked area (a microscope far from the taught place needs a new route)")
            z, res = got
            self.found["stage_view"] = tuple(res.pose)
            self.found["stage_estimate"] = {"view": [round(v, 3) for v in res.pose], "residual_px": round(res.residual_px, 2),
                                            "moves": res.moves, "seen_from_z": round(z)}
            return {"tag found, wrist over it at": [round(v, 2) for v in res.pose], "residual px": round(res.residual_px, 2), "moves": res.moves}

        def seat():
            tag = self.found.get("module_tag") or c.layout().get("setup", {}).get("module_tag")
            view = self.found.get("stage_view")
            if view is None or tag is None:
                raise Refused("needs the aruco tag taken on the module (swap section) and measured on the stage")
            old = c.layout()["fixtures"][self.scope]["pose"]
            x, y, yaw = servo.transfer_grip(tuple(tag["view"]), view, tuple(tag["grip"]))
            yaw = old[5] + geometry.wrap(yaw - old[5]) if abs(geometry.wrap(yaw - old[5])) <= 90 else old[5] + geometry.wrap(yaw - old[5] + 180.0)
            new = [round(x, 3), round(y, 3), old[2], old[3], old[4], round(yaw, 3)]
            moved = round(geometry.moved_mm(old, new), 2)
            reach = c.layout()["regions"]["microscope"].get("max_lateral_mm", 2.0)      # the taught route must still end at the hover
            if moved > reach:
                raise Refused(f"the seat found is {moved} mm from the taught one, beyond the taught route's reach "
                              f"({reach} mm); a route to a moved microscope is not planned yet. Nothing changed")
            keep = moved <= SEAT_MOVED_KEEP_MM

            def write(L):
                f = L["fixtures"][self.scope]
                if not keep:
                    if f.get("pose"):
                        f[f"previous_pose_{time.strftime('%Y_%m_%d')}"] = {"pose": f["pose"]}
                    f["pose"], f["taught_at"] = new, _stamp()
                self.setup_node(L).update(microscope={"seat_pose": f["pose"], "moved_mm": moved, "found_at": _stamp(),
                                                      "estimate": self.found.get("stage_estimate")})
            self.edit(write)
            return {"seat_pose": old if keep else new, "moved_mm": moved, "found_at": _stamp()}

        def route():
            cyc = self.cyc()
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            return {"route": c.layout()["fixtures"][self.scope].get("trajectory", "microscope_to_polar")}

        def drive():
            self.ready()
            cyc = self.cyc()
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            self.guard(lambda: cyc.move_to(self.scope))
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            return {"driven": "module -> microscope -> module, empty hand"}

        return Section("microscope.toml", [
            Step("computer", "read the stage travel limits from squid", limits),
            ])

    # ---- clear the stage
    def clear_stage(self) -> Section:
        c = self.cell

        def look():
            """After the user took the plate off: look down at the stage from the hover; the aruco must be gone."""
            self.ready()
            v = c.layout()["vision"]
            cyc = self.cyc()
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            self.guard(lambda: cyc.move_to(self.scope))
            try:
                seen = any(c.vision.aruco_pose(v["down_camera"], 1, None, v["led"]).pixel)
            except Exception:
                seen = False
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            if seen:
                raise Refused("the aruco plate is still on the microscope stage: take it off, then run again from \"clear the stage\"")
            self.found["stage_clear"] = True
            return {"stage": "clear"}

        return Section("clear the stage", [
            Step("user", "remove the aruco plate from the microscope stage", before=self.loosen),
            Step("computer", "note the stage as empty", lambda: self.found.update(stage_clear=True) or {"stage": "empty (the user took the plate off)"})])

    # ---- system_state.json
    def system_state(self) -> Section:
        c = self.cell
        seen: dict = {}

        def slots():
            self.ready()
            cyc = self.cyc()
            L = c.layout()
            names = [s for s in bench.slots(L) if s[0] in bench.state(L)["taught_columns"]]
            if L.get("setup", {}).get("hotel", {}).get("occupancy") == "by hand":      # the operator's list (layout fixtures.hotel.loaded) is the truth
                loaded = set(bench.loaded_slots(L))
                seen.update({s: "full" if s in loaded else "empty" for s in names})
                return {"slots": f"{len(loaded)} full, {len(names) - len(loaded)} empty (by hand, not looked at)"}
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            self.guard(cyc._to_hotel)
            for s in names:
                try:
                    v = exercise.probe_slots(cyc, [s], cyc.tele.log)[s]
                except SafetyViolation as e:                 # a slot the arm cannot reach is unknown, not a stop
                    cyc.tele.log("setup", f"probe:{s}", f"{s} is out of reach: {e}")
                    v = "unreachable"
                seen[s] = {"occupied": "full", "empty": "empty"}.get(v, "unknown")
            self.guard(lambda: exercise.along(cyc, "polar_to_hotel", reverse=True))
            return {"slots": ", ".join(f"{sum(v == k for v in seen.values())} {k}" for k in ("full", "empty", "unknown"))}

        def looks():
            cyc = self.cyc()
            spots = [("module", "polar@0", "polar.pose_at_0")]
            if self.found.get("stage_clear"):
                seen[self.scope] = "empty"                  # the stage was looked at a moment ago (clear the stage): no second trip over the microscope
            else:
                spots.append((self.scope, self.scope, self.scope))
            for loc, gate, pos in spots:
                self.guard(lambda: cyc.move_to(pos))
                img = cyc._frame(cyc.down_cam)
                full = cyc.lower.verdict(gate, "plate_present", img) if img else None
                empty = cyc.lower.verdict(gate, "empty", img) if img else None
                enough = full is not None and min(full.n_refs, empty.n_refs) >= cyc.lower.min_refs
                seen[loc] = ("unknown" if not enough else "full" if full.safe and not empty.safe
                             else "empty" if empty.safe and not full.safe else "unknown")
            jaw, g = c.robot.jaw(), c.layout()["gripper"]
            seen["gripper"] = "empty" if jaw > g["closed"]["short_edges"] + 2.0 or jaw < g["readings"]["empty_closed"] + 0.5 else "full"
            return {k: seen[k] for k in ("module", self.scope, "gripper")}

        def choose():
            cyc = self.cyc()
            cyc.store.s.contents = dict(seen)
            for s, v in seen.items():
                if s[:1] in "ABC" and s[1:].isdigit():
                    if v == "full":
                        cyc.store.s.hotel.setdefault(s, "")
                    elif v == "empty":
                        cyc.store.s.hotel.pop(s, None)
            cyc.store.save()
            if seen.get("gripper") == "full":
                task = "a plate is in the gripper: place it before anything else"
            elif seen.get("module") == "full":
                task = "plate on the module: run the test from there"
            else:
                task = "no plate on the module: put one there to run the test"
            places = {k: v for k, v in seen.items() if not (k[:1] in "ABC" and k[1:].isdigit())}
            return {"contents": places, "first task": task}

        return Section("system_state.json", [
            Step("polar", "look into every hotel slot", slots, moves=True),
            Step("polar", "look at microscope, module and gripper", looks, moves=True),
            Step("computer", "detect plate or empty at each, then choose the first task", choose)])

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
            self.ready()
            cyc = self.cyc()
            F = c.layout()["fixtures"]
            st = cyc.store.s
            taught = [x for x in F["hotel"].get("active_slots", []) if reachable(cyc, x)]
            holes = [x for x in taught if x not in set(bench.loaded_slots(c.layout())) and st.contents.get(x) != "full"]
            lo, hi = c.layout()["gripper"]["accept_band"]["long_edges"]
            moves = 0
            if lo <= c.robot.jaw() <= hi:                                                  # a plate in the hand at the long-edge width: the ledger can be stale
                st.plate_at, st.grip, st.step_done = "gripper", "long_edges", True
                cyc.store.save()
            if not holes and st.plate_at != "gripper":
                raise Refused("the test needs an empty taught slot for the module's plate, and another for the rotation to start: free two taught slots "
                              "(put their plates in untaught ones) and tell the occupancy")
            if st.plate_at == "gripper" and st.grip == "long_edges":
                slot = holes.pop(0)
                print(f"   a plate is in the hand by the long edges: scan, into {slot}")
                self.guard(lambda: cyc._run_steps(cyc._steps(slot)[0][4:]))
                self.edit(lambda L: bench.set_loaded(L, list(bench.loaded_slots(L)) + [slot]))
            elif st.contents.get("module") == "full" or st.plate_at in ("polar", self.scope):
                slot = holes.pop(0)
                out = cyc._steps(slot)[0]
                if st.plate_at == self.scope and st.step_done:
                    print(f"   the plate is already on the {self.scope} stage: carrying on from it")
                else:
                    P0, P90 = F["polar"]["pose_at_0"]["pose"], F["polar"]["pose_at_90"]["pose"]
                    self.guard(lambda: cyc.move_to("polar.pose_at_0"))
                    if cyc.polar.angle != 0:
                        self.guard(lambda: cyc._polar_rotate(0, P90))
                    st.plate_at, st.grip, st.step_done = "polar", "none", True
                    cyc.store.save()
                    route = F[self.scope].get("trajectory", "microscope_to_polar")
                    print(f"   the module's plate to the microscope, acquire, then into {slot}")
                    self.guard(lambda: cyc._pick("polar", P0, F["polar"], "short_edges", None, gate="polar@0"))
                    self.guard(lambda: cyc._place(self.scope, F[self.scope]["pose"], F[self.scope], self.scope, route, reverse=True))
                self._acquire()
                self.guard(lambda: cyc._run_steps(out))
                self.edit(lambda L: bench.set_loaded(L, list(bench.loaded_slots(L)) + [slot]))
                st.contents["module"] = "empty"
                cyc.store.save()
            passes = 0
            while not stopped():
                have = set(bench.loaded_slots(c.layout()))
                full = [x for x in taught if x in have]
                holes = [x for x in taught if x not in have and st.contents.get(x) != "full"]
                if not holes:
                    raise Refused("the rotation has no empty taught slot to start from: free one (put its plate in an untaught slot) and run: "
                                  "python cli.py setup --from \"test\"")
                passes += 1
                print(f"   rotation pass {passes} (stops when runs/STOP exists or with Ctrl+C)")
                for slot in full:
                    if stopped():
                        break
                    dest = holes.pop(0)
                    print(f"   rotate: {slot} to the microscope, then into {dest}")
                    self.edit(lambda L, slot=slot: bench.set_loaded(L, [x for x in bench.loaded_slots(L) if x != slot]))       # in transit: the slot is empty
                    self.guard(lambda: cyc._run_steps(cyc._steps(slot)[1]))
                    self._acquire()
                    self.guard(lambda: cyc._run_steps(cyc._steps(dest)[0]))
                    self.edit(lambda L, dest=dest: bench.set_loaded(L, list(bench.loaded_slots(L)) + [dest]))
                    holes.append(slot)
                    moves += 1
            return {"stopped": "by request, between plates", "passes": passes, "plates moved": moves}

        return Section("test", [Step("polar", "the rotation, for ever: the module's plate through the microscope into an empty slot, then every plate in a taught slot to the microscope and into the slot empty the longest", unattended(rotate), moves=True)])