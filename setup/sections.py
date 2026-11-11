"""
The setup, section by section, in the order the user meets them. Each line is one Step; a machine line calls the
function cli.py and the loop already use (setup.cameras, Cycle, exercise, the vision box, Squid) and returns
the values its comment names. Nothing here talks to a device directly except through those.

Written through `positions`: layout.json -> `setup` (what this run found), the fixtures it re-seats, the gripper
widths. Written to the ledger: `contents` (what is where) in state_microscope.json.

Not run on the real arm yet: the sections are exercised on the simulator (utils/tests/test_bringup.py).
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Callable, List, Optional


from run import positions
from run import lua
from setup.runner import Refused, Section, Step
from setup import cameras, hand
from vision import colour
from run import cad_poses
from vision.corrector import StopSafe
from run.cycle import Cycle
from run.planner.safety import SafetyViolation
from run.state import StateStore
from run.telemetry import Telemetry

CARRY_ON = ("safe to lower", "clear to lower", "clear to rotate", "fine to continue", "seated correctly", "held correctly", "Continue anyway", "Proceed with the taught pose")      # the camera gates' questions: the setup answers yes and logs the mismatch
VIEW_DISTANCE_MM = 140.0         # camera height above the tag for the capture grid (vision/calibration/plan.py tag_in_view)
VIEW_SEARCH_MM = ((0, 0), (25, 0), (-25, 0), (0, 25), (0, -25), (25, 25), (-25, -25), (25, -25), (-25, 25), (50, 0), (-50, 0))   # along, across the claw
VIEW_LIFTS_MM = (0, 60, 120)      # heights above the seat hover the view is tried at (the camera has to focus on the plate)


def _stamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


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
                from vision.calibration import marker
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
        # the setup is the arm only (docs/workflow.md): what the arm needs to know, uploaded; then the cameras. Instruments are taught after.
        return [self.connect(), self.preset(), self.cameras()]

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

        self._module_to_0, self._hover = module_to_0, hover
        return Section("connect", [Step("computer", "search ports", search_ports)])

    # ---- the program on the arm
    def preset(self) -> Section:
        """What the arm needs to know, written as its program and uploaded: the hotel slots, the module seats, the two grips with the
        force close, the stored routes, the slot entry and exit (run/lua.py). Nothing moves."""
        c = self.cell
        folder = os.path.join(c.runs_dir, "lua_project")

        def write():
            routes = json.load(open(os.path.join(positions.ROOT, "run", "preset", "routes.json"), encoding="utf-8"))
            got = lua.build(c.layout(), routes, folder)
            return {"places": got["places"], "with routes": len(got["with_routes"]), "without routes": ", ".join(got["without_routes"]) or "none"}

        def parse():
            if c.sim and not os.path.exists(lua.LUA_DLL):
                return {"files": "not parsed, no BioyondWorks on this PC"}
            return {"files": lua.check(folder)}

        def send():
            got = lua.upload(c.robot, folder, c.runs_dir, None if c.sim else c.layout())
            return {"files": got["files"], "backup": got["backup"]}

        def alive():
            if c.sim:
                return {"program": "not started on the simulator"}
            from utils.devices import clamp
            r = clamp.read(c.layout()["robot"]["ip"])           # starts the program, waits for its heartbeat, reads the jaw, stops it
            return {"jaw": r["width"], "torque": r["torque"]}

        return Section("preset", [
            Step("computer", "write the program for the arm from layout.json and the stored routes", write),
            Step("computer", "parse it with the vendor's Lua", parse),
            Step("polar", "back up the program on the arm, upload the new one, read it back and compare", send),
            Step("polar", "start it, wait for its heartbeat, stop it (nothing moves)", alive)])

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

        self.connect()                                        # for module_to_0 and hover, which it defines
        return Section("cameras", [
            Step("user", "put the aruco plate on the rotation module, tag up, and press Enter"),
            Step("polar", "rotation module to 0 deg", self._module_to_0, moves=True),
            Step("polar", "hover hand onto rotation module centroid", self._hover, moves=True),
            Step("nova", "detect aruco with the down camera", detect, moves=True),
            Step("computer", "solve intrinsic (lens) and extrinsic (camera-to-gripper) calibration together", solve),
            Step("computer", "refuse if px > 5.0 or mm > 2.0", limits),
            Step("computer", "re-upload to nova, which restarts", upload)])



