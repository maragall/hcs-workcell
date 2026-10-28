"""
The setup, section by section, in the order the user meets them. Each line is one Step; a machine line calls the
function cli.py and the loop already use (bringup.cameras, Cycle, exercise, the vision box, Squid) and returns
the values its comment names. Nothing here talks to a device directly except through those.

Written through `positions`: layout.json -> `setup` (what this run found), the fixtures it re-seats, the gripper
widths. Written to the ledger: `contents` (what is where) in state_microscope.json.

Not run on the real arm yet: the sections are exercised on the simulator (tests/test_bringup.py).
"""
from __future__ import annotations

import math
import os
import random
import time
from typing import Callable, List, Optional

import numpy as np

import exercise
from bringup import geometry
from bringup.runner import Refused, Section, Step
from bringup import bench, cameras, hand, servo
from perception.hotel_rows import HotelRows
from procedures import cad_poses
from procedures.corrector import StopSafe
from procedures.cycle import Cycle
from procedures.motion import offset
from procedures.safety import SafetyViolation
from procedures.state import StateStore
from procedures.telemetry import Telemetry

SEAT_MOVED_KEEP_MM = 1.0          # closer than this the taught seat stays: the read has that much noise
SEARCH_HEIGHTS = 4                # heights tried below the microscope hover, highest allowed first
BAND_BELOW, BAND_ABOVE = 0.1, {"short_edges": 0.5, "long_edges": 1.05}     # as gripper.accept_band was set 2026-10-22
RESEATS = 3                       # carry, seat and re-grip cycles in the grips section
ACQUISITION_TIMEOUT_S = 600
TAG_VIEW_MARGIN_MM = 5.0         # headroom under the workspace top for the stage view
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
    def __init__(self, cell, squid, ask: Callable[[str], str], scope: str = "microscope"):
        self.cell, self.squid, self.ask, self.scope = cell, squid, ask, scope
        self._cyc: Optional[Cycle] = None
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
        return self._cyc

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

    def tag_height(self) -> float:
        """Where the module's tag reference is taken from: straight above the module's grip pose, as high as the same view over
        the stage still fits under the workspace top (the stage seat is higher than the module's by SEAT_RISE). Both pictures
        are then taken from straight above a grip pose at the same height over the plate, which the microscope zone allows
        (only vertical moves there), and the tag, 71 mm ahead of the camera's axis at the seat, is in the picture."""
        L = self.cell.layout()
        rise = L["fixtures"][self.scope]["pose"][2] - L["fixtures"]["polar"]["pose_at_0"]["pose"][2]
        return round(L["workspace_box"]["z"][1] - rise - TAG_VIEW_MARGIN_MM, 1)

    def _servo(self) -> "servo.Servo":
        """The aruco servo (bringup/servo.py): the down camera's picture of the plate tag and small checked moves of the wrist. It
        needs no camera calibration: the tag's place is the arm's pose when the tag looks as it did at the reference."""
        c, v = self.cell, self.cell.layout()["vision"]
        cyc = self.cyc()
        read = servo.corner_reader(lambda: c.vision.frame_full(v["down_camera"], v["led"]))

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

    def view_module(self) -> List[float]:
        """Take the camera over the module's plate and read the marker: the CAD pose first, then a small cross of
        offsets (the camera's sideways offset from the claw is not known). Remembers where it read."""
        c, v = self.cell, self.cell.layout()["vision"]
        cyc = self.cyc()
        tries = [self.found["module_view"]] if self.found.get("module_view") else []
        tries += [self.module_view_pose(a, b, z) for z in VIEW_LIFTS_MM for a, b in VIEW_SEARCH_MM]
        last = "no pose tried"
        for pose in tries:
            try:
                self.guard(lambda pose=pose: cyc.go_to(pose, "module view"))
            except Refused as e:                  # a candidate the safety check refuses is skipped, not fatal
                last = str(e)
                continue
            try:
                if any(c.vision.aruco_pose(v["down_camera"], 1, None, v["led"]).pixel):
                    self.found["module_view"] = pose
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
            got = self.guard(lambda: cameras.pictures(c, False, "down"))
            return {"pictures": sum(got.values()), "captured_at": _stamp()}

        def solve():
            res = cameras.solve(c, "down")
            r = next(iter(res.values()))
            self.found["camera"] = r
            return {"error_px": r["px"], "error_mm": r["mm"]}

        def limits():
            r = self.found.get("camera")
            if not r or not r["ok"]:
                raise Refused(f"calibration error {r and r['px']} px / {r and r['mm']} mm is above "
                              f"{cameras.LIMIT_PX} px / {cameras.LIMIT_MM} mm: nothing uploaded, run again")

        def upload():
            cameras.upload(c, "down")
            at = node().get("uploaded_at")
            if not at:
                raise Refused("the box did not take the files")
            return {"uploaded_at": at}

        def validate():
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
            Step("computer", "refuse if px > 0.5 or mm > 1.0", limits),
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
                self.guard(lambda: cyc.go_to([P0[0], P0[1], self.tag_height(), P0[3], P0[4], P0[5]], "tag view"))
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
            prev = self.setup_node(L).get("module_tag")
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
                n.setdefault("module_tag", dict(tag, taken_at=_stamp()))
            self.edit(write)
            return {"plate_offset": offset_, "tag": "reference taken" if not prev else "found again"}

        def to_stage():
            """The arm carries the aruco plate from the module to the stage, the way the cycle carries any plate, so the plate keeps
            the orientation it had in the claw: the tag looks the same from the stage as it did from the module."""
            self.ready()
            cyc = self.cyc()
            F = c.layout()["fixtures"]
            if not self._parked():
                raise Refused("the stage is not parked: home Squid first")
            route = F[self.scope].get("trajectory", "microscope_to_polar")
            self.guard(lambda: cyc._pick("polar", F["polar"]["pose_at_0"]["pose"], F["polar"], "short_edges", None, gate="polar@0"))
            self.guard(lambda: cyc._place(self.scope, F[self.scope]["pose"], F[self.scope], self.scope, route, reverse=True))
            cyc.store.s.contents = dict(cyc.store.s.contents, **{"module": "empty", self.scope: "full"})
            cyc.store.save()
            if cyc.polar.angle != 0:                                   # the half turn is undone with the module empty
                self.guard(lambda: cyc.move_to("polar.pose_at_0"))
                self.guard(lambda: cyc._polar_rotate(0, F["polar"]["pose_at_0"]["pose"]))
            return {"aruco plate": f"module -> {self.scope} stage"}

        return Section("swap", [
            Step("nova", "detect aruco on the plate in the rotation module with the down camera (the module turns the plate half a turn if the tag is at the far end)", read_plate, moves=True),
            Step("polar", "carry the aruco plate from the module to the microscope stage", to_stage, moves=True),
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
            s = self._servo()
            ref = np.array(tag["corners"], float)
            got, last = None, "the tag is not in the picture"
            for z in heights:
                self.guard(lambda z=z: cyc._move("L", [hx, hy, z, rx, ry, hyaw], f"setup:look z{z:.0f}", cyc.safety.profile.approach))
                try:
                    est = s.estimate(ref)
                except servo.ServoError as e:
                    last = str(e)
                    continue
                got = (z, est)
                break
            self.guard(lambda: cyc._move("L", [hx, hy, hz, rx, ry, hyaw], "setup:back to the hover", cyc.safety.profile.approach))
            if got is None:
                raise Refused(f"no aruco tag seen over the stage ({last}): check the aruco plate is in the holder and the "
                              "microscope stands on the marked area (a microscope far from the taught place needs a new route)")
            z, est = got
            self.found["stage_view"] = (hx + est.offset[0], hy + est.offset[1], hyaw + est.offset[2])
            self.found["stage_estimate"] = {"offset": [round(v, 3) for v in est.offset], "residual_px": round(est.residual_px, 2),
                                            "warning": est.warning, "seen_from_z": round(z), "same_height_z": round(z_same)}
            if est.warning:
                self.cell.note(f"the stage tag estimate: {est.warning}")
            return {"tag offset": [round(v, 2) for v in est.offset], "residual px": round(est.residual_px, 2), "seen from z": round(z)}

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
            Step("polar", "over the scope, straight down to the height the module view had over the plate, and measure the aruco tag", find_marker, moves=True),
            Step("computer", "seat_pose from the tag on the stage, carried from the module's tag and grip pose", seat),
            Step("polar", "leave the scope zone for the module", route, moves=True),
            Step("polar", "drive the route with the empty hand", drive, moves=True)])

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
            return {"stage": "clear"}

        return Section("clear the stage", [
            Step("user", "remove the aruco plate from the microscope stage", before=self.loosen),
            Step("polar", "look down at the stage: the aruco must be gone", look, moves=True)])

    # ---- system_state.json
    def system_state(self) -> Section:
        c = self.cell
        seen: dict = {}

        def slots():
            self.ready()
            cyc = self.cyc()
            L = c.layout()
            names = [s for s in bench.slots(L) if s[0] in bench.state(L)["taught_columns"]]
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
            for loc, gate, pos in (("module", "polar@0", "polar.pose_at_0"), (self.scope, self.scope, self.scope)):
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
    def test(self) -> Section:
        c = self.cell

        def run():
            self.ready()
            cyc = self.cyc()
            F = c.layout()["fixtures"]
            free = [s for s in F["hotel"].get("active_slots", []) if cyc.store.s.contents.get(s) != "full" and reachable(cyc, s)]
            if not free:
                raise Refused("no free hotel slot the arm can reach to put the plate in")
            slot = random.choice(free)
            out, _ = cyc._steps(slot)
            P0, P90 = F["polar"]["pose_at_0"]["pose"], F["polar"]["pose_at_90"]["pose"]
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            if cyc.polar.angle != 0:
                self.guard(lambda: cyc._polar_rotate(0, P90))
            cyc.store.s.plate_at, cyc.store.s.grip, cyc.store.s.step_done = "polar", "none", True
            cyc.store.save()
            route = F[self.scope].get("trajectory", "microscope_to_polar")
            self.guard(lambda: cyc._pick("polar", P0, F["polar"], "short_edges", None, gate="polar@0"))
            self.guard(lambda: cyc._place(self.scope, F[self.scope]["pose"], F[self.scope], self.scope, route, reverse=True))
            self.squid.acquire()
            deadline = time.time() + ACQUISITION_TIMEOUT_S
            while not self.squid.done():
                if c.stop_event.is_set() or time.time() > deadline:
                    raise Refused("the acquisition did not finish")
                time.sleep(0.2)
            self.squid.home()
            self.guard(lambda: cyc._run_steps(out))
            return {"plate": f"module -> {self.scope} -> module -> hotel {slot}", "acquisitions": getattr(self.squid, "acquisitions", "n/a")}

        return Section("test", [Step("polar", "module to microscope stage, acquire, back to the module, grip the long edges, to a free hotel slot", run, moves=True)])
