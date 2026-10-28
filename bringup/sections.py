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

import exercise
from bringup import geometry
from bringup.runner import Refused, Section, Step
from bringup import bench, cameras, hand
from perception.hotel_size import HotelSize
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
VIEW_SEARCH_MM = ((0, 0), (25, 0), (-25, 0), (0, 25), (0, -25), (25, 25), (-25, -25), (25, -25), (-25, 25), (50, 0), (-50, 0))   # along, across the claw


def reachable(cyc: Cycle, slot: str) -> bool:
    """The slot's seat is inside the workspace box and above its region's floor."""
    try:
        cyc.safety.check_target(cyc.hotel_slot_pose(slot), f"slot {slot}", allow_floor_regions=("hotel",))
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

    def module_view_pose(self, along: float = 0.0, across: float = 0.0) -> List[float]:
        """The wrist pose that puts the down camera over the module's plate: the taught seat's wrist pose moved along
        the claw. The camera looks `camera_ahead_of_wrist_mm` ahead of the wrist, the plate centre is
        `module_wrist_to_plate_mm` ahead of it at the seat (both in hardware.json, from the CAD)."""
        claw = cad_poses.load_hardware()["claw"]
        taught = self.cyc()._position("polar.pose_at_0")
        yaw = math.radians(taught[5])
        d, n = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
        shift = claw["module_wrist_to_plate_mm"] - claw["camera_ahead_of_wrist_mm"] + along
        return [taught[0] + shift * d[0] + across * n[0], taught[1] + shift * d[1] + across * n[1], taught[2], 0.0, 0.0, taught[5]]

    def view_module(self) -> List[float]:
        """Take the camera over the module's plate and read the marker: the CAD pose first, then a small cross of
        offsets (the camera's sideways offset from the claw is not known). Remembers where it read."""
        c, v = self.cell, self.cell.layout()["vision"]
        cyc = self.cyc()
        tries = [self.found["module_view"]] if self.found.get("module_view") else []
        tries += [self.module_view_pose(a, b) for a, b in VIEW_SEARCH_MM]
        for pose in tries:
            self.guard(lambda pose=pose: cyc.go_to(pose, "module view"))
            try:
                if any(c.vision.aruco_pose(v["down_camera"], 1, None, v["led"]).pixel):
                    self.found["module_view"] = pose
                    return pose
            except Exception:
                pass
        raise Refused("the down camera cannot read the aruco plate over the module: is the practice plate on the module, "
                      "marker up, and the lens clean?")

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

        return Section("connect", [
            Step("computer", "search ports", search_ports),
            Step("squid", "home stage", home, moves=True),
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

        def to_hotel():
            self.ready()
            cyc = self.cyc()
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))
            self.guard(cyc._to_hotel)
            self.guard(lambda: cyc._hotel_go_to_approach("A1", "setup:hotel"))

        def image():
            cyc = self.cyc()
            img = cyc._frame(cyc.front_cam)
            if img is None:
                raise Refused("the front camera gave no picture")
            self.found["hotel_image"] = img
            cyc.tele.image("hotel_view", img)
            return {"front camera": "1 picture"}

        def detect():
            det = HotelSize(c.ref_dir)
            v = det.verdict(self.found["hotel_image"])
            method, size = "image", v.size
            if size is None:
                said = self.ask(f"      the camera cannot tell the hotel size ({v.reason}). 15 or 30 slots? ").strip()
                if said not in ("15", "30"):
                    raise Refused(f"hotel size {said!r} is neither 15 nor 30")
                size, method = int(said), "declared"
                det.learn(size, self.found["hotel_image"])        # the next run recognises this hotel
            at = _stamp()
            self.found["hotel_size"] = size
            self.edit(lambda L: (bench.set_hotel_size(L, size),
                                 self.setup_node(L).update(hotel={"size": size, "method": method, "detected_at": at})))
            return {"size": size, "method": method, "detected_at": at}

        def leave():
            cyc = self.cyc()
            self.guard(lambda: cyc._hotel_leave_approach("A1", "setup:hotel"))
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))

        return Section("hotel.toml", [
            Step("polar", "arm to the hotel's first slot, the front camera looking in", to_hotel, moves=True),
            Step("nova", "image the hotel with the front camera", image),
            Step("computer", "detect hotel size, 15 or 30", detect),
            Step("polar", "back to the rotation module", leave, moves=True)])

    # ---- swap the plates
    def swap(self) -> Section:
        c = self.cell

        def read_plate():
            self.ready()
            L = c.layout()
            v, F = L["vision"], L["fixtures"]["polar"]
            self.view_module()
            a = c.vision.aruco_pose(v["down_camera"], F.get("aruco_marker_id", 1), None, v["led"])
            target = [float(x) for x in a.target]
            ref = F.get("aruco_reference")
            if ref:
                off = c.vision.pose_offset(v["down_camera"], ref, v["workspace"], F.get("aruco_marker_id", 1), None, v["led"])
                dx, dy, dyaw = float(off.d[0]), float(off.d[1]), float(off.d[5])
            else:                                                  # no box reference yet: the first read is the nominal
                nominal = self.setup_node(L).get("module_marker")
                dx, dy, dyaw = ((target[0] - nominal[0], target[1] - nominal[1], target[5] - nominal[5]) if nominal else (0.0, 0.0, 0.0))
            rel = geometry.grip_in_marker_frame(target, F["pose_at_0"]["pose"])
            offset_ = [round(dx, 3), round(dy, 3), round(dyaw, 3)]
            self.found.update(module_marker=target, plate_offset=offset_, marker_to_grip=rel)

            def write(L):
                n = self.setup_node(L)
                n.update(plate_offset=offset_, marker_to_grip=rel, plate_offset_at=_stamp())
                n.setdefault("module_marker", target)
            self.edit(write)
            return {"plate_offset": offset_}

        return Section("swap", [
            Step("nova", "detect aruco on the plate in the rotation module with the down camera", read_plate, moves=True),
            Step("user", "move the aruco plate to the microscope stage", before=self.loosen),
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
            self.ready()
            L = c.layout()
            cyc = self.cyc()
            top = L["workspace_box"]["z"][1]
            v = L["vision"]
            if not self._parked():
                raise Refused("the stage is not parked: home Squid first")
            # the microscope zone is entered along the taught route, to the hover, and only vertically from there
            self.guard(lambda: cyc.move_to("polar.pose_at_0"))          # the taught route starts at the module hover
            self.guard(lambda: cyc.move_to(self.scope))
            hx, hy, hz, *rest = cyc._position(self.scope)
            heights = [top - (top - hz) * i / max(SEARCH_HEIGHTS - 1, 1) for i in range(SEARCH_HEIGHTS)]
            for z in heights:
                self.guard(lambda: cyc._move("L", [hx, hy, z] + rest, f"setup:search z{z:.0f}", cyc.safety.profile.approach))
                try:
                    a = c.vision.aruco_pose(v["down_camera"], 1, None, v["led"])
                except Exception:
                    continue
                if any(a.pixel):
                    self.found["stage_marker"] = [float(t) for t in a.target]
                    self.guard(lambda: cyc._move("L", [hx, hy, hz] + rest, "setup:back to the hover", cyc.safety.profile.approach))
                    return {"marker at": [round(t, 1) for t in a.target[:3]], "seen from z": round(z)}
            self.guard(lambda: cyc._move("L", [hx, hy, hz] + rest, "setup:back to the hover", cyc.safety.profile.approach))
            raise Refused("no aruco seen below the microscope hover from any height: check the aruco plate is in the holder and the "
                          "microscope stands on the marked area (a microscope far from the taught place needs a new route)")

        def seat():
            m = self.found.get("stage_marker")
            rel = self.found.get("marker_to_grip") or c.layout().get("setup", {}).get("marker_to_grip")
            if m is None or rel is None:
                raise Refused("needs the aruco read on the module (swap section) and on the stage")
            old = c.layout()["fixtures"][self.scope]["pose"]
            new = geometry.seat_from_marker(m, rel, old[5] if old else 0.0, old or [0, 0, 0, 0, 0, 0])
            moved = round(geometry.moved_mm(old, new), 2) if old else None
            reach = c.layout()["regions"]["microscope"].get("max_lateral_mm", 2.0)      # the taught route must still end at the hover
            if moved is not None and moved > reach:
                raise Refused(f"the seat found is {moved} mm from the taught one, beyond the taught route's reach "
                              f"({reach} mm); a route to a moved microscope is not planned yet. Nothing changed")
            keep = moved is not None and moved <= SEAT_MOVED_KEEP_MM

            def write(L):
                f = L["fixtures"][self.scope]
                if not keep:
                    if f.get("pose"):
                        f[f"previous_pose_{time.strftime('%Y_%m_%d')}"] = {"pose": f["pose"]}
                    f["pose"], f["taught_at"] = new, _stamp()
                self.setup_node(L).update(microscope={"seat_pose": f["pose"], "moved_mm": moved, "found_at": _stamp()})
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
            Step("polar", "descend from the highest allowed height over the scope and look for the aruco", find_marker, moves=True),
            Step("computer", "seat_pose from the aruco on the stage and the module's marker-to-grip", seat),
            Step("polar", "leave the scope zone for the module", route, moves=True),
            Step("polar", "drive the route with the empty hand", drive, moves=True)])

    # ---- clear the stage
    def clear_stage(self) -> Section:
        return Section("clear the stage", [Step("user", "remove the aruco plate from the microscope stage", before=self.loosen)])

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
