"""
The setup, section by section, in the order the user meets them. Each line is one Step; a machine line calls the
function cli.py and the loop already use (Cycle, the arm's program writer, Squid's control server) and returns
the values its comment names. Nothing here talks to a device directly except through those.

Written through `positions`: layout.json -> `setup` (what this run found), the fixtures it re-seats, the gripper
widths. Written to the ledger: `contents` (what is where) in state_microscope.json.

Not run on the real arm yet: the sections are exercised on the simulator (utils/tests/test_bringup.py).
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import time
from typing import Callable, List, Optional


from run import positions
from run import lua
from setup.runner import Refused, Section, Step
from setup import hand, places
from utils import colour
from run.corrector import StopSafe
from run.cycle import Cycle
from run.planner.safety import SafetyViolation
from run.state import StateStore
from run.telemetry import Telemetry

ASK_AGAIN = 3                    # times a device that does not answer is asked for again in place, before the section stops


class Bringup:
    def __init__(self, cell, squid, ask: Callable[[str], str], scope: str = "microscope", speed: str = "stress", check_squid: bool = True):
        self.cell, self.squid, self.ask, self.scope = cell, squid, ask, scope
        self.check_squid = check_squid         # False (--no-squid): the connect section does not ask Squid whether it answers
        self.speed = speed                     # the speed profile (layout speeds) the test runs at; the setup steps keep the supervised one
        self._cyc: Optional[Cycle] = None
        self.unattended = False               # set while the rotation runs for ever: a question that would wait for a person is answered no
        self.found: dict = {}                 # what this run learned

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
            self._cyc = Cycle(c.layout(), c.robot, Telemetry(run_dir), store, "stress", run_dir,
                              scope=self.scope, layout_path=c.layout_path, stop_event=c.stop_event, parked=self._parked)
            self._answer_for(self._cyc)
        return self._cyc

    def _answer_for(self, cyc: Cycle) -> None:
        """A question the motion code has for the operator: asked in the terminal, or answered no when nobody is there."""
        def ask(question, timeout=None):
            if self.unattended:
                cyc.tele.log("setup", "question", f"not asked (unattended run): {question.strip()} -> n")
                print(f"      (unattended: no one is waited for; '{question.strip()}' -> n, so the run stops here with the reason above)")
                return "n"
            return self._ask_user(question)
        cyc.corr.ask = ask

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
        # the setup is the arm: found and enabled (and Squid answering), the places shown by hand, what the arm needs to know uploaded.
        # Instruments are taught after (python cli.py teach).
        return [self.connect(), self.places(), self.preset()]

    # ---- connect: the arm and Squid
    def _again(self, what: str, why: str, fix: str, tries_left: int) -> None:
        """A device does not answer: say why and what to do, and wait for Enter to ask it again, here. With no tries left, or with
        nobody at the keyboard (an empty answer comes back at once, --yes), the section stops with the same words."""
        if tries_left <= 0:
            raise Refused(f"{what}: {why}. {fix}")
        print(colour.paint(f"      {what}: {why}", colour.BAD))
        t0 = time.time()
        answer = self.ask(colour.paint(f"      {fix}.\n      Then press Enter to try again ({tries_left} more), or type q to stop: ")).strip().lower()
        if answer in ("q", "quit") or time.time() - t0 < 0.05:
            raise Refused(f"{what}: {why}. {fix}")

    def connect(self) -> Section:
        """Nothing else can start before this holds: the arm found on the network, this program let in, the arm enabled with live
        readouts, the controller's own port answering (the upload and the force close use it), and every Squid of the positions
        file answering on its control server. A device that does not answer is asked again in place after the user fixed it."""
        c = self.cell

        def row(name: str) -> dict:
            return next((d for d in c.devices if d["name"] == name), {})

        def find_arm():
            for left in range(ASK_AGAIN, -1, -1):
                c.discover()
                arm = row("Arm")
                if arm.get("ok"):
                    return {"arm": arm["found"], "state": arm["state"]}
                self._again("the arm is not connected", arm.get("state", "not found"), arm.get("fix") or "check power and cables", left)

        def enable_arm():
            if c.robot.state().drag:
                hand.drag(c, False)
            if not c.robot.state().enabled and not c.robot.enable():
                raise Refused("the arm could not be enabled. Release the emergency stop, clear the error on the controller "
                              "(python cli.py recover), then: python cli.py setup")
            pose, joints, jaw = c.robot.pose(), c.robot.joints(), c.robot.jaw()      # stale until the first enable: read after it
            if not any(abs(float(v)) > 1e-6 for v in list(pose[:3]) + list(joints[:4])):
                raise Refused("the arm is enabled but reads zero everywhere: its readouts are not live yet. Wait ten seconds and run again")
            c.arm = dict(c.arm, ok=True, enabled=True, drag=False)
            errs = [e for e in (c.robot.errors() or []) if e.get("errCode")]
            if errs:
                raise Refused("the controller reports " + "; ".join(f"{e.get('errCode')} {e.get('errMsg', '')}" for e in errs)
                              + ". Clear it: python cli.py recover")
            return {"pose": [round(float(v), 1) for v in pose[:3]], "jaw": round(float(jaw), 2), "module": c._stage_words(c.layout())}

        def squid_answers():
            if c.sim:
                return {"squid": "simulator"}
            if not self.check_squid:
                return {"squid": "not asked (--no-squid)"}
            for left in range(ASK_AGAIN, -1, -1):
                c.discover()
                rows = [d for d in c.devices if d.get("squid")]
                if not rows:
                    return {"squid": "none in layout.json (squid): nothing to ask"}
                bad = [d for d in rows if not d["ok"]]
                if not bad:
                    return {d["name"]: f"{d['found']} {d['state']}" for d in rows}
                self._again(bad[0]["name"], bad[0]["state"], bad[0]["fix"] + " (or run the setup with --no-squid: the arm's setup does not need the microscope, teach does)", left)

        return Section("connect", [
            Step("computer", "find the arm on the network and connect to it", find_arm),
            Step("polar", "lock and enable the arm, read where it stands (nothing moves)", enable_arm),
            Step("squid", "Squid answers on its control server (nothing moves)", squid_answers)])

    # ---- the places shown by hand
    def places(self) -> Section:
        """What differs from one workcell to the next, shown by hand with a plate: the module's two seats (their angle then found
        by force) and six hotel slots (the hotel fitted through them as a rigid box). Then every route is planned again (setup/places.py)."""
        c = self.cell
        index = lambda: int(c.layout()["polar"]["ext_axis"]["index"])          # noqa: E731

        def keep():
            path = os.path.join(c.runs_dir, time.strftime("layout_before_places_%Y%m%d_%H%M%S.json"))
            shutil.copyfile(c.layout_path, path)
            return {"copy": path}

        def clear_height() -> float:
            F = c.layout()["fixtures"]["polar"]
            return max(F["pose_at_0"]["pose"][2], F["pose_at_90"]["pose"][2]) + float(F["hover_mm"]) + float(F["clear_mm"])

        def module_to(deg: int, grip: str):
            def go():
                self.ready()
                L = c.layout()
                self.guard(lambda: c.robot.ext_axis_to(index(), places.module_reading(L, deg)))
                self.guard(lambda: c.robot.jaw_to(places.jaw_for(L, grip)))
                return {"module": f"{deg} deg", "jaw": places.jaw_for(L, grip)}
            return go

        def seat(deg: int, grip: str):
            def go():
                key = "pose_at_0" if deg == 0 else "pose_at_90"
                while True:
                    got = places.record(c.robot, c.layout())
                    self.ready()                                   # locks the arm where the hand left it
                    c.edit(lambda L: L["fixtures"]["polar"][key].update(got, taught_how="by hand, the pads in the plate's grooves"))
                    if c.sim:
                        return {"seat": got["pose"][:3], "yaw": "not refined on the simulator"}
                    from utils.seat_torques import read
                    try:
                        r = self.guard(lambda: places.refine_yaw(c.robot, c.layout(), grip, read, c.note))
                        break
                    except places.PlacesError as e:                # this seat again, here: what was taught before it stays
                        c.note(f"NOT TAKEN: {e}")
                        self.loosen()
                        self.ask(f"      fit the gripper onto the plate's {grip.split('_')[0]} edges again, then press Enter: ")
                c.edit(lambda L: L["fixtures"]["polar"][key].update(
                    pose=r["pose"], taught_how=f"by hand, then the yaw by force: the module was left alone from {r['free_from']} to {r['free_to']} deg, "
                                                f"found from how far each close pushed the module (the hand had {r['taught_yaw']:.2f})"))
                here = c.robot.pose()
                self.guard(lambda: c.robot.move_linear([here[0], here[1], clear_height()] + list(here[3:])))
                return {"seat": [round(v, 2) for v in r["pose"][:3]], "yaw": r["pose"][5], "by hand": round(r["taught_yaw"], 2)}
            return go

        def slot(name: str):
            def go():
                got = places.record(c.robot, c.layout())
                c.edit(lambda L: L["fixtures"]["hotel"].setdefault("taught_detail", {}).__setitem__(name, got))
                return {name: [round(v, 1) for v in got["pose"][:3]], "jaw": got["jaw_at_teach"]}
            return go

        def fit():
            L = c.layout()
            try:
                got = places.fit_hotel(L)
            except places.PlacesError as e:
                raise Refused(str(e))
            c.save(L)
            return got

        def plan():
            if c.sim:
                return {"routes": "not planned on the simulator"}
            r = subprocess.run([sys.executable, "-X", "utf8", os.path.join(positions.ROOT, "utils", "plan_preset_routes.py")],
                               capture_output=True, text=True, errors="replace", cwd=positions.ROOT)
            last = (r.stdout.strip().splitlines() or [""])[-1]
            if r.returncode != 0 or "planned" not in last:
                raise Refused(f"the routes were not planned: {(r.stderr or r.stdout)[-300:]}")
            return {"routes": last.split(" ->")[0], "instruments": "teach them again after the setup (python cli.py teach)"}

        def long_jaw():
            self.ready()
            self.guard(lambda: c.robot.jaw_to(places.jaw_for(c.layout(), "long_edges")))
            return {"jaw": places.jaw_for(c.layout(), "long_edges")}

        # The hotel first, the module last: the arm ends above the module's seat, a known place, with the plate on the module.
        # (Operator, 2026-11-10: after the last slot the hand left the arm anywhere.)
        on = "fit the gripper onto the plate's {} edges until the pads sit in the grooves, then press Enter"
        steps = [
            Step("computer", "keep a copy of the positions file", keep),
            Step("user", "the hotel must be screwed onto the platform, empty. Take the gripper off any plate: the jaw is about to move. Press Enter", before=self.loosen),
            Step("polar", "jaw just wider than the plate's long edges", long_jaw, moves=True)]
        for name in places.SLOTS:
            steps += [Step("user", f"put the plate in slot {name}, pushed fully in. " + on.format("long"), before=self.loosen),
                      Step("computer", f"take that pose as slot {name}", slot(name))]
        steps += [
            Step("user", "take the gripper off the plate and out of the hotel. Put the plate on the rotation module. Keep the gripper off the rotation module: it is about to turn. Press Enter", before=self.loosen),
            Step("polar", "rotation module to 0 deg, jaw just wider than the plate's short edges", module_to(0, "short_edges"), moves=True),
            Step("user", on.format("short"), before=self.loosen),
            Step("polar", "take that pose as the module's short-edge seat, then find its angle by force (the gripper closes on the plate several times)", seat(0, "short_edges"), moves=True),
            Step("polar", "rotation module to 90 deg, jaw just wider than the plate's long edges", module_to(90, "long_edges"), moves=True),
            Step("user", on.format("long"), before=self.loosen),
            Step("polar", "take that pose as the module's long-edge seat, then find its angle by force. The arm ends above the module", seat(90, "long_edges"), moves=True),
            Step("computer", "fit the hotel through the six slots", fit),
            Step("computer", "plan every route slot <-> rotation module again (about 4 minutes)", plan)]
        return Section("places", steps)

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
            if c.sim and not lua.find_dll(c.layout()):
                return {"files": "not parsed, no BioyondWorks on this PC"}
            return {"files": lua.check(folder, layout=c.layout())}

        def send():
            got = lua.upload(c.robot, folder, c.runs_dir, None if c.sim else c.layout())
            return {"files": got["files"], "backup": got["backup"]}

        def alive():
            if c.sim:
                return {"program": "not started on the simulator"}
            from utils.devices import clamp
            self.ready()
            if not c.robot.state().enabled and not c.robot.enable():      # the upload leaves the arm disabled, and the controller starts no program then
                raise Refused("the arm could not be enabled, so its program cannot be started")
            r = clamp.read(c.layout()["robot"]["ip"])           # starts the program, waits for its heartbeat, reads the jaw, stops it
            return {"jaw": r["width"], "torque": r["torque"]}

        return Section("preset", [
            Step("computer", "write the program for the arm from layout.json and the stored routes", write),
            Step("computer", "parse it with the vendor's Lua", parse),
            Step("polar", "back up the program on the arm, upload the new one, read it back and compare", send),
            Step("polar", "start it, wait for its heartbeat, stop it (nothing moves)", alive)])
