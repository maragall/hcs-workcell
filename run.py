"""
hcs-workcell main loop, asyncio: one task per microscope, one arm shared, a list of where every plate is.

    python run.py --sim                                              fake arm, one fake microscope
    python run.py --sim --scopes microscope,microscope_2             fake arm, two fake microscopes
    python run.py --scopes microscope --slots A1 --mode supervised   the one-microscope cell, real arm
    python run.py --scopes microscope,microscope_2 --fake-scopes microscope_2 --slots A1,A3
                                                                     the test rig: two nests, one imaging
    python run.py ... --resume                                       continue after a stop

Each microscope task repeats: change the plate (the arm moves run under the one arm lock), start the
acquisition, wait for it while the other tasks use the arm. The arm moves are the blocking procedures
in procedures/cycle.py; each runs in a thread. The first fault stops every other task at its next step.
"""
from __future__ import annotations

import argparse
import asyncio
import copy
import os
import sys
import threading
import time
from typing import Dict, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from devices import nova  # noqa: E402
from devices.fake_squid import FakeSquid  # noqa: E402
from devices.squid import Squid  # noqa: E402
from procedures import positions  # noqa: E402
from procedures.corrector import StopSafe  # noqa: E402
from procedures.recovery import is_motion_fault, mark, recover_to_nest  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.hotel import plate_order  # noqa: E402
from procedures.preflight import preflight  # noqa: E402
from procedures.safety import SafetyViolation  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402


class Workcell:
    """One run. `scopes` maps a microscope name (its fixture in the positions file) to its device.
    layout, robot and vision are shared by every task and touched only while holding arm_lock."""

    def __init__(self, layout: dict, robot, vision, scopes: Dict[str, object], tele: Telemetry, mode: str,
                 run_dir: str, ref_dir: str, state_dir: str, layout_path: Optional[str] = None, poll_s: float = 1.0,
                 clock=time, frame_source=None):
        missing = [s for s in scopes if s not in layout["fixtures"]]
        if missing:
            raise KeyError(f"no fixture entry in the positions file for {missing}")
        self.scopes, self.tele, self.poll_s = scopes, tele, poll_s
        self.park_timeout_s = layout["limits"]["stage_park_timeout_s"]
        self.arm_lock = asyncio.Lock()                   # the one shared physical resource
        self.stop = threading.Event()                    # set by the first task that faults
        self.fault: Optional[BaseException] = None       # that first fault, the one reported
        self.cycles: Dict[str, Cycle] = {}
        for name in scopes:
            store = StateStore(os.path.join(state_dir, f"state_{name}.json"))
            if store.s.updated == 0.0:                   # never saved: a new ledger, the holder is empty
                store.save()
                tele.log("run", "ledger", f"{name}: new ledger; the holder is assumed empty, plates in the hotel")
            scope_dir = os.path.join(run_dir, name)
            self.cycles[name] = Cycle(layout, robot, vision, Telemetry(scope_dir), store, mode, scope_dir, ref_dir,
                                      scope=name, layout_path=layout_path, stop_event=self.stop, clock=clock,
                                      parked=scopes[name].parked, frame_source=frame_source)

    def plates(self) -> Dict[str, str]:
        """Where every plate is, by microscope: the microscope name, 'hotel:<slot>' or 'gripper'."""
        return {name: cyc.store.s.plate_at for name, cyc in self.cycles.items()}

    def close(self) -> None:
        for cyc in self.cycles.values():
            cyc.tele.close()

    def park(self, name: str) -> None:
        """Home the microscope's stage and wait until it reports parked; the arm does not move before that."""
        scope = self.scopes[name]
        scope.home()
        deadline = time.monotonic() + self.park_timeout_s
        while not scope.parked():
            if self.stop.is_set():
                return
            if time.monotonic() > deadline:
                raise StopSafe(f"{name}: stage not parked {self.park_timeout_s} s after home(); the arm stays out")
            time.sleep(self.poll_s)
        self.tele.log("run", "scope", f"{name}: stage parked")

    async def run_scope(self, name: str, slot: str, cycles: int, resume: bool = False,
                        stop_file: Optional[str] = None, order: Optional[list] = None) -> None:
        """`order`: a shared list of slots; each change takes the next one, so the microscopes walk the hotel
        together. Without it every change uses `slot`."""
        cyc, scope = self.cycles[name], self.scopes[name]
        try:
            for _ in range(cycles):
                if order is not None:
                    if not order:
                        return                       # no plates left
                    slot = order.pop(0)
                if stop_file and os.path.exists(stop_file):
                    return
                async with self.arm_lock:                # queued: one task moves the arm at a time
                    if self.stop.is_set():               # a fault landed while we waited for the arm
                        return
                    await asyncio.to_thread(self.park, name)      # the interlock: stage parked before the arm enters
                    if self.stop.is_set():
                        return
                    self.tele.log("run", "arm", f"{name} takes the arm", plates=self.plates())
                    await asyncio.to_thread(cyc.run, slot, 1, resume, stop_file)
                    self.tele.log("run", "arm", f"{name} returns the arm", plates=self.plates())
                    resume = False
                await asyncio.to_thread(scope.acquire)
                while not self.stop.is_set() and not await asyncio.to_thread(scope.done):
                    await asyncio.sleep(self.poll_s)     # other tasks use the arm meanwhile
                if self.stop.is_set():
                    return
                self.tele.log("run", "scope", f"{name}: acquisition done ({scope.last_dataset})")
        except BaseException as e:                       # also CancelledError: the others must stop too
            if self.fault is None:
                self.fault = e
            self.stop.set()
            raise

    async def run(self, slots: Dict[str, str], cycles: int, resume: bool = False,
                  stop_file: Optional[str] = None, order: Optional[list] = None) -> None:
        """Waits for every task, so no arm thread is left running when the caller disconnects the robot,
        then raises the first fault. (A cancelled task cannot stop its thread; it finishes its current step.)
        The microscope whose ledger says its step did not finish, or whose plate is in the gripper, takes the arm
        first: the other ledgers know nothing of what the gripper holds."""
        def arm_is_free(name: str) -> bool:
            s = self.cycles[name].store.s
            return s.step_done and s.plate_at != "gripper"
        names = sorted(self.scopes, key=arm_is_free)
        await asyncio.gather(*(self.run_scope(n, slots[n], cycles, resume, stop_file, order) for n in names),
                             return_exceptions=True)
        if self.fault is not None:
            raise self.fault


def borrow_fixture(layout: dict, name: str, source: str = "microscope") -> None:
    """Give `name` its own copy of `source`'s fixture and region, for running the loop on the simulator."""
    layout["fixtures"][name] = copy.deepcopy(layout["fixtures"][source])
    layout["regions"][name] = copy.deepcopy(layout["regions"][source])


def _scopes(layout: dict, names, fake_names, acquisition_s: float, sim: bool, log) -> Dict[str, object]:
    """One device per microscope; a fake waits `acquisition_s` and does nothing."""
    return {name: FakeSquid(name, acquisition_s, log) if (sim or name in fake_names)
            else Squid(name, (layout.get("squid") or {}).get(name, {}), log) for name in names}


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scopes", default="microscope", help="comma-separated microscope names (fixtures in layout.json)")
    ap.add_argument("--fake-scopes", default="", help="which of them wait and do nothing")
    ap.add_argument("--slots", default="", help="one hotel slot per microscope (default: active_slots in order)")
    ap.add_argument("--cycles", type=int, default=1, help="plate changes per microscope")
    ap.add_argument("--order", choices=("interleave",), help="walk fixtures.hotel.loaded, columns interleaved, all microscopes sharing the list")
    ap.add_argument("--acquisition-s", type=float, default=5.0, help="how long a fake microscope pretends to image")
    ap.add_argument("--mode", choices=("supervised", "unsupervised"), default="supervised")
    ap.add_argument("--sim", action="store_true", help="fake arm and fake microscopes; the vision box is still real")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--stop-file", default=None)
    ap.add_argument("--yes", action="store_true")
    return ap


def build(args, layout: dict, vision=None, tag: str = "", ref_dir: str = positions.REFS, state_dir: Optional[str] = None):
    """Everything a run needs from parsed args: (cell, robot, tele, slot_of). `vision` and `ref_dir` let a test
    pass a fake box and its own reference photos."""
    names = [s for s in args.scopes.split(",") if s]              # one microscope is the product; more are a flag
    fakes = {s for s in args.fake_scopes.split(",") if s}
    active = layout["fixtures"]["hotel"].get("active_slots", ["A1"])
    slots = [s for s in args.slots.split(",") if s] or active[:len(names)]
    if len(slots) != len(names):
        sys.exit(f"{len(names)} microscopes need {len(names)} hotel slots, got {slots}")
    slot_of = dict(zip(names, slots))
    if args.sim:
        for name in names:
            if not layout["fixtures"].get(name, {}).get("pose"):      # missing or not taught yet
                borrow_fixture(layout, name)
    else:
        problems = preflight(layout, ref_dir, names, squid=[n for n in names if n not in fakes])
        if problems:
            sys.exit("preflight failed:\n  " + "\n  ".join(problems))
    run_dir = os.path.join(positions.RUNS, time.strftime("%Y%m%d_%H%M%S") + tag + ("_sim" if args.sim else ""))
    tele = Telemetry(run_dir)
    vision = vision or nova.vision_box(layout, tele.log)
    robot = nova.arm(layout, tele.log, sim=args.sim)
    robot.connect()
    st = robot.state()
    if st.drag:
        sys.exit("drag mode is on; turn it off with the jaw closed or empty, then rerun")
    if not st.enabled and not robot.enable():
        sys.exit("robot could not be enabled")
    scopes = _scopes(layout, names, fakes, args.acquisition_s, args.sim, tele.log)
    state_dir = state_dir or os.path.join(positions.ROOT, "state_sim" if args.sim else "")
    os.makedirs(state_dir, exist_ok=True)
    cell = Workcell(layout, robot, vision, scopes, tele, args.mode, run_dir, ref_dir, state_dir,
                    layout_path=None if args.sim else positions.PATH)
    tele.log("run", "start", mode=args.mode, scopes=names, slots=slot_of, cycles=args.cycles, sim=args.sim,
             plates=cell.plates())
    return cell, robot, tele, slot_of


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    layout = positions.load()
    cell, robot, tele, slot_of = build(args, layout)
    if not args.sim and not args.yes and input(f"Plates: {cell.plates()}. Arm clear, hands away. Start? type YES: ") != "YES":
        sys.exit("not started")
    try:
        order = plate_order(layout) if args.order else None
        asyncio.run(cell.run(slot_of, args.cycles, args.resume, args.stop_file, order))
        tele.log("run", "finished", plates=cell.plates())
        return 0
    except (StopSafe, SafetyViolation) as e:
        tele.log("stop", "safe", str(e), plates=cell.plates())
        if is_motion_fault(str(e)):
            outcome = recover_to_nest(robot, next(iter(cell.cycles.values())), tele.log)
            marker = mark(os.path.dirname(tele.run_dir), f"{e} | {outcome}")
            tele.log("stop", "recovered", outcome)
            print(f"\nSTOPPED after a collision or controller fault: {e}\n{outcome}.\nThe loop does not continue; {marker} "
                  "keeps a supervisor from relaunching. Look at the cell, then delete that file.", flush=True)
        return 2
    except Exception as e:
        tele.log("stop", "crash", repr(e), plates=cell.plates())
        raise
    finally:
        cell.close()
        robot.disconnect()
        tele.close()


if __name__ == "__main__":
    sys.exit(main())
