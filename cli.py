"""
hcs-workcell command line (teaching, tests, one-microscope runs). The asyncio main loop is run.py.

  python cli.py status                              robot + vision state, read-only
  python cli.py record-pose NAME                    store the current robot pose under fixtures.<NAME> in layout.json
  python cli.py teach [NAME]                        arm loose, hand it onto the plate in the microscope, Enter: seat + routes to the module stored
  python cli.py trace NAME                          record a hand-guided route (stop: create runs/trace.stop)
  python cli.py normalise NAME                      average all traces of NAME into a waypoint list in layout.json
  python cli.py record-ref FIXTURE STATE            save the down-camera view as a "safe to lower" reference
  python cli.py check-lower FIXTURE STATE           score the current view against the references
  python cli.py run --mode supervised|unsupervised --slot A1 [--cycles N] [--sim]
  python cli.py calibrate CAMERA read|capture|upload|check      camera calibration (package calibration/)
  python cli.py recover                             clear controller errors after a hard stop, leave the arm loose
  python cli.py setup [--sim] [--from SECTION]      the one-click setup: printed steps, Enter after each of yours
"""
from __future__ import annotations

import argparse
import copy
import glob
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from devices import nova  # noqa: E402
from devices.arm import MotionRefused, RobotFault  # noqa: E402
from devices.controller import BrcapiError  # noqa: E402
from devices.vision_box import VisionError  # noqa: E402
from procedures import positions  # noqa: E402
from procedures.corrector import StopSafe  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.preflight import preflight  # noqa: E402
from procedures.safety import SafetyViolation  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402

LAYOUT, RUNS, REFS = positions.PATH, positions.RUNS, positions.REFS
load_layout, save_layout = positions.load, positions.save


def make_robot(layout: dict, args, log):
    return nova.arm(layout, log, sim=getattr(args, "sim", False))


def connect_robot_readonly(layout, log):
    robot = nova.arm(layout, log)
    robot.connect()
    st = robot.state()
    if not st.enabled:
        log("robot", "state", "not enabled: readouts may be stale")
    return robot, st


def cmd_status(args):
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "status"))
    v = nova.vision_box(layout, tele.log)
    print("vision :", v.version())
    print("status :", v.status())
    print("cameras:", v.cameras())
    print("leds   :", v.led_status())
    robot, st = connect_robot_readonly(layout, tele.log)
    print("robot  :", st)
    print("pose   :", [round(x, 3) for x in robot.pose()])
    print("joints :", [round(x, 2) for x in robot.joints()])
    print("jaw    :", round(robot.jaw(), 3))
    robot.disconnect()


# vendor workspace names: the vision box is the system of record for taught positions.
# pick_method follows the vendor vocabulary: "wide" = fingers on the long edges, "long" = fingers on the short edges.
VENDOR_TEACH = {
    "microscope": ("microscope-short", "long"),
    "polar.pose_at_0": ("polar0-short", "long"),
    "polar.pose_at_90": ("polar90-wide", "wide"),
}


















def _settle(robot, target, label, max_dev, timeout=120):
    t0 = time.time()
    while robot.state().moving and time.time() - t0 < timeout:
        time.sleep(0.1)
    after = robot.pose()
    dev = max(abs(after[k] - target[k]) for k in range(3))
    print(f"  {label}: at {[round(v, 1) for v in after[:3]]} yaw {after[5]:.1f}  dev {dev:.1f} mm  {time.time() - t0:.1f}s")
    if dev > max_dev:
        raise SystemExit(f"STOP: {label} ended {dev:.1f} mm from target (limit {max_dev}); possible contact")
    return after
















def _acquire_run_lock():
    """One live run per machine: the controller accepts one client and two loops would fight over the arm."""
    import msvcrt
    path = os.path.join(HERE, "runs", "run.lock")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    f = open(path, "a+")
    try:
        msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
    except OSError:
        f.seek(0)
        sys.exit(f"another run holds {path} (pid {f.read().strip() or '?'}); refusing to start a second loop")
    f.seek(0)
    f.truncate()
    f.write(str(os.getpid()))
    f.flush()
    return f




def cmd_goto(args):
    """Move to a position's hover: a taught route if there is one, else lift / travel at routes.safe_z_mm / lower."""
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        sys.exit("drag mode is on; turn it off first")
    if not st.enabled and not robot.enable():
        sys.exit("robot could not be enabled")
    store = StateStore(os.path.join(RUNS, "teach", "state_goto.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", os.path.join(RUNS, "teach"), REFS)
    try:
        cyc.move_to(args.name)
    except (StopSafe, SafetyViolation) as e:
        tele.log("stop", "safe", str(e))
        return 2
    finally:
        robot.disconnect()
        tele.close()
    return 0


def cmd_positions(args):
    """Every position the arm can be sent to, taught or not, and the hotel plate order."""
    from procedures.hotel import plate_order
    layout = load_layout()
    F = layout["fixtures"]
    def show(name, node):
        pose = node.get("pose")
        where = " ".join(f"{v:8.2f}" for v in pose[:3]) + f"  yaw {pose[5]:7.2f}" if pose else "NOT TAUGHT"
        print(f"{name:20s} {where}   hover {node.get('hover_mm', '-')}  route {node.get('trajectory', '-')}")
    for name, node in F.items():
        if name == "hotel" or name.startswith("_"):
            continue
        if "pose" in node:
            show(name, node)
        for sub, child in node.items():
            if isinstance(child, dict) and "pose" in child:
                show(f"{name}.{sub}", child)
    h = F["hotel"]
    print(f"hotel: columns {h['columns']} rows {h['rows'][0]}..{h['rows'][-1]}, taught {sorted(h['taught'])}, "
          f"lift {h['lift_mm']} mm, headroom {h.get('headroom_mm', '-')} mm")
    print("plate order:", " ".join(plate_order(layout)) or "(fixtures.hotel.loaded is empty)")
    return 0










from bringup.colour import plain as _plain


def _tee_console(name: str) -> str:
    """Everything printed from here on also goes to runs/<name>_console_<time>.log, so a run that fails can be read afterwards."""
    os.makedirs(RUNS, exist_ok=True)
    path = os.path.join(RUNS, f"{name}_console_{time.strftime('%Y%m%d_%H%M%S')}.log")
    log = open(path, "a", encoding="utf-8", buffering=1)

    class Tee:
        def __init__(self, stream):
            self.stream = stream

        def write(self, text):
            self.stream.write(text)
            log.write(_plain(text))

        def flush(self):
            self.stream.flush()
            log.flush()

        def __getattr__(self, item):
            return getattr(self.stream, item)
    sys.stdout, sys.stderr = Tee(sys.stdout), Tee(sys.stderr)
    return path


def cmd_setup(args):
    """The one-click setup in the terminal: each line printed, Enter on the user's, the machine's run in order."""
    if getattr(args, "list", False):                      # the sections and their step numbers, so a run can start where it is needed
        from bringup import sim
        from bringup.cell import Cell
        from bringup.sections import Bringup
        from devices.fake_squid import FakeSquid
        cell = Cell(*sim.build(), sim=True)
        for sec in Bringup(cell, FakeSquid("microscope", 0.0), lambda p: "").sections():
            print(f"{sec.name}")
            for i, st in enumerate(sec.steps):
                print(f"   step {i}: {st.actor} : {st.text[:95]}")
        print(chr(10) + 'start in a section:  python cli.py setup --from "<section>"   at a step:  --step N   stop early:  --stop-after N')
        cell.frames.stop()
        return 0
    log_path = _tee_console("setup")
    print(f"this run is logged to {log_path} (and runs/setup_*/events.jsonl)")

    from bringup import prompt as prompt_mod
    from bringup.runner import Runner
    from bringup.sections import Bringup
    from devices.fake_squid import FakeSquid
    from bringup import sim
    from bringup.cell import Cell
    if args.sim:
        cell = Cell(*sim.build(), sim=True)
    else:
        layout = load_layout()
        cell = Cell(LAYOUT, REFS, HERE, RUNS, make_robot(layout, args, lambda *a: None), nova.vision_box(layout), sim=False)
    squid = FakeSquid("microscope", args.acquisition_s)       # the practice Squid until the real one is wired in (devices/squid.py)
    cell.connect()
    cell.discover()
    if not args.sim and cell.arm_error:
        sys.exit(f"the arm is not reachable: {cell.arm_error}. Close the GUI and RobStudio (one client at a time) and run again")
    answers = {"slot in front of the camera": "yes" if args.hotel_size == 30 else "no" if args.hotel_size == 15 else ""}   # what a driver knows; the hands-clear Enter is the user's go
    runner = Runner(ask=(lambda prompt: next((v for k, v in answers.items() if k in prompt), "")) if args.yes or args.auto else prompt_mod.ask,
                    stop=cell.stop_event, on_section_done=save_known_good)
    bring = Bringup(cell, squid, runner.ask, confirm_stage=args.confirm_stage, speed=getattr(args, "speed", "stress"))
    todo = [bring.shuffle(args.moves, getattr(args, "speed", "stress"), getattr(args, "probe", 0))] if getattr(args, "shuffle", False) else bring.sections()
    stopped = runner.run(todo, args.start, args.step, pause_at_user=args.auto, skip=args.skip.split(",") if args.skip else None,
                         stop_after=args.stop_after)
    cell.frames.stop()
    if runner.waiting:
        name, nxt = runner.waiting
        print(f"\nwaiting for the user. After they have done it: python cli.py setup --auto --from \"{name}\" --step {nxt}")
        return 3
    if stopped:
        print(f"\nstopped in '{stopped}'. Fix what it said, then: python cli.py setup --from \"{stopped}\"")
        return 2
    print("\nsetup done")
    return 0


def cmd_teach(args):
    """Teach an instrument's seat by hand (arm loose, Enter) and plan its routes to the rotation module: bringup/teach.py."""
    from bringup import teach
    return teach.run(args.name, lambda layout: make_robot(layout, args, lambda *a: None))


def cmd_shuffle(args):
    """Random plates through the microscope into random empty taught slots, for ever: no setup. Stop: runs/STOP or Ctrl+C."""
    args.shuffle = True
    try:
        os.remove(os.path.join(RUNS, "STOP"))          # the last run's stop request does not stop this one
    except OSError:
        pass
    return cmd_setup(args)




def cmd_doctor(args):
    """Is this PC and this cell ready? Read-only; prints PASS/FAIL lines with the fix for each."""
    from tools import doctor
    return doctor.main(["--sim"] if args.sim else [])


KNOWN_GOOD = os.path.join(RUNS, "known_good")
KNOWN_FILES = ("state.json", "state_microscope.json")


def save_known_good(section: str) -> None:
    """When a section of the setup has run every step, keep the layout and the state files as they are then: a later failure can
    go back to them (`python cli.py restore "<section>"`) and resume from the next section."""
    import shutil
    dest = os.path.join(KNOWN_GOOD, section.replace(" ", "_").replace(".", "_"))
    os.makedirs(dest, exist_ok=True)
    for f in (LAYOUT,) + tuple(os.path.join(HERE, n) for n in KNOWN_FILES):
        if os.path.exists(f):
            shutil.copy2(f, os.path.join(dest, os.path.basename(f)))
    with open(os.path.join(dest, "saved_at.txt"), "w", encoding="utf-8") as fh:
        fh.write(time.strftime("%Y-%m-%d %H:%M:%S") + f" after {section}" + chr(10))
    print(f"   (known good after {section}: saved to {dest})")




def cmd_recover(args):
    """After a hard stop or a controller fault: clear the errors, auto, enabled, then the arm loose so you can move it, or with
    --nest the arm locked and taken to the nest (routes.nest: a pose in the clear in front of the hotel; it backs out along the
    claw first if it stopped against something). Close RobStudio first (one client at a time)."""
    layout = load_layout()
    robot = make_robot(layout, args, lambda *a: None)
    robot.connect()
    try:
        robot.clear_errors()
        left = [e for e in (robot.errors() or []) if e.get("errCode")]
        if left:
            sys.exit("the controller still reports: " + "; ".join(f"{e.get('errCode')} {e.get('errMsg', '')}" for e in left))
        if args.nest:
            robot.enable()
            print("errors cleared, arm enabled and locked; going to the nest")
        else:
            robot.drag(True)
            print("errors cleared, arm enabled and loose: move it by hand; python cli.py setup locks it again")
    finally:
        robot.disconnect()
    if not args.nest:
        return 0
    from bringup import sim
    from bringup.cell import Cell
    from bringup.runner import Refused
    from bringup.sections import Bringup
    from devices.fake_squid import FakeSquid
    cell = Cell(*sim.build(), sim=True) if args.sim else Cell(LAYOUT, REFS, HERE, RUNS, make_robot(layout, args, lambda *a: None),
                                                             nova.vision_box(layout), sim=False)
    cell.connect()
    try:
        todo = Bringup(cell, FakeSquid("microscope", 0.0), lambda prompt: "")
        todo.ready()
        todo.guard(todo.cyc().go_nest)
        print("at the nest")
        return 0
    except Refused as e:
        print(f"could not reach the nest: {e}")
        return 2
    finally:
        cell.frames.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    p = sub.add_parser("goto", help="move to a position's hover (taught route, or lift/travel/lower at routes.safe_z_mm)")
    p.add_argument("name"); p.set_defaults(fn=cmd_goto)
    sub.add_parser("positions", help="every position, taught or not, and the hotel plate order").set_defaults(fn=cmd_positions)
    p = sub.add_parser("setup", help="the one-click setup: follow the printed steps, Enter after each of yours")
    p.add_argument("--sim", action="store_true"); p.add_argument("--from", dest="start", help="continue from this section"); p.add_argument("--list", action="store_true", help="list the sections and their step numbers, then exit")
    p.add_argument("--yes", action="store_true", help="answer every prompt with Enter (simulator tests)")
    p.add_argument("--acquisition-s", type=float, default=0.0, help="how long the practice Squid pretends to image")
    p.add_argument("--auto", action="store_true", help="a driver is watching: answer 'hands clear' with Enter, stop before each user step (exit 3)")
    p.add_argument("--stop-after", type=int, help="with --from: stop after this step (index) of that section")
    p.add_argument("--confirm-stage", action="store_true", help="the operator confirms, for this run, that lowering at the stage is safe (its lower-check has no pictures yet)")
    p.add_argument("--skip", help="sections to leave out, comma separated, e.g. cameras.toml")
    p.add_argument("--step", type=int, default=0, help="with --from: the first step of that section to run")
    p.add_argument("--hotel-size", type=int, choices=(15, 30), help="with --auto/--yes: the hotel you have, answers 'is there a slot here' for every row")
    p.add_argument("--speed", default="stress", choices=("supervised", "fast", "stress"), help="speed profile (layout.json speeds) of the test section; the setup steps stay at supervised")
    p.set_defaults(fn=cmd_setup, moves=0)
    p = sub.add_parser("teach", help="arm loose; move it by hand onto the plate in the instrument's holder, Enter: seat and routes to the module stored")
    p.add_argument("name", nargs="?", default="microscope"); p.set_defaults(fn=cmd_teach)
    p = sub.add_parser("shuffle", help="random plates through the microscope into random empty taught slots, for ever (occupancy = fixtures.hotel.loaded)")
    p.add_argument("--sim", action="store_true"); p.add_argument("--moves", type=int, default=0, help="stop after this many moves (default: for ever)")
    p.add_argument("--probe", type=int, default=0, help="every N moves, photograph every slot's approach with the hand empty and with a plate in it (data for the classifiers)")
    p.add_argument("--speed", default="stress", choices=("stress", "fast", "supervised"), help="speed profile from layout.json (default stress; supervised when models are in the loop)")
    p.set_defaults(fn=cmd_shuffle, start=None, list=False, yes=False, acquisition_s=0.0, auto=False, stop_after=None, confirm_stage=False, skip=None, step=0, hotel_size=None)
    p = sub.add_parser("doctor", help="check this PC and the cell are ready (read-only)")
    p.add_argument("--sim", action="store_true"); p.set_defaults(fn=cmd_doctor)
    p = sub.add_parser("recover", help="after a hard stop or controller fault: clear errors, enable, leave the arm loose")
    p.add_argument("--sim", action="store_true")
    p.add_argument("--nest", action="store_true", help="lock the arm and take it to the nest")
    p.set_defaults(fn=cmd_recover)
    args = ap.parse_args()
    try:
        return args.fn(args)
    except (RobotFault, VisionError, BrcapiError, MotionRefused, StopSafe, SafetyViolation, OSError) as e:
        if os.environ.get("HCS_TRACEBACK"):
            raise
        sys.exit(f"{type(e).__name__}: {e}")
    except (KeyError, TypeError, IndexError) as e:
        if os.environ.get("HCS_TRACEBACK"):
            raise
        sys.exit(f"a value is missing in layout.json ({type(e).__name__} {e}); `python cli.py positions` and `preflight` list what is not taught")


if __name__ == "__main__":
    sys.exit(main())
