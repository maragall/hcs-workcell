"""
nova_cell command line.

  python run.py status                              robot + vision state, read-only
  python run.py record-pose NAME                    store the current robot pose under fixtures.<NAME> in layout.json
  python run.py trace NAME                          record a hand-guided route (stop: create runs/trace.stop)
  python run.py normalise NAME                      average all traces of NAME into a waypoint list in layout.json
  python run.py record-ref FIXTURE STATE            save the down-camera view as a "safe to lower" reference
  python run.py check-lower FIXTURE STATE           score the current view against the references
  python run.py run --mode supervised|unsupervised --slot A1 [--cycles N] [--sim]
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from nova_cell import trajectory  # noqa: E402
from nova_cell.corrector import StopSafe  # noqa: E402
from nova_cell.cycle import Cycle  # noqa: E402
from nova_cell.lower_check import LowerCheck  # noqa: E402
from nova_cell.robot import SimFaults, SimRobot, VendorRobot  # noqa: E402
from nova_cell.safety import SafetyViolation  # noqa: E402
from nova_cell.state import StateStore  # noqa: E402
from nova_cell.telemetry import Telemetry  # noqa: E402
from nova_cell.vision import VisionClient  # noqa: E402

LAYOUT = os.path.join(HERE, "layout.json")
RUNS = os.path.join(HERE, "runs")
REFS = os.path.join(HERE, "reference_images")


def load_layout() -> dict:
    with open(LAYOUT, "r", encoding="utf-8") as f:
        return json.load(f)


def save_layout(layout: dict):
    tmp = LAYOUT + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(layout, f, indent=2)
    os.replace(tmp, LAYOUT)


def make_robot(layout: dict, args, log):
    if getattr(args, "sim", False):
        start = layout["fixtures"]["microscope"]["pose"] or [120, 60, 120, 0, 0, 0]
        return SimRobot(start, SimFaults(), log, jaw=layout["gripper"]["closed_on_plate"])
    r = layout["robot"]
    return VendorRobot(r["ip"], r["port"], r["grip_axis"], log)


def connect_robot_readonly(layout, log):
    robot = make_robot(layout, argparse.Namespace(sim=False), log)
    robot.connect()
    st = robot.state()
    if not st.enabled:
        log("robot", "state", "not enabled: readouts may be stale")
    return robot, st


def cmd_status(args):
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "status"))
    v = VisionClient(layout["vision"]["host"], layout["vision"]["port"], log=tele.log)
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


def cmd_record_pose(args):
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    pose, jaw = robot.pose(), robot.jaw()
    robot.disconnect()
    node = layout["fixtures"]
    for key in args.name.split("."):
        node = node.setdefault(key, {})
    node["pose"] = [round(v, 4) for v in pose]
    node["taught_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    node["jaw_at_teach"] = round(jaw, 3)
    node.pop("_todo", None)
    save_layout(layout)
    print(f"fixtures.{args.name}.pose = {node['pose']}  (jaw {jaw:.2f})")


def cmd_trace(args):
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, _ = connect_robot_readonly(layout, tele.log)
    os.makedirs(RUNS, exist_ok=True)
    tr = trajectory.record(robot, args.name, os.path.join(RUNS, "trace.stop"), 0.1, args.max_seconds)
    robot.disconnect()
    n = len(glob.glob(os.path.join(RUNS, f"trace_{args.name}_*.json"))) + 1
    path = os.path.join(RUNS, f"trace_{args.name}_{n:02d}.json")
    tr.save(path)
    print(f"saved {len(tr.t)} samples to {path}")


def cmd_normalise(args):
    layout = load_layout()
    files = sorted(glob.glob(os.path.join(RUNS, f"trace_{args.name}_*.json")))
    if not files:
        sys.exit(f"no traces named {args.name}")
    traces = [trajectory.Trace.load(f) for f in files]
    wp = trajectory.normalise(traces, max_joint_step=layout["limits"]["max_waypoint_joint_step_deg"])
    from nova_cell.safety import Safety
    safety = Safety(layout, "supervised")
    for i, p in enumerate(wp["poses"], 1):
        safety.check_target(p, f"{args.name} waypoint {i}")
    layout["trajectories"][args.name] = wp
    save_layout(layout)
    print(f"{args.name}: {wp['demos']} demos -> {len(wp['joints'])} waypoints, demo spread {wp['spread_deg']:.1f} deg, "
          f"max joint step {wp['max_step_deg']:.1f} deg")
    for i, (j, p) in enumerate(zip(wp["joints"], wp["poses"]), 1):
        print(f"  {i:2d} joints {[round(v, 1) for v in j[:4]]}  pose {[round(v, 1) for v in p[:3]]}")


def cmd_record_ref(args):
    layout = load_layout()
    v = VisionClient(layout["vision"]["host"], layout["vision"]["port"])
    img = v.frame(layout["vision"]["down_camera"], 1920, 1080, 85, None, layout["vision"]["led"])
    path = LowerCheck(REFS).add_reference(args.fixture, args.state, img)
    print("saved", path)


def cmd_check_lower(args):
    layout = load_layout()
    v = VisionClient(layout["vision"]["host"], layout["vision"]["port"])
    lc = layout["vision"]["lower_check"]
    img = v.frame(layout["vision"]["down_camera"], 1920, 1080, 85, None, layout["vision"]["led"])
    print(LowerCheck(REFS, lc["min_refs"], lc["margin"], lc["floor"]).verdict(args.fixture, args.state, img))


def cmd_run(args):
    layout = load_layout()
    missing = [k for k in ("microscope",) if not layout["fixtures"][k].get("pose")]
    if not layout["fixtures"]["polar"]["pose_at_0"].get("pose"):
        missing.append("polar.pose_at_0")
    if missing and not args.sim:
        sys.exit("poses not taught yet: " + ", ".join(missing))
    run_dir = os.path.join(RUNS, time.strftime("%Y%m%d_%H%M%S") + ("_sim" if args.sim else ""))
    tele = Telemetry(run_dir)
    store = StateStore(os.path.join(HERE, "state.json" if not args.sim else "state_sim.json"))
    vision = VisionClient(layout["vision"]["host"], layout["vision"]["port"], log=tele.log)
    robot = make_robot(layout, args, tele.log)
    robot.connect()
    st = robot.state()
    if st.drag:
        sys.exit("drag mode is on; turn it off with the jaw closed or empty, then rerun")
    if not st.enabled and not robot.enable():
        sys.exit("robot could not be enabled")
    tele.log("run", "start", mode=args.mode, slot=args.slot, cycles=args.cycles, sim=args.sim,
             state=store.s.plate_at, jaw=round(robot.jaw(), 2))
    if args.mode == "unsupervised" and layout["polar"]["driver"] == "manual":
        sys.exit("unsupervised mode needs a real polar stage driver (layout.polar.driver)")
    if not args.sim and input("Arm clear, plate where state.json says, hands away. Start? type YES: ") != "YES":
        sys.exit("not started")
    cyc = Cycle(layout, robot, vision, tele, store, args.mode, run_dir, REFS)
    try:
        cyc.run(args.slot, args.cycles)
        tele.log("run", "finished", state=store.s.plate_at)
        return 0
    except (StopSafe, SafetyViolation) as e:
        tele.log("stop", "safe", str(e), state=store.s.plate_at, grip=store.s.grip)
        return 2
    finally:
        robot.disconnect()
        tele.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    p = sub.add_parser("record-pose"); p.add_argument("name"); p.set_defaults(fn=cmd_record_pose)
    p = sub.add_parser("trace"); p.add_argument("name"); p.add_argument("--max-seconds", type=float, default=900)
    p.set_defaults(fn=cmd_trace)
    p = sub.add_parser("normalise"); p.add_argument("name"); p.set_defaults(fn=cmd_normalise)
    p = sub.add_parser("record-ref"); p.add_argument("fixture"); p.add_argument("state"); p.set_defaults(fn=cmd_record_ref)
    p = sub.add_parser("check-lower"); p.add_argument("fixture"); p.add_argument("state"); p.set_defaults(fn=cmd_check_lower)
    p = sub.add_parser("run")
    p.add_argument("--mode", choices=("supervised", "unsupervised"), required=True)
    p.add_argument("--slot", default="A1")
    p.add_argument("--cycles", type=int, default=1)
    p.add_argument("--sim", action="store_true")
    p.set_defaults(fn=cmd_run)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
