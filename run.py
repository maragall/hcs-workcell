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
from nova_cell.recorder import Recorder  # noqa: E402
from nova_cell.robot import MotionRefused, SimFaults, SimRobot, VendorRobot  # noqa: E402
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


# vendor workspace names: the vision box is the system of record for taught positions.
# pick_method follows the vendor vocabulary: "wide" = fingers on the long edges, "long" = fingers on the short edges.
VENDOR_TEACH = {
    "microscope": ("microscope-short", "long"),
    "polar.pose_at_0": ("polar0-short", "long"),
    "polar.pose_at_90": ("polar90-wide", "wide"),
}


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
    vendor = VENDOR_TEACH.get(args.name)
    if vendor and not args.local_only:
        name, pick = vendor
        v = VisionClient(layout["vision"]["host"], layout["vision"]["port"], log=tele.log)
        ws = layout["vision"]["workspace"]
        existing = {p["name"]: p for p in v.positions(ws, "teach")}
        if name in existing:
            v.delete_position(name, "teach", ws)
        v.add_teach(name, pose, pick, ws)
        stored = {p["name"]: p for p in v.positions(ws, "teach")}.get(name)
        node["vendor_teach"] = {"name": name, "pick_method": pick, "id": stored["id"] if stored else None,
                                "workspace": ws}
        print(f"vision box: teach point '{name}' ({pick}) stored as {node['vendor_teach']['id']}")
    save_layout(layout)
    print(f"fixtures.{args.name}.pose = {node['pose']}  (jaw {jaw:.2f})")


def cmd_record_trigger(args):
    """Store the current pose as a vendor trigger point (camera over the marker) and relate it to a teach point."""
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    pose = robot.pose()
    robot.disconnect()
    v = VisionClient(layout["vision"]["host"], layout["vision"]["port"], log=tele.log)
    ws = layout["vision"]["workspace"]
    teach_name, pick = VENDOR_TEACH[args.fixture]
    cam = layout["vision"]["down_camera"]
    if args.name in {p["name"] for p in v.positions(ws, "trigger")}:
        v.delete_position(args.name, "trigger", ws)
    v.add_trigger(args.name, pose, cam, "aruco", args.caltab_mm, pick, ws, layout["vision"]["led"])
    rel = f"{args.name}-rel"
    v.add_relation(args.name, teach_name, rel, pick, ws)
    node = layout["fixtures"]
    for key in args.fixture.split("."):
        node = node[key]
    node.setdefault("vendor_triggers", []).append({"name": args.name, "relation": rel, "pose": [round(x, 4) for x in pose],
                                                   "caltab_mm": args.caltab_mm})
    save_layout(layout)
    print(f"vision box: trigger '{args.name}' -> teach '{teach_name}' via relation '{rel}'")


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


def cmd_polar(args):
    """polar jog <deg> | polar zero | polar goto <0|90>  (external axis; drag must be off)."""
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        sys.exit("drag mode is on; turn it off first")
    a = layout["polar"]["ext_axis"]
    idx = int(a["index"])
    now = robot.ext_axis(idx)
    if args.what == "open":
        if args.value <= 0:
            sys.exit("give the opening value")
        robot.ext_axis_to(0, args.value)
        time.sleep(2.0)
        print(f"jaw: {robot.jaw():.2f}")
    elif args.what == "jog":
        robot.set_speed(float(a["speed"]))
        robot.ext_axis_to(idx, now + args.value)
        time.sleep(abs(args.value) / 5.0 + 1.0)
        print(f"axis {idx + 1}: {now:.2f} -> {robot.ext_axis(idx):.2f}")
    elif args.what == "zero":
        layout["polar"]["ext_axis"]["zero_reading"] = round(now, 3)
        layout["polar"]["ext_axis"].pop("_todo", None)
        save_layout(layout)
        print(f"polar zero reference = {now:.3f} (axis reading when parallel to x)")
    elif args.what == "goto":
        if a.get("zero_reading") is None:
            sys.exit("zero not set; run polar zero first")
        target = a["zero_reading"] + a["direction"] * args.value
        robot.set_speed(float(a["speed"]))
        robot.ext_axis_to(idx, target)
        time.sleep(abs(target - now) / 5.0 + 1.0)
        print(f"axis {idx + 1}: {now:.2f} -> {robot.ext_axis(idx):.2f} (target {target:.2f})")
    robot.disconnect()


def cmd_preflight(args):
    problems = preflight(load_layout(), REFS)
    print("READY: all poses, widths, trajectories and references present" if not problems else
          "missing:" + chr(10) + chr(10).join("  " + p for p in problems))
    return 1 if problems else 0


def cmd_move_z(args):
    """Vertical move of the arm by <delta_mm> at contact speed, drag off, with the safety checks. Nothing else moves."""
    from nova_cell.safety import Safety
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        sys.exit("drag mode is on; turn it off first")
    if not st.enabled and not robot.enable():
        sys.exit("robot could not be enabled")
    here = robot.pose()
    target = list(here)
    target[2] += args.delta_mm
    safety = Safety(layout, "supervised")
    region = safety.region_of(target)
    try:
        safety.check_segment(here, target, "move-z", allow_floor_regions=(region,) if args.allow_floor else ())
    except Exception as e:
        robot.disconnect()
        sys.exit(f"refused: {e}")
    print(f"z {here[2]:.1f} -> {target[2]:.1f} at {args.speed * 100:.0f} % (region {region})")
    robot.set_speed(args.speed)
    robot.move_linear(target)
    t0 = time.time()
    while robot.state().moving and time.time() - t0 < 60:
        time.sleep(0.1)
    after = robot.pose()
    print(f"now z {after[2]:.1f} (deviation {max(abs(after[i] - target[i]) for i in range(3)):.2f} mm)")
    robot.disconnect()


def cmd_replay(args):
    """Replay a normalised trajectory waypoint by waypoint at travel speed, drag off, confirming each step."""
    from nova_cell.safety import Safety
    layout = load_layout()
    wp = layout["trajectories"].get(args.name)
    if not wp:
        sys.exit(f"trajectory {args.name} not normalised yet")
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        sys.exit("drag mode is on; turn it off first")
    if not st.enabled and not robot.enable():
        sys.exit("robot could not be enabled")
    safety = Safety(layout, "supervised")
    pts = list(zip(wp["joints"], wp["poses"]))
    if args.reverse:
        pts.reverse()
    here = robot.pose()
    first = pts[0][1]
    gap = max(abs(here[i] - first[i]) for i in range(3))
    print(f"arm at {[round(v, 1) for v in here[:3]]}, first waypoint {[round(v, 1) for v in first[:3]]}, gap {gap:.1f} mm")
    if gap > args.start_tolerance:
        robot.disconnect()
        sys.exit(f"arm is {gap:.0f} mm from the first waypoint; drag it there first (tolerance {args.start_tolerance})")
    robot.set_speed(args.speed)
    for i, (j, p) in enumerate(pts, 1):
        safety.check_target(p, f"{args.name} waypoint {i}")
        if args.step and input(f"  waypoint {i}/{len(pts)} -> {[round(v, 1) for v in p[:3]]}  Enter to go, q to quit: ").strip().lower() == "q":
            break
        t0 = time.time()
        robot.move_abs_joints_stepped(list(j[:4]))
        after = robot.pose()
        dev = max(abs(after[k] - p[k]) for k in range(3))
        print(f"  {i:2d}/{len(pts)} pose {[round(v, 1) for v in after[:3]]} dev {dev:.1f} mm  {time.time() - t0:.1f}s")
        if dev > args.max_dev:
            print(f"  stopped: deviation {dev:.1f} mm exceeds {args.max_dev}")
            break
    robot.disconnect()


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


def _confirm(text):
    if input(f"  >>> {text}  [y = go / anything else = stop here]: ").strip().lower() != "y":
        raise SystemExit("stopped by operator; arm holds position")


def cmd_place_test(args):
    """Supervised first placement, one stage per call so the operator checks between stages:
    stage 1 route (stage -> in front of hotel), 2 pre-approach, 3 slide in, 4 lower into the slot."""
    from nova_cell.cycle import Cycle
    from nova_cell.safety import Safety
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot._r.EnableDragModeAsync(False)
        time.sleep(1.5)
        print("drag off")
    if not robot.enable():
        sys.exit("robot could not be enabled")
    safety = Safety(layout, "supervised")
    geom = Cycle.__new__(Cycle)
    geom.L = layout
    slot_pose = Cycle.hotel_slot_pose(geom, args.slot)
    h = layout["fixtures"]["hotel"]
    ax, mm, lift = h["approach"]["axis"], h["approach"]["mm"], h["lift_mm"]
    approach = [slot_pose[0] + ax[0] * mm, slot_pose[1] + ax[1] * mm, slot_pose[2] + lift] + list(slot_pose[3:])
    carry = [slot_pose[0], slot_pose[1], slot_pose[2] + lift] + list(slot_pose[3:])
    wp = layout["trajectories"]["polar_to_hotel"]
    pts = list(zip(wp["joints"], wp["poses"]))
    for i, (j, pp) in enumerate(pts, 1):
        safety.check_target(pp, f"route waypoint {i}")
    for lbl, pp in (("pre-approach", approach), ("carry", carry), ("seat", slot_pose)):
        safety.check_target(pp, lbl, allow_floor_regions=("hotel",))

    def expect_near(pose, tol, what):
        here = robot.pose()
        gap = max(abs(here[i] - pose[i]) for i in range(3))
        print(f"arm {[round(v, 1) for v in here[:3]]} yaw {here[5]:.1f} jaw {robot.jaw():.2f}; {what} gap {gap:.1f} mm")
        if gap > tol:
            robot.disconnect()
            sys.exit(f"arm is {gap:.0f} mm from the {what}; not running this stage")

    robot.set_speed(args.speed)
    try:
        if args.stage == 1:
            expect_near(pts[0][1], args.start_tolerance, "route start")
            for i, (j, pp) in enumerate(pts, 1):
                robot.move_abs_joints_stepped(list(j[:4]))
                _settle(robot, pp, f"waypoint {i}/{len(pts)}", args.max_dev)
            print(f"stage 1 done. Next: stage 2 = joint move to pre-approach {[round(v, 1) for v in approach[:3]]} "
                  f"with the claw turning to yaw {approach[5]:.1f}")
        elif args.stage == 2:
            expect_near(pts[-1][1], 20.0, "route end")
            robot.move_joint(approach)
            _settle(robot, approach, "pre-approach", 2.0)
            print("stage 2 done. Check the alignment with the slot by eye. Next: stage 3 = slide in +x at carry height")
        elif args.stage == 3:
            expect_near(approach, 5.0, "pre-approach")
            robot.move_linear(carry)
            _settle(robot, carry, "slide in", 2.0)
            print(f"stage 3 done. Next: stage 4 = lower {lift:.0f} mm into the slot")
        elif args.stage == 4:
            expect_near(carry, 5.0, "carry point")
            robot.move_linear(slot_pose)
            _settle(robot, slot_pose, "seat", 2.0)
            print(f"plate in {args.slot}; fingers still closed at jaw {robot.jaw():.2f}. Arm holds.")
    finally:
        robot.disconnect()


def cmd_hotel_cycle_test(args):
    """Supervised: pick from the slot, put it back, pick again, route back to the rotating stage, place at -90, lift."""
    from nova_cell.cycle import Cycle
    from nova_cell.safety import Safety
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot._r.EnableDragModeAsync(False)
        time.sleep(1.5)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    safety = Safety(layout, "supervised")
    g = layout["gripper"]
    geom = Cycle.__new__(Cycle)
    geom.L = layout
    slot = Cycle.hotel_slot_pose(geom, args.slot)
    h = layout["fixtures"]["hotel"]
    ax, mm, lift = h["approach"]["axis"], h["approach"]["mm"], h["lift_mm"]
    approach = [slot[0] + ax[0] * mm, slot[1] + ax[1] * mm, slot[2] + lift] + list(slot[3:])
    carry = [slot[0], slot[1], slot[2] + lift] + list(slot[3:])
    wp = layout["trajectories"]["polar_to_hotel"]
    pts = list(zip(wp["joints"], wp["poses"]))
    seat = pts[0][1]
    above_seat = pts[1][1]
    hover = layout["fixtures"]["polar"]["hover"]["pose"]
    pz = layout["polar"]["ext_axis"]
    stage_target = pz["zero_reading"] + pz["direction"] * 90

    def say(label):
        p = robot.pose()
        print(f"  {label:26s} {[round(v, 1) for v in p[:3]]} yaw {p[5]:.1f} jaw {robot.jaw():.2f}", flush=True)

    def close_on(grip):
        want = g["closed"][grip]
        v = robot.jaw()
        while v > want + 1e-6:
            v = max(want, v - 0.5)
            try:
                robot.jaw_to(v)
            except MotionRefused:
                break
            time.sleep(0.15)
        time.sleep(0.4)
        jaw = robot.jaw()
        if abs(jaw - want) > g["tolerance"]:
            raise SystemExit(f"STOP: grasp check failed, jaw {jaw:.2f} expected {want} +- {g['tolerance']}")
        say(f"closed ({grip})")

    def lin(p, label, speed):
        safety.check_target(p, label, allow_floor_regions=("hotel", "polar"))
        robot.set_speed(speed)
        robot.move_linear(p)
        say(label)

    fast, mid, slow = args.speed, args.speed / 2, args.speed / 3
    try:
        say("start")
        robot.set_speed(fast)
        robot.jaw_to(g["open"]["long_edges"])
        lin(approach, "pre-approach", fast)
        for rep, action in enumerate(("pick", "place", "pick"), 1):
            lin(carry, f"{rep} slide in", mid)
            lin(slot, f"{rep} lower", slow)
            if action == "pick":
                close_on("long_edges")
            else:
                robot.jaw_to(g["open"]["long_edges"])
                time.sleep(0.5)
                say("released")
            lin(carry, f"{rep} lift", slow)
            lin(approach, f"{rep} slide out", mid)
        robot.set_speed(fast)
        robot.move_joint(pts[-1][1])
        say("route end (joint)")
        for i, (j, pp) in reversed(list(enumerate(pts, 1))):
            if i == 1:
                break
            robot.move_abs_joints_stepped(list(j[:4]))
            say(f"route wp {i}")
        angle = robot.ext_axis(int(pz["index"]))
        if abs(angle - stage_target) > 2.0:
            raise SystemExit(f"STOP: rotating stage at {angle:.1f}, expected {stage_target:.1f} for the long-edge seat")
        lin(seat, "stage seat", slow)
        robot.jaw_to(g["open"]["long_edges"])
        time.sleep(0.5)
        say("released on stage")
        lin(hover, "stage hover", slow)
        print("done: plate on the rotating stage at -90, arm at the hover, fingers open")
    finally:
        robot.disconnect()


def cmd_scope_place_test(args):
    """Supervised: from the rotating-stage hover with the plate held on its SHORT edges, replay route B in reverse
    to the microscope hover, lower onto the holder, release, lift. Leaves the cell in the cycle's start state."""
    from nova_cell.safety import Safety
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot._r.EnableDragModeAsync(False)
        time.sleep(1.5)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    safety = Safety(layout, "supervised")
    g = layout["gripper"]
    m = layout["fixtures"]["microscope"]
    seat, hover = m["pose"], m["hover"]["pose"]
    wp = layout["trajectories"]["microscope_to_polar"]
    pts = list(zip(wp["joints"], wp["poses"]))[::-1]

    def say(label):
        p = robot.pose()
        print(f"  {label:22s} {[round(v, 1) for v in p[:3]]} yaw {p[5]:.1f} jaw {robot.jaw():.2f}", flush=True)

    jaw = robot.jaw()
    if abs(jaw - g["closed"]["short_edges"]) > g["tolerance"]:
        robot.disconnect()
        sys.exit(f"jaw {jaw:.2f}: plate not held on its short edges ({g['closed']['short_edges']})")
    here = robot.pose()
    gap = max(abs(here[i] - pts[0][1][i]) for i in range(3))
    print(f"route B end gap {gap:.1f} mm")
    try:
        robot.set_speed(args.speed)
        if gap > 3.0:
            robot.move_joint(pts[0][1])
            say("to route end")
        for i, (j, pp) in enumerate(pts, 1):
            robot.move_abs_joints_stepped(list(j[:4]))
            if i % 5 == 0 or i == len(pts):
                say(f"route wp {len(pts) - i + 1}")
        here = robot.pose()
        gap = max(abs(here[i] - hover[i]) for i in range(3))
        if gap > 3.0:
            safety.check_target(hover, "microscope hover")
            robot.set_speed(args.speed / 3)
            robot.move_joint(hover)
        say("microscope hover")
        safety.check_segment(robot.pose(), seat, "lower to seat", allow_floor_regions=("microscope",))
        robot.set_speed(args.speed / 4)
        robot.move_linear(seat)
        say("seat")
        robot.jaw_to(g["open"]["short_edges"])
        time.sleep(0.5)
        say("released")
        robot.move_linear(hover)
        say("hover (empty)")
        print("done: plate on the microscope holder, arm at the hover. Start the cycle with --set-state microscope.")
    finally:
        robot.disconnect()


def cmd_measure_grip(args):
    """Close in steps on a plate held between the fingers until the jaw stops following; store the width."""
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        sys.exit("drag mode is on; gripper commands are refused in drag mode")
    g = layout["gripper"]
    v = robot.jaw()
    print(f"start {v:.2f}, stepping {args.step} every {args.pause}s; lag > {g['stall_lag']} means contact")
    contact = None
    while v > 0.2:
        v = round(v - args.step, 3)
        try:
            robot.jaw_to(v)
        except MotionRefused:
            contact = robot.jaw()          # the controller refuses a step the servo cannot complete: fingers on the plate
            print(f"  cmd {v:5.2f} refused; jaw holds at {contact:.2f}")
            break
        time.sleep(args.pause)
        read = robot.jaw()
        print(f"  cmd {v:5.2f}  read {read:5.2f}  lag {read - v:+.2f}")
        if read - v > g["stall_lag"]:
            contact = read
            break
    if contact is None:
        robot.jaw_to(g["open"])
        robot.disconnect()
        sys.exit("jaw closed fully without meeting a plate; nothing measured, jaw reopened")
    width = round(contact, 2)
    final = robot.jaw()
    robot.disconnect()
    layout["gripper"]["closed"][args.grip] = width
    if not layout["gripper"]["open"].get(args.grip):
        layout["gripper"]["open"][args.grip] = round(width + 2.0, 2)
    if all(layout["gripper"]["closed"].values()):
        layout["gripper"].pop("_todo", None)
    save_layout(layout)
    print(f"{args.grip}: contact at {contact:.2f}, stored closed width {width:.2f}, jaw now {final:.2f}")


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


def cmd_to_mp4(args):
    files = Recorder.to_mp4(os.path.join(args.run_dir, "video"), args.fps)
    print(chr(10).join(files) if files else "ffmpeg not found or no video folders")


def _walk(node, path=""):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from _walk(v, f"{path}.{k}" if path else k)
    else:
        yield path, node


def preflight(layout: dict, ref_dir: str) -> list:
    problems = [p for p, v in _walk(layout) if p.endswith("_todo")]
    F = layout["fixtures"]
    for name, pose in (("microscope", F["microscope"].get("pose")),
                       ("polar.pose_at_0", F["polar"]["pose_at_0"].get("pose")),
                       ("polar.pose_at_90", F["polar"]["pose_at_90"].get("pose"))):
        if not pose:
            problems.append(f"pose not taught: {name}")
    for grip, w in layout["gripper"]["closed"].items():
        if not w:
            problems.append(f"grip width not measured: {grip}")
    for grip, w in layout["gripper"]["open"].items():
        if not w:
            problems.append(f"open width not set: {grip}")
    if layout["polar"]["ext_axis"].get("zero_reading") is None:
        problems.append("polar zero reference not set")
    for name, t in layout["trajectories"].items():
        if not t:
            problems.append(f"trajectory not taught: {name}")
    box = layout["regions"]["microscope"]["box"]
    if box["x"][0] >= box["x"][1] or box["y"][0] >= box["y"][1]:
        problems.append("regions.microscope box not set")
    need = layout["vision"]["lower_check"]["min_refs"]
    for gate, state in (("microscope", "plate_present"), ("microscope", "empty"), ("polar@0", "plate_present"),
                        ("polar@0", "empty"), ("polar@90", "plate_present"), ("polar@90", "empty"),
                        ("gripper", "short_edges"), ("gripper", "long_edges")):
        d = os.path.join(ref_dir, gate, state)
        n = len([f for f in os.listdir(d)]) if os.path.isdir(d) else 0
        if n < need:
            problems.append(f"reference images {gate}/{state}: {n}/{need}")
    return problems


def cmd_run(args):
    layout = load_layout()
    commission = args.no_gates or args.no_vision_correction
    if commission and args.mode != "supervised":
        sys.exit("commissioning flags are only allowed in supervised mode")
    if not args.sim:
        problems = preflight(layout, REFS)
        if args.no_gates:
            problems = [p for p in problems if not p.startswith("reference images")]
        if problems:
            sys.exit("preflight failed:" + chr(10) + "  " + (chr(10) + "  ").join(problems))
    if args.mode == "supervised" and not sys.stdin.isatty() and not args.allow_no_tty:
        sys.exit("supervised mode needs an interactive terminal (or --allow-no-tty: operator questions default to stop)")
    if args.speeds:
        tr, ap_, ct = (float(x) for x in args.speeds.split(","))
        layout["speeds"][args.mode].update({"travel": tr, "approach": ap_, "contact": ct})
    run_dir = os.path.join(RUNS, time.strftime("%Y%m%d_%H%M%S") + ("_sim" if args.sim else ""))
    tele = Telemetry(run_dir)
    store = StateStore(os.path.join(HERE, "state.json" if not args.sim else "state_sim.json"))
    if args.set_state:
        store.s.plate_at = "microscope" if args.set_state == "microscope" else f"hotel:{args.slot}"
        store.s.grip, store.s.step_done, store.s.last_safe_pose = "none", True, None
        store.s.polar_angle = 0 if args.set_state == "microscope" else 90
        store.save()
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
    if not args.sim and not args.yes and input("Arm clear, plate where state.json says, hands away. Start? type YES: ") != "YES":
        sys.exit("not started")
    rec = None
    if args.record:
        rec = Recorder(run_dir, vision, [layout["vision"]["down_camera"], layout["vision"]["front_camera"]],
                       fps=args.record_fps, screen=True, log=tele.log)
        rec.start()
    try:
        cyc = Cycle(layout, robot, vision, tele, store, args.mode, run_dir, REFS)
        cyc.commission = {"no_gates": bool(args.no_gates), "no_vision_correction": bool(args.no_vision_correction)}
        if commission:
            tele.log("run", "COMMISSIONING", f"gates off={args.no_gates} vision correction off={args.no_vision_correction}")
    except Exception as e:
        tele.log("stop", "setup", repr(e))
        robot.disconnect()
        sys.exit(f"setup failed: {e}")
    try:
        cyc.run(args.slot, args.cycles, resume=args.resume)
        tele.log("run", "finished", state=store.s.plate_at)
        return 0
    except (StopSafe, SafetyViolation) as e:
        tele.log("stop", "safe", str(e), state=store.s.plate_at, grip=store.s.grip)
        return 2
    except Exception as e:
        tele.log("stop", "crash", repr(e), state=store.s.plate_at, grip=store.s.grip)
        cyc.corr.alert("crash", repr(e))
        raise
    finally:
        if rec:
            rec.stop()
        robot.disconnect()
        tele.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    sub.add_parser("preflight").set_defaults(fn=cmd_preflight)
    p = sub.add_parser("record-pose"); p.add_argument("name"); p.add_argument("--local-only", action="store_true")
    p.set_defaults(fn=cmd_record_pose)
    p = sub.add_parser("record-trigger"); p.add_argument("fixture", choices=sorted(VENDOR_TEACH))
    p.add_argument("name"); p.add_argument("--caltab-mm", type=float, default=150)
    p.set_defaults(fn=cmd_record_trigger)
    p = sub.add_parser("trace"); p.add_argument("name"); p.add_argument("--max-seconds", type=float, default=900)
    p.set_defaults(fn=cmd_trace)
    p = sub.add_parser("normalise"); p.add_argument("name"); p.set_defaults(fn=cmd_normalise)
    p = sub.add_parser("polar"); p.add_argument("what", choices=("jog", "zero", "goto", "open"))
    p.add_argument("value", type=float, nargs="?", default=0.0); p.set_defaults(fn=cmd_polar)
    p = sub.add_parser("move-z"); p.add_argument("delta_mm", type=float); p.add_argument("--speed", type=float, default=0.01)
    p.add_argument("--allow-floor", action="store_true", help="permit going below the region floor (seat moves)")
    p.set_defaults(fn=cmd_move_z)
    p = sub.add_parser("replay"); p.add_argument("name"); p.add_argument("--reverse", action="store_true")
    p.add_argument("--speed", type=float, default=0.20); p.add_argument("--step", action="store_true")
    p.add_argument("--start-tolerance", type=float, default=15.0); p.add_argument("--max-dev", type=float, default=5.0)
    p.set_defaults(fn=cmd_replay)
    p = sub.add_parser("place-test"); p.add_argument("slot", default="A1", nargs="?")
    p.add_argument("--stage", type=int, choices=(1, 2, 3, 4), required=True)
    p.add_argument("--speed", type=float, default=0.20); p.add_argument("--max-dev", type=float, default=5.0)
    p.add_argument("--start-tolerance", type=float, default=40.0); p.set_defaults(fn=cmd_place_test)
    p = sub.add_parser("hotel-cycle-test"); p.add_argument("slot", default="A1", nargs="?")
    p.add_argument("--speed", type=float, default=0.30); p.set_defaults(fn=cmd_hotel_cycle_test)
    p = sub.add_parser("scope-place-test"); p.add_argument("--speed", type=float, default=0.30)
    p.set_defaults(fn=cmd_scope_place_test)
    p = sub.add_parser("measure-grip"); p.add_argument("grip", choices=("short_edges", "long_edges"))
    p.add_argument("--step", type=float, default=0.25); p.add_argument("--pause", type=float, default=0.3)
    p.add_argument("--backoff", type=float, default=0.1); p.set_defaults(fn=cmd_measure_grip)
    p = sub.add_parser("record-ref"); p.add_argument("fixture"); p.add_argument("state"); p.set_defaults(fn=cmd_record_ref)
    p = sub.add_parser("check-lower"); p.add_argument("fixture"); p.add_argument("state"); p.set_defaults(fn=cmd_check_lower)
    p = sub.add_parser("run")
    p.add_argument("--mode", choices=("supervised", "unsupervised"), required=True)
    p.add_argument("--slot", default="A1")
    p.add_argument("--cycles", type=int, default=1)
    p.add_argument("--sim", action="store_true")
    p.add_argument("--record", action="store_true", help="record both cameras and the screen for the whole run")
    p.add_argument("--no-gates", action="store_true", help="COMMISSIONING, supervised only: skip the image gates")
    p.add_argument("--no-vision-correction", action="store_true", help="COMMISSIONING, supervised only: taught poses only")
    p.add_argument("--allow-no-tty", action="store_true"); p.add_argument("--yes", action="store_true")
    p.add_argument("--set-state", choices=("microscope", "hotel"), help="declare where the plate is before starting")
    p.add_argument("--resume", action="store_true", help="continue an interrupted carry from the nearest route waypoint")
    p.add_argument("--speeds", help="override travel,approach,contact fractions for this run, e.g. 0.30,0.15,0.05")
    p.add_argument("--record-fps", type=float, default=2.0)
    p.set_defaults(fn=cmd_run)
    p = sub.add_parser("to-mp4", help="assemble a run's video folders into mp4 files (needs ffmpeg)")
    p.add_argument("run_dir"); p.add_argument("--fps", type=float, default=2.0)
    p.set_defaults(fn=cmd_to_mp4)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
