"""
hcs-workcell command line (teaching, tests, one-microscope runs). The asyncio main loop is run.py.

  python cli.py status                              robot + vision state, read-only
  python cli.py record-pose NAME                    store the current robot pose under fixtures.<NAME> in layout.json
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
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from devices import nova  # noqa: E402
from devices.arm import MotionRefused, RobotFault  # noqa: E402
from devices.controller import BrcapiError  # noqa: E402
from devices.vision_box import VisionError  # noqa: E402
from perception.lower_check import LowerCheck  # noqa: E402
from procedures import positions  # noqa: E402
from procedures.corrector import StopSafe  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.preflight import preflight  # noqa: E402
from procedures.recorder import Recorder  # noqa: E402
from procedures.safety import SafetyViolation  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402
from routes import trajectory  # noqa: E402

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


def cmd_record_pose(args):
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    pose, jaw, joints = robot.pose(), robot.jaw(), robot.joints()
    robot.disconnect()
    node = layout["fixtures"]
    for key in args.name.split("."):
        node = node.setdefault(key, {})
    node["pose"] = [round(v, 4) for v in pose]
    node["joints_at_teach"] = [round(v, 3) for v in joints[:4]]
    node["taught_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    node["jaw_at_teach"] = round(jaw, 3)
    node.pop("_todo", None)
    # both cameras at the taught point: the first references for this plate at this seat
    try:
        v = nova.vision_box(layout, tele.log)
        d = os.path.join(REFS, "teach", args.name.replace(".", "_"))
        os.makedirs(d, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        for label, cam in (("down", layout["vision"]["down_camera"]), ("front", layout["vision"]["front_camera"])):
            img = v.frame(cam, 1280, 720, 85, None, layout["vision"]["led"])
            with open(os.path.join(d, f"{stamp}_{label}.jpg"), "wb") as f:
                f.write(img)
        node["teach_frames"] = d
        print(f"frames saved under {d}")
    except Exception as e:
        print(f"camera frames not saved: {e}")
    vendor = VENDOR_TEACH.get(args.name)
    if vendor and not args.local_only:
        name, pick = vendor
        v = nova.vision_box(layout, tele.log)
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
    v = nova.vision_box(layout, tele.log)
    ws = layout["vision"]["workspace"]
    teach_name, pick = VENDOR_TEACH[args.fixture]
    cam = layout["vision"]["down_camera"]
    if args.name in {p["name"] for p in v.positions(ws, "trigger")}:
        v.delete_position(args.name, "trigger", ws)
    v.add_trigger(args.name, pose, cam, "aruco", args.marker_mm, pick, ws, layout["vision"]["led"])
    rel = f"{args.name}-rel"
    v.add_relation(args.name, teach_name, rel, pick, ws)
    node = layout["fixtures"]
    for key in args.fixture.split("."):
        node = node[key]
    node.setdefault("vendor_triggers", []).append({"name": args.name, "relation": rel, "pose": [round(x, 4) for x in pose],
                                                   "marker_mm": args.marker_mm})
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
    from procedures.safety import Safety
    safety = Safety(layout, "supervised")                 # offline: no arm, a keep-out is checked on both elbow branches
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
    from procedures.safety import Safety
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
    safety = Safety(layout, "supervised", robot.joints)
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
    from procedures.safety import Safety
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
    safety = Safety(layout, "supervised", robot.joints)
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


def cmd_place_test(args):
    """Supervised first placement, one stage per call so the operator checks between stages:
    stage 1 route (stage -> in front of hotel), 2 pre-approach, 3 slide in, 4 lower into the slot."""
    from procedures.cycle import Cycle
    from procedures.safety import Safety
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot._r.EnableDragModeAsync(False)
        time.sleep(1.5)
        print("drag off")
    if not robot.enable():
        sys.exit("robot could not be enabled")
    safety = Safety(layout, "supervised", robot.joints)
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
    from procedures.cycle import Cycle
    from procedures.safety import Safety
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot._r.EnableDragModeAsync(False)
        time.sleep(1.5)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    safety = Safety(layout, "supervised", robot.joints)
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
    from procedures.safety import Safety
    layout = load_layout()
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot._r.EnableDragModeAsync(False)
        time.sleep(1.5)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    safety = Safety(layout, "supervised", robot.joints)
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
    if args.dry_run:
        print(f"{args.grip}: contact at {contact:.2f} (dry run, nothing stored; stored value is "
              f"{layout['gripper']['closed'].get(args.grip)})")
        return
    layout["gripper"]["closed"][args.grip] = width
    if not layout["gripper"]["open"].get(args.grip):
        layout["gripper"]["open"][args.grip] = round(width + 2.0, 2)
    if all(layout["gripper"]["closed"].values()):
        layout["gripper"].pop("_todo", None)
    save_layout(layout)
    print(f"{args.grip}: contact at {contact:.2f}, stored closed width {width:.2f}, jaw now {final:.2f}")


def cmd_record_ref(args):
    layout = load_layout()
    v = nova.vision_box(layout)
    img = v.frame(layout["vision"]["down_camera"], 1920, 1080, 85, None, layout["vision"]["led"])
    path = LowerCheck(REFS).add_reference(args.fixture, args.state, img)
    print("saved", path)


def cmd_check_lower(args):
    layout = load_layout()
    v = nova.vision_box(layout)
    lc = layout["vision"]["lower_check"]
    img = v.frame(layout["vision"]["down_camera"], 1920, 1080, 85, None, layout["vision"]["led"])
    print(LowerCheck(REFS, lc["min_refs"], lc["margin"], lc["floor"]).verdict(args.fixture, args.state, img))


def cmd_to_mp4(args):
    files = Recorder.to_mp4(os.path.join(args.run_dir, "video"), args.fps)
    print(chr(10).join(files) if files else "ffmpeg not found or no video folders")


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
    if args.profile:
        layout["speeds"][args.mode] = dict(layout["speeds"][args.profile])
    if args.route_b:
        layout["trajectories"]["microscope_to_polar"] = layout["trajectories"][args.route_b]
    _run_lock = _acquire_run_lock() if not args.sim else None   # held until exit: one live run per machine
    run_dir = os.path.join(RUNS, time.strftime("%Y%m%d_%H%M%S") + ("_sim" if args.sim else ""))
    tele = Telemetry(run_dir)
    state_dir = os.path.join(HERE, "state_sim") if args.sim else HERE      # the same ledger run.py uses
    os.makedirs(state_dir, exist_ok=True)
    store = StateStore(os.path.join(state_dir, "state_microscope.json"))
    if args.set_state:
        store.s.plate_at = "microscope" if args.set_state == "microscope" else f"hotel:{args.slot}"
        store.s.grip, store.s.step_done, store.s.last_safe_pose = "none", True, None
        store.s.polar_angle = 0 if args.set_state == "microscope" else 90
        store.save()
    vision = nova.vision_box(layout, tele.log)
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
                       fps=args.record_fps, screen=True, size=(640, 360), quality=70, log=tele.log,
                       max_gb=args.record_max_gb)
        rec.start()
    try:
        cyc = Cycle(layout, robot, vision, tele, store, args.mode, run_dir, REFS,
                    layout_path=None if args.sim else LAYOUT)
        cyc.commission = {"no_gates": bool(args.no_gates), "no_vision_correction": bool(args.no_vision_correction)}
        if args.drift_warn_only:
            cyc.drift.stop_z = float("inf")
            tele.log("run", "drift", "warn-only: drift never stops the run")
        if commission:
            tele.log("run", "COMMISSIONING", f"gates off={args.no_gates} vision correction off={args.no_vision_correction}")
    except Exception as e:
        tele.log("stop", "setup", repr(e))
        robot.disconnect()
        sys.exit(f"setup failed: {e}")
    try:
        cyc.run(args.slot, args.cycles, resume=args.resume, stop_file=args.stop_file)
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


def cmd_rotation_speed(args):
    """Try a speed for the rotation module (the arm's own axis): turn 0 -> 90 -> 0 N times, time each turn, verify the
    readback. --save writes the speed into the positions file only when every turn landed within tolerance."""
    from devices.stage import PolarError, PolarStage
    layout = load_layout()
    cfg = copy.deepcopy(layout["polar"])
    cfg["ext_axis"]["speed"] = args.speed
    tele = Telemetry(os.path.join(RUNS, "teach"))
    robot, st = connect_robot_readonly(layout, tele.log)
    if st.drag:
        sys.exit("drag mode is on; turn it off first")
    if not st.enabled and not robot.enable():
        sys.exit("robot could not be enabled")
    stage = PolarStage(cfg, robot, tele.log, True, 0)
    print(f"arm must be clear of the rotation module. speed {args.speed}, {args.turns} turns each way")
    worst = 0.0
    try:
        for i in range(args.turns):
            for deg in (90, 0):
                t0 = time.time()
                stage.rotate_to(deg)
                err = abs(stage.read_angle() - deg)
                worst = max(worst, err)
                print(f"  turn {i + 1} to {deg}: {time.time() - t0:.1f} s, readback off by {err:.2f} deg")
    except PolarError as e:
        print(f"stopped: {e}")
        return 2
    finally:
        robot.disconnect()
        tele.close()
    if args.save:
        layout["polar"]["ext_axis"]["speed"] = args.speed
        save_layout(layout)
        print(f"saved polar.ext_axis.speed = {args.speed} (worst readback error {worst:.2f} deg)")
    return 0


def cmd_forget_taught(args):
    """Blank every number taught on another bench, keep the settings: poses, routes, grip widths, learned corrections.
    Run this first on a new cell, then teach. The reference photos folder is renamed aside, not deleted."""
    layout = load_layout()
    def blank(node):
        for k in list(node):
            v = node[k]
            if k in ("pose", "aruco_reference", "joints_at_teach", "jaw_at_teach", "teach_frames", "vendor_teach", "grasp_width", "corrections"):
                node[k] = None if k != "corrections" else []
            elif isinstance(v, dict):
                blank(v)
    blank(layout["fixtures"])
    layout["fixtures"]["hotel"]["taught"] = {}
    layout["gripper"]["closed"] = {g: None for g in layout["gripper"]["closed"]}
    layout["trajectories"] = {}
    layout["routes"]["safe_z_mm"] = None
    layout["polar"]["ext_axis"]["zero_reading"] = None
    if os.path.isdir(REFS) and os.listdir(REFS) != ["README.md"]:
        os.rename(REFS, REFS + "_" + time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(REFS)
    save_layout(layout)
    print("forgot every taught number; cli.py preflight now lists everything to teach")
    return 0


def cmd_marker_check(args):
    """At the current pose: how far the fixture's marker says the arm is from the taught position. Go there first
    (goto FIXTURE); the offset is what the cycle applies before lowering."""
    layout = load_layout()
    node = layout["fixtures"]
    for part in args.fixture.split("."):
        node = node[part]
    ref = node.get("aruco_reference")
    if not ref:
        sys.exit(f"{args.fixture} has no marker reference; teach it with record-trigger")
    v = nova.vision_box(layout)
    off = v.pose_offset(layout["vision"]["down_camera"], ref, layout["vision"]["workspace"],
                        node.get("aruco_marker_id", 1), None, layout["vision"]["led"])
    mag = (off.d[0] ** 2 + off.d[1] ** 2 + off.d[2] ** 2) ** 0.5
    limit = layout["vision"]["max_correction_mm"]
    print(f"{args.fixture}: offset {[round(x, 2) for x in off.d[:3]]} mm, {mag:.2f} mm total, limit {limit} mm -> "
          f"{'ok' if mag <= limit else 'TOO FAR: re-teach the position'}")
    return 0 if mag <= limit else 2


def cmd_occupancy(args):
    """Per slot: reference counts, leave-one-out self-test, whether the detector may block a move."""
    from perception.occupancy import Occupancy
    cfg = load_layout()["vision"].get("slot_check", {})
    rows = Occupancy(REFS, cfg.get("min_refs", 5), cfg.get("min_match", 0.75)).report()
    if not rows:
        print("no slot references yet (they are collected by the running cell)")
        return 0
    print(f"{'slot':6s} {'empty':>5s} {'occupied':>8s} {'tested':>6s} {'wrong':>5s} {'min_gap':>8s}  ready")
    for r in rows:
        print(f"{r['slot']:6s} {r['empty']:5d} {r['occupied']:8d} {r['tested']:6d} {r['wrong']:5d} {r['min_gap']:8.3f}  {r['ready']}")
    return 0


def cmd_calibrate(args):
    """Camera calibration with the calibration package, the marker on the practice plate as the target.
      read     the box reads the marker (and a code, if any): ok / cannot read
      capture  the arm drives the plan around the marker, one joint solve writes CamParam.dat and CamInTool.dat
      upload   SFTP both files to the box and reboot it (credentials: layout.json vision.ssh)
      check    the marker from two tool poses: the box's robot-frame reading must not move"""
    from calibration import capture, halcon, plan, solve, upload, verify
    from devices.vision_box import VisionError
    from procedures.safety import Safety
    layout = load_layout()
    cam = args.camera
    out = args.out or os.path.join(RUNS, "calibration", cam)
    v = nova.vision_box(layout)
    led = layout["vision"]["led"]
    if args.step == "read":
        try:
            a = v.aruco_pose(cam, args.marker_id, None, led)
            print(f"ok: marker {args.marker_id} at {[round(x, 2) for x in a.target[:3]]} ({a.frame})")
        except VisionError as e:
            print(f"cannot read the marker: {e}")
            return 2
        codes, _ = v.scan(cam, 3.0, "QR", led)
        print(f"code: {codes[0] if codes else 'none'}")
        return 0
    if args.step == "capture":
        robot = make_robot(layout, args, lambda *a: None)
        robot.connect()
        try:
            safety = Safety(layout, "supervised")
            poses = plan.around(robot.pose(), lambda p: safety.check_target(p, "capture plan"))
            folder = os.path.join(out, "capture")
            n = capture.run_plan(poses, cam, robot, v, folder, led)
        finally:
            robot.disconnect()
        print(f"{n} captures in {folder}")
        start = os.path.join(out, "CamParam.dat")
        if not os.path.exists(start):
            start = os.path.join(HERE, "vendor_data", "CalibrationData", "IniIntrinsic", "CamParam.dat")
        res = solve.run(folder, halcon.CamParam.read(start), layout["vision"]["marker_mm"] / 1000.0, args.marker_id)
        res.write(out)
        print(f"CamParam.dat: focus {res.camparam.focus:.6g} m, cx {res.camparam.cx:.1f}, cy {res.camparam.cy:.1f}, "
              f"{res.reprojection_px:.3f} px; CamInTool.dat: r {[round(x, 3) for x in res.camintool.r]} "
              f"t {[round(x, 5) for x in res.camintool.t]} m, marker error {res.mean_error_m * 1e3:.3f} mm; "
              f"{res.images} captures, skipped {list(res.skipped)}, unobservable {list(res.unobservable)}")
        return 0
    if args.step == "upload":
        creds = upload.credentials(layout)
        host = layout["vision"]["host"]
        backups = upload.push(host, cam, os.path.join(out, "CamParam.dat"), os.path.join(out, "CamInTool.dat"), creds)
        print(f"uploaded to {upload.remote_dir(cam)}; backups {backups}")
        if not args.no_reboot:
            upload.reboot(host, creds)
            print("box rebooting")
        return 0
    if args.step == "check":
        robot = make_robot(layout, args, lambda *a: None)
        robot.connect()
        try:
            a = robot.pose()
            b = list(a)
            b[0] += args.move_mm
            delta = verify.check(v, cam, robot, a, b, args.marker_id, led)
            robot.move_linear(a)
        finally:
            robot.disconnect()
        print(f"verification {'passed' if delta <= args.tolerance_mm else 'FAILED'} (dT = {delta:.3f} mm over a {args.move_mm} mm move)")
        return 0 if delta <= args.tolerance_mm else 2


def cmd_setup(args):
    """The one-click setup in the terminal: each line printed, Enter on the user's, the machine's run in order."""

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
    answers = {"15 or 30": str(args.hotel_size or "")}      # what a driver knows; the hands-clear Enter is the user's go
    runner = Runner(ask=(lambda prompt: next((v for k, v in answers.items() if k in prompt), "")) if args.yes or args.auto else input,
                    stop=cell.stop_event)
    todo = Bringup(cell, squid, runner.ask).sections()
    stopped = runner.run(todo, args.start, args.step, pause_at_user=args.auto)
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


def cmd_recover(args):
    """After a hard stop or a controller fault: clear the errors, auto, enabled, then the arm loose so you can move it.
    Nothing moves. Close RobStudio first (one client at a time)."""
    layout = load_layout()
    robot = make_robot(layout, args, lambda *a: None)
    robot.connect()
    try:
        robot.clear_errors()
        left = [e for e in (robot.errors() or []) if e.get("errCode")]
        if left:
            sys.exit("the controller still reports: " + "; ".join(f"{e.get('errCode')} {e.get('errMsg', '')}" for e in left))
        robot.drag(True)
        print("errors cleared, arm enabled and loose: move it by hand; python cli.py setup locks it again")
    finally:
        robot.disconnect()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    sub.add_parser("preflight").set_defaults(fn=cmd_preflight)
    p = sub.add_parser("record-pose"); p.add_argument("name"); p.add_argument("--local-only", action="store_true")
    p.set_defaults(fn=cmd_record_pose)
    p = sub.add_parser("record-trigger"); p.add_argument("fixture", choices=sorted(VENDOR_TEACH))
    p.add_argument("name"); p.add_argument("--marker-mm", type=float, default=150)
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
    p.add_argument("--backoff", type=float, default=0.1)
    p.add_argument("--dry-run", action="store_true", help="measure and print only; keep the stored width")
    p.set_defaults(fn=cmd_measure_grip)
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
    p.add_argument("--drift-warn-only", action="store_true", help="drift statistics log warnings but never stop the run")
    p.add_argument("--profile", choices=("supervised", "unsupervised", "fast", "stress"), help="use this speed profile's numbers")
    p.add_argument("--route-b", help="trajectory name to use for microscope<->stage, e.g. microscope_to_polar_fast")
    p.add_argument("--stop-file", default=os.path.join(HERE, "runs", "STOP"), help="create this file to end the loop between cycles")
    p.add_argument("--record-fps", type=float, default=0.5)
    p.add_argument("--record-max-gb", type=float, default=8.0, help="video ring buffer per run")
    p.set_defaults(fn=cmd_run)
    p = sub.add_parser("goto", help="move to a position's hover (taught route, or lift/travel/lower at routes.safe_z_mm)")
    p.add_argument("name"); p.set_defaults(fn=cmd_goto)
    p = sub.add_parser("rotation-speed", help="try a speed for the rotation module, time the turns, verify the readback; --save keeps it")
    p.add_argument("speed", type=float); p.add_argument("--turns", type=int, default=3); p.add_argument("--save", action="store_true")
    p.set_defaults(fn=cmd_rotation_speed)
    sub.add_parser("forget-taught", help="new bench: blank every taught pose, route, width and correction, keep the settings").set_defaults(fn=cmd_forget_taught)
    p = sub.add_parser("marker-check", help="read the marker offset at the current pose (go there with goto first)")
    p.add_argument("fixture"); p.set_defaults(fn=cmd_marker_check)
    sub.add_parser("positions", help="every position, taught or not, and the hotel plate order").set_defaults(fn=cmd_positions)
    sub.add_parser("occupancy", help="slot occupancy detector: references, self-test, ready").set_defaults(fn=cmd_occupancy)
    p = sub.add_parser("calibrate", help="camera calibration: read | capture | upload | check")
    p.add_argument("camera"); p.add_argument("step", choices=("read", "capture", "upload", "check"))
    p.add_argument("--out", help="working folder (default runs/calibration/<camera>)")
    p.add_argument("--marker-id", type=int, default=1); p.add_argument("--move-mm", type=float, default=20.0)
    p.add_argument("--tolerance-mm", type=float, default=1.0); p.add_argument("--no-reboot", action="store_true")
    p.add_argument("--sim", action="store_true"); p.set_defaults(fn=cmd_calibrate)
    p = sub.add_parser("setup", help="the one-click setup: follow the printed steps, Enter after each of yours")
    p.add_argument("--sim", action="store_true"); p.add_argument("--from", dest="start", help="continue from this section")
    p.add_argument("--yes", action="store_true", help="answer every prompt with Enter (simulator tests)")
    p.add_argument("--acquisition-s", type=float, default=0.0, help="how long the practice Squid pretends to image")
    p.add_argument("--auto", action="store_true", help="a driver is watching: answer 'hands clear' with Enter, stop before each user step (exit 3)")
    p.add_argument("--step", type=int, default=0, help="with --from: the first step of that section to run")
    p.add_argument("--hotel-size", type=int, choices=(15, 30), help="with --auto/--yes: the answer to '15 or 30'")
    p.set_defaults(fn=cmd_setup)
    p = sub.add_parser("recover", help="after a hard stop or controller fault: clear errors, enable, leave the arm loose")
    p.add_argument("--sim", action="store_true"); p.set_defaults(fn=cmd_recover)
    p = sub.add_parser("to-mp4", help="assemble a run's video folders into mp4 files (needs ffmpeg)")
    p.add_argument("run_dir"); p.add_argument("--fps", type=float, default=2.0)
    p.set_defaults(fn=cmd_to_mp4)
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
