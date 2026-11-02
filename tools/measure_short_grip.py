"""
Measure a grip on the plate that sits on the rotation module (short edges at 0 deg, long edges at 90): the jaw opens, the hand goes to the module hover
(stage "hover"), and with --lower it lowers to the seat, closes in steps from the open width until the jaw stops following the command
(contact), prints every step live, opens again and rises. Nothing is stored. The loop must be stopped.

    python tools/measure_short_grip.py            to the hover, a frame saved, nothing lowered
    python tools/measure_short_grip.py --lower    lower, measure, release, rise
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from devices.arm import MotionRefused  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.safety import SpeedProfile  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lower", action="store_true")
    ap.add_argument("--grip", default="short_edges", choices=("short_edges", "long_edges"), help="long_edges: the plate on the module at 90 deg")
    ap.add_argument("--step", type=float, default=0.25)
    ap.add_argument("--pause", type=float, default=0.3)
    a = ap.parse_args(argv)
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "measure_grip")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot.drag(False)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    store = StateStore(os.path.join(runs, "state.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, R.REFS)
    cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
    cyc.corr.ask = lambda q, t=0: "y"
    s = store.s
    s.plate_at, s.grip, s.step_done, s.hotel = "polar", "none", True, {}
    store.save()
    F = layout["fixtures"]["polar"]
    pose_name = "pose_at_90" if a.grip == "long_edges" else "pose_at_0"
    P0 = F[pose_name]["pose"]
    g = layout["gripper"]
    try:
        open_w = g["open"][a.grip]
        print(f"jaw now {robot.jaw():.2f}; opening to {open_w}", flush=True)
        robot.jaw_to(open_w)
        time.sleep(1.0)
        if not robot.pose()[2] > P0[2] + F["hover_mm"] - 3.0 or abs(robot.pose()[0] - P0[0]) > 5 or abs(robot.pose()[1] - P0[1]) > 5:
            cyc.move_to("polar." + pose_name)
        img = cyc._frame(cyc.down_cam)
        print("down-camera frame:", tele.image("measure_short_hover", img), flush=True)
        if not a.lower:
            print("at the hover; nothing lowered. Look at the frame, then run with --lower", flush=True)
            return 0
        target = cyc._seat(P0, [0.0] * 6)
        print(f"lowering to the seat z {target[2]:.2f}", flush=True)
        cyc._descend(target, "measure lower", allow_floor=("polar",))
        v = robot.jaw()
        contact, cmd_at_contact = None, None
        print(f"start {v:.2f}; step {a.step}, pause {a.pause}s; contact when the read lags the command by > {g['stall_lag']}", flush=True)
        while v > 0.2:
            v = round(v - a.step, 3)
            try:
                robot.jaw_to(v)
            except MotionRefused:
                contact, cmd_at_contact = robot.jaw(), v
                print(f"  cmd {v:6.2f} refused; the jaw holds at {contact:.2f}", flush=True)
                break
            time.sleep(a.pause)
            read = robot.jaw()
            print(f"  cmd {v:6.2f}  read {read:6.2f}  lag {read - v:+.2f}", flush=True)
            if read - v > g["stall_lag"]:
                contact, cmd_at_contact = read, v
                break
        print("", flush=True)
        if contact is None:
            print("RESULT: the jaw closed all the way without meeting a plate: nothing between the fingers", flush=True)
        else:
            print(f"RESULT: contact at {contact:.2f} (command {cmd_at_contact:.2f} at that moment); stored closed width {g['closed'][a.grip]}, "
                  f"band {g['accept_band'][a.grip]}", flush=True)
        robot.jaw_to(open_w)
        time.sleep(1.5)
        cyc._move("L", [target[0], target[1], target[2] + F["hover_mm"]] + list(target[3:]), "measure rise", cyc.safety.profile.approach)
        print(f"released and back at the hover; jaw {robot.jaw():.2f}", flush=True)
    finally:
        robot.disconnect()
        tele.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
