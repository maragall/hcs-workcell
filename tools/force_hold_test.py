"""Does the force clamp survive the probe stopping and a lift? Plate on the module (short edges at 0 deg).
The hand lowers, closes under the torque limit through the probe, the probe stops, the SDK lifts the plate, the probe starts again and reads
the holding torque and the width, stops; the hand lowers, releases, rises. Prints every reading. The loop must be stopped.

    python tools/force_hold_test.py [--trq 0.5] [--lift 30]
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from devices.controller import Brcapi  # noqa: E402
from run.cycle import Cycle  # noqa: E402
from run.safety import SpeedProfile  # noqa: E402
from run.state import StateStore  # noqa: E402
from run.telemetry import Telemetry  # noqa: E402
from tools.force_grip import probe_close, sample  # noqa: E402
from tools.torque_probe import require_probe  # noqa: E402


def probe_read(b):
    b.start_program()
    t0 = time.time()
    while not b.state().lua_ready or time.time() - t0 < 1.0:
        if time.time() - t0 > 8:
            raise RuntimeError("the probe did not come up")
        time.sleep(0.2)
    require_probe(b)
    s = sample(b)
    b.stop_program()
    time.sleep(0.7)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trq", type=float, default=0.5)
    ap.add_argument("--lift", type=float, default=30.0)
    a = ap.parse_args()
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "force_hold")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot.drag(False)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    b = Brcapi(layout["robot"]["ip"], layout["robot"].get("brcapi_port", 56788), log=None)
    b.connect()
    store = StateStore(os.path.join(runs, "state.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, R.REFS)
    cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
    cyc.corr.ask = lambda q, t=0: "y"
    s = store.s
    s.plate_at, s.grip, s.step_done, s.hotel = "polar", "none", True, {}
    store.save()
    F = layout["fixtures"]["polar"]
    P = F["pose_at_0"]["pose"]
    g = layout["gripper"]
    open_w, clamp = g["open"]["short_edges"], g["closed"]["short_edges"]
    try:
        b.stop_program()
        robot.jaw_to(open_w)
        cyc.move_to("polar.pose_at_0")
        target = cyc._seat(P, [0.0] * 6)
        cyc._descend(target, "hold lower", allow_floor=("polar",))
        stats, after = probe_close(b, robot, clamp, a.trq)
        print("close on the plate :", stats, "jaw after stop", after, flush=True)
        up = [target[0], target[1], target[2] + a.lift] + list(target[3:])
        cyc._move("L", up, "hold lift", cyc.safety.profile.approach)
        print("lifted; jaw by the SDK", round(robot.jaw(), 2), flush=True)
        print("torque in the air  :", probe_read(b), flush=True)
        time.sleep(3.0)
        print("jaw after 3 s      :", round(robot.jaw(), 2), "torque", probe_read(b), flush=True)
        cyc._move("L", [target[0], target[1], target[2] + 3.0] + list(target[3:]), "hold lower again", cyc.safety.profile.approach)
        cyc._move("L", list(target), "hold seat", cyc.safety.profile.approach)
        robot.jaw_to(open_w)
        time.sleep(0.5)
        cyc._move("L", up, "hold rise", cyc.safety.profile.approach)
        print("released and risen; jaw", round(robot.jaw(), 2), flush=True)
    finally:
        try:
            b.stop_program()
            robot.jaw_to(open_w)
        except Exception:
            pass
        b.close()
        robot.disconnect()
        tele.close()


if __name__ == "__main__":
    main()
