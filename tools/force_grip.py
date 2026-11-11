"""Gripper force on the plate against empty, read from the controller's torque probe (lua/torque_probe running).

With the plate on the module (short edges at 0 deg): the hand lowers to the seat, the jaw closes to the clamp command through the probe, the
normalised E1 torque is sampled while it holds, the jaw opens, the hand rises 25 mm and closes again on nothing. Prints both, repeated.
Nothing is stored. The loop must be stopped.

    python tools/force_grip.py [--grip short_edges] [--repeat 3]
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
from tools.torque_probe import job, read, require_probe  # noqa: E402


def sample(b: Brcapi, n: int = 10, dt: float = 0.1):
    rows = [read(b) for _ in range(n) if time.sleep(dt) is None]
    norm = [r["norm"] for r in rows]
    return round(min(norm), 3), round(sum(norm) / n, 3), round(max(norm), 3), rows[-1]["w"]


def probe_close(b: Brcapi, robot, clamp: float, trq: float, v: int = 20):
    """The probe only runs for the close: while a Lua program runs the SDK's moves return OK and do nothing (2026-11-02). Start it,
    close and sample the torque, stop it; then the jaw as the SDK reads it, to see whether the clamp survived the stop."""
    b.start_program()
    t0 = time.time()
    while not b.state().lua_ready or time.time() - t0 < 1.0:
        if time.time() - t0 > 8:
            raise RuntimeError("the probe did not come up")
        time.sleep(0.2)
    require_probe(b)
    try:
        code = job(b, 23, clamp, trq, v, timeout=15.0)       # MoveExtJWithForce: the close under a torque limit
    except Exception as e:
        code = f"no completion ({str(e)[:60]})"
    time.sleep(0.3)
    stats = (code,) + sample(b)
    b.stop_program()
    time.sleep(0.7)
    return stats, round(robot.jaw(), 2)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grip", default="short_edges", choices=("short_edges", "long_edges"))
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--trq", type=float, default=0.10, help="torque limit of the close, 0..1")
    a = ap.parse_args(argv)
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "force_grip")
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
    pose_name = "pose_at_90" if a.grip == "long_edges" else "pose_at_0"
    P = F[pose_name]["pose"]
    g = layout["gripper"]
    open_w, clamp = g["open"][a.grip], g["closed"][a.grip]
    try:
        print(f"module reads {cyc.polar.read_angle():.2f}; grip {a.grip}: open {open_w}, clamp command {clamp}", flush=True)
        b.stop_program()
        robot.jaw_to(open_w)
        cyc.move_to("polar." + pose_name)
        target = cyc._seat(P, [0.0] * 6)
        cyc._descend(target, "force lower", allow_floor=("polar",))
        print("  clamp on the plate            (code, min, mean, max torque, width), jaw by the SDK after the probe stops", flush=True)
        for i in range(a.repeat):
            stats, after = probe_close(b, robot, clamp, a.trq)
            print(f"    close {i + 1}: ", stats, "jaw after stop", after, flush=True)
            robot.jaw_to(open_w)
            time.sleep(0.3)
        cyc._move("L", [target[0], target[1], target[2] + 25.0] + list(target[3:]), "force rise", cyc.safety.profile.approach)
        print("  clamp on nothing (hand 25 mm up)", flush=True)
        for i in range(a.repeat):
            stats, after = probe_close(b, robot, clamp, a.trq)
            print(f"    close {i + 1}: ", stats, "jaw after stop", after, flush=True)
            robot.jaw_to(open_w)
            time.sleep(0.3)
    finally:
        try:
            b.stop_program()
            robot.jaw_to(open_w)
        except Exception:
            pass
        b.close()
        robot.disconnect()
        tele.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
