"""
Seat tour: the plate is in the claw (long edges). For each slot given: slide in, lower, open, hold so a person can see whether the
plate sits square between the pillars, close, lift and slide out again with the plate. The plate stays in the claw between slots.
Supervised speed. If the claw is inside a slot (a stopped slide) it first backs straight out to that slot's approach point.

    python tools/seat_tour.py B7 B5 B3 B1 B9 [--hold 8]
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.safety import SpeedProfile  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("slots", nargs="*")
    ap.add_argument("--to-module", action="store_true", help="put the plate in the claw down on the rotation module at 90 degrees, then stop")
    ap.add_argument("--from-module", action="store_true", help="the plate is in the rotation module at 90 degrees: pick it first")
    ap.add_argument("--hold", type=float, default=8.0)
    ap.add_argument("--speed", default="supervised", help="a profile of layout speeds: supervised, fast, stress")
    a = ap.parse_args(argv)
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "seat_tour")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot.drag(False)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    store = StateStore(os.path.join(runs, "state.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, R.REFS)
    cyc.safety.profile = SpeedProfile(**layout["speeds"][a.speed])
    cyc.corr.ask = lambda q, t=0: (print(f"  doubt, carrying on (a person is watching): {q.strip()}", flush=True), "y")[1]
    s = store.s
    s.plate_id, s.step_done, s.hotel = "tour", True, {}
    if a.from_module:
        s.plate_at, s.grip = "polar", "none"
    else:
        s.plate_at, s.grip = "gripper", "long_edges"
    cyc.polar.angle = 90 if a.from_module else getattr(cyc.polar, "angle", 0)
    store.save()
    try:
        here = robot.pose()
        if here[0] > 230 and abs(here[1]) < 150:                 # stopped inside a slot: straight back out to the approach point
            print(f"claw is inside the rack at x {here[0]:.1f}; backing straight out", flush=True)
            cyc._move("L", [191.0, here[1], here[2]] + list(here[3:]), "back out of the rack", cyc.safety.profile.approach)
        if a.to_module:
            F = layout["fixtures"]
            cyc.move_to("polar.pose_at_90")
            cyc._place("polar", F["polar"]["pose_at_90"]["pose"], F["polar"], "polar", None, gate="polar@90")
            print("plate is on the module", flush=True)
            return 0
        if a.from_module:
            F = layout["fixtures"]
            cyc.move_to("polar.pose_at_90")
            cyc._pick("polar", F["polar"]["pose_at_90"]["pose"], F["polar"], "long_edges", None, gate="polar@90")
            print(f"picked from the module, jaw {robot.jaw():.3f} (long-edge grip reads 3.5-3.8)", flush=True)
        for slot in a.slots:
            p, approach, align, carry = cyc._hotel_geometry(slot)
            here = robot.pose()
            if here[0] > approach[0] + 20 and abs(here[2] - carry[2]) < 6:      # stopped inside a slide: straight back out
                print(f"{slot}: claw is inside the rack at x {here[0]:.1f}; backing out", flush=True)
                cyc._move("L", [approach[0], here[1], here[2]] + list(here[3:]), "back out", cyc.safety.profile.approach)
            print(f"{slot}: seat {[round(v, 1) for v in p[:3]]}; placing", flush=True)
            cyc._hotel_place(slot)
            print(f"{slot}: placed; plate sits there {a.hold:.0f} s, look", flush=True)
            time.sleep(a.hold)
            cyc._hotel_pick(slot)
            print(f"{slot}: picked again, jaw {robot.jaw():.2f}", flush=True)
    finally:
        robot.disconnect()
        tele.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
