"""
Dry visit to hotel seats: the hand empty and open goes to the slot's approach point, slides in and lowers to the seat height, holds there
(front camera frame saved) so a person can see whether the open fingers straddle the slot, then lifts and backs out. Nothing is gripped.
The arm must be locked and the hand empty. Slow (the supervised profile).

    python tools/dry_hotel.py B1 B3 B9 [--hold 20]
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
    ap.add_argument("slots", nargs="+")
    ap.add_argument("--hold", type=float, default=20.0)
    a = ap.parse_args(argv)
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "dry_hotel")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot.drag(False)                    # left loose by the teaching: it locks where it stands
    if not robot.enable():
        sys.exit("robot could not be enabled")
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, StateStore(os.path.join(runs, "state.json")), "supervised", runs, R.REFS)
    cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
    try:
        for slot in a.slots:
            p, approach, align, carry = cyc._hotel_geometry(slot)
            print(f"{slot}: seat {[round(v, 1) for v in p[:3]]} yaw {p[5]:.1f}; to the approach point", flush=True)
            cyc._hotel_go_to_approach(slot, "dry")
            cyc._jaw_open("long_edges")
            cyc._slide(carry, f"dry {slot} slide in")
            cyc._descend(p, f"dry {slot} lower to the seat")
            tele.image(f"dry_{slot}_seat", cyc._frame(cyc.front_cam))
            print(f"{slot}: at the seat, holding {a.hold:.0f} s", flush=True)
            time.sleep(a.hold)
            cyc._move("L", carry, f"dry {slot} lift", cyc.safety.profile.approach)
            cyc._move("L", approach, f"dry {slot} slide out", cyc.safety.profile.approach)
            cyc._hotel_leave_approach(slot, "dry")
    finally:
        robot.disconnect()
        tele.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
