"""Pictures of a plate seated well or badly, for the tilt check at release: the arm stands at a seat exactly as it does the moment the fingers
open (lowered to the seat pose, jaw open), the operator seats the plate flat or tilts it by hand, and each state is recorded from the front
camera (and the down camera) with the pose, under reference_images/seat/<station>/<state>_<stamp>_{front,down}.jpg.

  python tools/seat_record.py go polar          the empty claw to the module's seat (module turned to 0), jaw open, lowered as at release
  python tools/seat_record.py go microscope     the same at the microscope seat
  python tools/seat_record.py go A1             the same in a hotel slot (approach, slide in, lower)
  python tools/seat_record.py record polar flat     then: record polar tilted   (tilt it another way: record polar tilted again)
  python tools/seat_record.py leave polar       lift (and for a slot slide out) when done

The arm does not move between `go` and `leave`; the operator's hands are between the fingers, so nothing here clamps or lifts on its own.
"""
from __future__ import annotations

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
os.chdir(HERE)

import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from devices.vision_box import locating  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.motion import offset  # noqa: E402
from procedures.safety import SpeedProfile  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402

OUT = os.path.join(HERE, "reference_images", "seat")
SIZE = (1280, 960)


def setup():
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "seat_record")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    store = StateStore(os.path.join(R.HERE, "state_microscope.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, R.REFS)
    cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
    cyc.corr.ask = lambda q, t=0: "y"
    return layout, tele, robot, st, store, cyc


def seat_of(cyc, layout, station):
    """The release pose of a station: (seat pose, hover pose, fixture or None for a slot)."""
    F = layout["fixtures"]
    if station == "polar":
        p = cyc._seat(F["polar"]["pose_at_0"]["pose"], [0.0] * 6)
        return p, offset(p, [0, 0, F["polar"]["hover_mm"]]), F["polar"]
    if station in F and F[station].get("pose"):
        p = cyc._seat(F[station]["pose"], [0.0] * 6)
        return p, offset(p, [0, 0, F[station]["hover_mm"]]), F[station]
    p, approach, _, carry = cyc._hotel_geometry(station)
    return p, carry, None


def go(station):
    layout, tele, robot, st, store, cyc = setup()
    try:
        if store.s.grip != "none":
            sys.exit(f"the ledger says the claw holds a plate ({store.s.grip}): not for this")
        if st.drag:
            robot.drag(False)
        if not robot.enable():
            sys.exit("robot could not be enabled")
        seat, hover, fixture = seat_of(cyc, layout, station)
        grip = "short_edges" if fixture is not None else "long_edges"
        if fixture is not None:
            cyc.move_to("polar.pose_at_0" if station == "polar" else station)
            if station == "polar":
                cyc._polar_ensure("polar", "polar@0", layout["fixtures"]["polar"]["pose_at_0"]["pose"], hover)
            cyc._jaw_open(grip, fixture)
            cyc._descend(seat, f"{station} seat", allow_floor=(station,))
        else:
            p, approach, _, carry = cyc._hotel_geometry(station)
            cyc._hotel_go_to_approach(station, f"seat {station}")
            cyc._square_up(approach, f"seat {station}")
            cyc._jaw_open(grip)
            cyc._slide(carry, f"seat {station} slide in")
            cyc._descend(p, f"seat {station} lower")
        print(f"at {station}'s seat, jaw open, as at release: {[round(v, 1) for v in robot.pose()[:3]]}. Seat the plate flat or tilt it, then: record {station} flat|tilted", flush=True)
    finally:
        robot.disconnect()
        tele.close()


def hold():
    """Keep the arm enabled with the jaw open while the operator seats and tilts the plate: at disconnect the controller drops enable and the
    unpowered fingers drift onto the plate (00:20 2026-11-04, jaw 28.5 -> 24.7). Ends when runs/HOLD_STOP exists."""
    layout = R.load_layout()
    tele = Telemetry(os.path.join(R.RUNS, "seat_record"))
    robot, st = R.connect_robot_readonly(layout, tele.log)
    stop = os.path.join(R.RUNS, "HOLD_STOP")
    if os.path.exists(stop):
        os.remove(stop)
    try:
        if st.drag:
            robot.drag(False)
        robot.enable()
        robot.jaw_to(29.0)
        print("holding: enabled, jaw open; create runs/HOLD_STOP to end", flush=True)
        while not os.path.exists(stop):
            time.sleep(1.0)
            if not robot.state().enabled:
                robot.enable()
            if robot.jaw() < 28.0:
                robot.jaw_to(29.0)
        os.remove(stop)
    finally:
        robot.disconnect()
        tele.close()


def pose_now(layout):
    """Pose (mm, deg) and jaw from the controller's JSON port: no SDK connection, so `hold` keeps its own."""
    import math
    from devices.controller import Brcapi
    b = Brcapi(layout["robot"]["ip"], log=None, wait_s=6.0)
    b.connect()
    try:
        p = b.pose_mm_rad()
        j = b.joints()
        return [p[0], p[1], p[2], math.degrees(p[3]), math.degrees(p[4]), math.degrees(p[5])], float(j[1][0])
    finally:
        b.close()


def record(station, state):
    layout = R.load_layout()
    tele = Telemetry(os.path.join(R.RUNS, "seat_record"))
    from devices.fake_arm import SimRobot                                    # only for the geometry (seat_of); the pose comes from the JSON port
    pose, jaw = pose_now(layout)
    store = StateStore(os.path.join(R.HERE, "state_microscope.json"))
    cyc = Cycle(layout, SimRobot(pose), nova.vision_box(layout, tele.log), tele, store, "supervised", os.path.join(R.RUNS, "seat_record"), R.REFS)
    try:
        seat, _, _ = seat_of(cyc, layout, station)
        far = max(abs(pose[i] - seat[i]) for i in range(3))
        if far > 5.0:
            sys.exit(f"the arm is {far:.1f} mm from {station}'s seat: not recording (go {station} first)")
        v = layout["vision"]
        box = nova.vision_box(layout, tele.log)
        d = os.path.join(OUT, station)
        os.makedirs(d, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        cams = {"front": (v["front_camera"], int(v.get("front_led", v.get("led", 0)))), "down": (v["down_camera"], int(v.get("led", 0)))}
        with locating(v["host"], wait_s=10.0):
            for label, (cam, led) in cams.items():
                if led:
                    box.led(cam, led)
                open(os.path.join(d, f"{state}_{stamp}_{label}.jpg"), "wb").write(box.frame(cam, SIZE[0], SIZE[1], 90, None, 0))
        json.dump({"station": station, "state": state, "pose": [round(x, 3) for x in pose], "jaw": round(jaw, 2), "size": SIZE,
                   "cameras": {k: cams[k][0] for k in cams}, "at": time.strftime("%Y-%m-%d %H:%M:%S")},
                  open(os.path.join(d, f"{state}_{stamp}.json"), "w", encoding="utf-8"), indent=1)
        labels = sorted({f.rsplit("_", 2)[0] for f in os.listdir(d) if f.endswith(".json")})
        n = {s: sum(f.startswith(s + "_") and f.endswith(".json") for f in os.listdir(d)) for s in labels}
        print(f"recorded {station} {state}: {d}\\{state}_{stamp}_(front|down).jpg   ({station}: " + ", ".join(f"{k} {v}" for k, v in n.items()) + ")", flush=True)
    finally:
        tele.close()


def leave(station):
    layout, tele, robot, st, store, cyc = setup()
    try:
        if st.drag:
            robot.drag(False)
        robot.enable()
        seat, hover, fixture = seat_of(cyc, layout, station)
        if fixture is not None:
            cyc._move("L", hover, f"{station} lift", cyc.safety.profile.contact)
        else:
            p, approach, _, carry = cyc._hotel_geometry(station)
            cyc._move("L", carry, f"{station} lift off", cyc.safety.profile.contact)
            cyc._move("L", approach, f"{station} slide out", cyc.safety.profile.approach)
            cyc._hotel_leave_approach(station, f"leave {station}")
        print("clear of the seat at", [round(v, 1) for v in robot.pose()[:3]], flush=True)
    finally:
        robot.disconnect()
        tele.close()


if __name__ == "__main__":
    a = sys.argv[1:]
    if a == ["hold"]:
        hold()
    elif len(a) == 2 and a[0] in ("go", "leave"):
        {"go": go, "leave": leave}[a[0]](a[1])
    elif len(a) == 3 and a[0] == "record" and a[2].replace("_", "").isalnum():     # flat, tilted, q1 .. q15 (the lip-edge types), anything the operator names
        record(a[1], a[2].lower())
    else:
        sys.exit(__doc__)
