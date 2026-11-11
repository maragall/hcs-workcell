"""Occupancy references, recorded by hand, station by station: the arm looks at a station, the operator makes it empty or puts a plate in,
and says which. Both cameras, full frames, with the pose, under reference_images/occupancy/<station>/.

  python utils/occupancy_record.py go A1            the empty claw to A1's approach point, by the planner (the front camera looks into the slot)
  python utils/occupancy_record.py go polar         to the module's taught viewing pose (utils/view_pose.py); microscope the same
  python utils/occupancy_record.py record A1 empty  pictures of A1 as it is now, labelled empty; then `record A1 full` once a plate is in
  python utils/occupancy_record.py round empty      every taught slot in turn, one recording each (the hotel empty); `round full A` for one column
  python utils/occupancy_record.py list             what has been recorded

A recording is refused when the arm is not within 5 mm of the station's viewing pose, so a label never goes with the wrong view. The
window's pictures pause for the second the capture takes (utils/devices/vision_box.locating).
"""
from __future__ import annotations

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
os.chdir(HERE)

from run import positions  # noqa: E402
from utils.devices import nova  # noqa: E402
from utils.devices.vision_box import locating  # noqa: E402
from run.cycle import Cycle  # noqa: E402
from run.planner.safety import SpeedProfile  # noqa: E402
from run.state import StateStore  # noqa: E402
from run.telemetry import Telemetry  # noqa: E402
from vision.view import from_tool_frame, to_view  # noqa: E402

OUT = os.path.join(HERE, "vision", "reference_images", "occupancy")
SIZE = (1280, 960)


def cameras_for(station: str):
    """Which camera sees the station: a slot only the front camera (at a slot's mouth the down camera looks at the rotation module); the
    module and the microscope only the down camera (their front frames show the room). Operator, 2026-11-03."""
    return ("down",) if station in ("polar", "microscope") else ("front",)


def view_of(cyc, layout, station):
    """The viewing pose of a station: a slot's approach point, or a taught view pose (fixtures.<station>.view)."""
    F = layout["fixtures"]
    if station in F and F[station].get("view", {}).get("tool_offset"):
        seat = F["polar"]["pose_at_0"]["pose"] if station == "polar" else F[station]["pose"]
        fwd, left, up = F[station]["view"]["tool_offset"]
        return from_tool_frame(seat, fwd, left, up)
    if station in F:
        sys.exit(f"fixtures.{station}.view is not taught: python utils/view_pose.py go {station}, adjust, save {station}")
    _, approach, _, _ = cyc._hotel_geometry(station)
    return approach


def setup():
    layout = positions.load()
    runs = os.path.join(positions.RUNS, "occupancy_record")
    tele = Telemetry(runs)
    robot, st = nova.connect_readonly(layout, tele.log)
    store = StateStore(os.path.join(positions.ROOT, "state_microscope.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, positions.REFS)
    cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
    cyc.corr.ask = lambda q, t=0: "y"
    return layout, tele, robot, st, store, cyc


def go(station):
    layout, tele, robot, st, store, cyc = setup()
    try:
        if store.s.grip != "none":
            sys.exit(f"the ledger says the claw holds a plate ({store.s.grip}): the viewing round is for an empty claw")
        if st.drag:
            robot.drag(False)
        if not robot.enable():
            sys.exit("robot could not be enabled")
        target = view_of(cyc, layout, station)
        if station in layout["fixtures"]:
            cyc.move_to("polar.pose_at_0" if station == "polar" else station)
            to_view(cyc, station, target)
        else:
            cyc._hotel_go_to_approach(station, f"view {station}")
            cyc._square_up(target, f"view {station}")
        print(f"at {station}'s view: {[round(v, 1) for v in robot.pose()[:3]]} yaw {robot.pose()[5]:.1f}. Make it empty or put a plate in, then: record {station} empty|full", flush=True)
    finally:
        robot.disconnect()
        tele.close()


def capture(layout, tele, robot, cyc, station, state):
    pose = robot.pose()
    target = view_of(cyc, layout, station)
    far = max(abs(pose[i] - target[i]) for i in range(3))
    if far > 5.0:
        sys.exit(f"the arm is {far:.1f} mm from {station}'s viewing pose: not recording (go {station} first)")
    v = layout["vision"]
    box = nova.vision_box(layout, tele.log)
    d = os.path.join(OUT, station)
    os.makedirs(d, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    cams = {"front": (v["front_camera"], int(v.get("front_led", v.get("led", 0)))), "down": (v["down_camera"], int(v.get("led", 0)))}
    with locating(v["host"], wait_s=10.0):
        for label in cameras_for(station):
            cam, led = cams[label]
            if led:
                box.led(cam, led)                                             # the lamp as the window keeps it: on for the session, never a flash per frame
            jpg = box.frame(cam, SIZE[0], SIZE[1], 90, None, 0)
            open(os.path.join(d, f"{state}_{stamp}_{label}.jpg"), "wb").write(jpg)
    json.dump({"station": station, "state": state, "pose": [round(x, 3) for x in pose], "view_pose": [round(x, 3) for x in target], "size": SIZE,
               "cameras": {k: cams[k][0] for k in cameras_for(station)}, "led": {k: cams[k][1] for k in cameras_for(station)},
               "at": time.strftime("%Y-%m-%d %H:%M:%S")}, open(os.path.join(d, f"{state}_{stamp}.json"), "w", encoding="utf-8"), indent=1)
    print(f"recorded {station} {state}: {d}\\{state}_{stamp}_{'|'.join(cameras_for(station))}.jpg", flush=True)


def record(station, state):
    layout, tele, robot, st, store, cyc = setup()
    try:
        capture(layout, tele, robot, cyc, station, state)
        print(summary(station), flush=True)
    finally:
        robot.disconnect()
        tele.close()


def hotel_round(state):
    """Every taught slot in turn, the empty claw: to its mouth by the planner, one recording labelled `state`. For the whole hotel empty,
    or a column just filled by hand (then only its slots: `round full A`)."""
    layout, tele, robot, st, store, cyc = setup()
    try:
        if store.s.grip != "none":
            sys.exit(f"the ledger says the claw holds a plate ({store.s.grip}): the round is for an empty claw")
        if st.drag:
            robot.drag(False)
        if not robot.enable():
            sys.exit("robot could not be enabled")
        only = sys.argv[3] if len(sys.argv) > 3 else ""
        h = layout["fixtures"]["hotel"]
        slots = [s for s in h.get("active_slots", []) if s.startswith(only)]
        if only and not slots and only in h.get("columns", []):                 # a column outside the taught set (C): its slots from the hotel's rows
            slots = [f"{only}{r}" for r in h["rows"]]
        print("round", state, "over", slots, flush=True)
        for slot in slots:
            target = view_of(cyc, layout, slot)
            cyc._hotel_go_to_approach(slot, f"view {slot}")
            cyc._square_up(target, f"view {slot}")
            capture(layout, tele, robot, cyc, slot, state)
        print(summary(), flush=True)
    finally:
        robot.disconnect()
        tele.close()


def sweep():
    """Every slot of every column with the empty claw: a verdict per slot from the station's empty references (vision/occupancy.py),
    printed and shown live in the window; the picture is kept as sweep_<stamp> (not a reference; the operator says what was true)."""
    import cv2
    import numpy as np
    from vision.occupancy import Station
    layout, tele, robot, st, store, cyc = setup()
    try:
        if st.drag:
            robot.drag(False)
        if not robot.enable():
            sys.exit("robot could not be enabled")
        jaw = robot.jaw()
        if jaw < 1.0 and store.s.grip != "none":                              # fingers together: nothing between them, whatever the ledger says
            store.s.grip = "none"
            store.save()
        if store.s.grip != "none":
            sys.exit(f"the ledger says the claw holds a plate ({store.s.grip}) and the jaw reads {jaw:.1f}: not sweeping")
        if jaw < 20.0:
            robot.jaw_to(29.0)
        h = layout["fixtures"]["hotel"]
        v = layout["vision"]
        box = nova.vision_box(layout, tele.log)
        rows = []
        for slot in [f"{c}{r}" for c in h.get("columns", []) for r in h.get("rows", [])]:
            target = view_of(cyc, layout, slot)
            cyc._hotel_go_to_approach(slot, f"sweep {slot}")
            cyc._square_up(target, f"sweep {slot}")
            time.sleep(0.6)                                                   # settled before the picture: a frame taken as the arm arrives reads as unsure
            with locating(v["host"], wait_s=10.0):
                if int(v.get("front_led", 0)):
                    box.led(v["front_camera"], int(v.get("front_led", 0)))
                jpg = box.frame(v["front_camera"], SIZE[0], SIZE[1], 90, None, 0)
            stn = Station.load(slot)
            if stn.empties:
                vd = stn.verdict(cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR))
                line = f"{slot}: {vd.state}  (score {vd.score:.2f}, noise {vd.noise:.2f}{'' if stn.calibrated else '*'}, {len(stn.empties)} empties)"
            else:
                line = f"{slot}: no empty reference"
            d = os.path.join(OUT, slot)
            os.makedirs(d, exist_ok=True)
            open(os.path.join(d, f"sweep_{time.strftime('%Y%m%d_%H%M%S')}_front.jpg"), "wb").write(jpg)
            print("   " + line, flush=True)
            rows.append(line)
            time.sleep(1.5)                                                   # long enough to read the label in the window
        print("\n".join(rows))
    finally:
        robot.disconnect()
        tele.close()


def summary(station=None):
    rows = []
    for s in sorted(os.listdir(OUT)) if os.path.isdir(OUT) else []:
        if (station and s != station) or not os.path.isdir(os.path.join(OUT, s)):
            continue
        files = os.listdir(os.path.join(OUT, s))
        rows.append(f"  {s}: empty {sum(f.startswith('empty_') and f.endswith('.json') for f in files)}, full {sum(f.startswith('full_') and f.endswith('.json') for f in files)}")
    return "\n".join(rows) or "  nothing recorded yet"


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[:1] == ["list"]:
        print(summary())
    elif a == ["sweep"]:
        sweep()
    elif len(a) == 2 and a[0] == "go":
        go(a[1])
    elif len(a) == 3 and a[0] == "record" and a[2] in ("empty", "full"):
        record(a[1], a[2])
    elif len(a) in (2, 3) and a[0] == "round" and a[1] in ("empty", "full"):
        hotel_round(a[1])
    else:
        sys.exit(__doc__)
