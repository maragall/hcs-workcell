"""The places each workcell is shown by hand during the setup: the two seats of the rotation module and six hotel slots.

The module's seats and the hotel's place differ from one workcell to the next; the hotel's shape (column pitch, row pitch) does not.
So eight hand placements with a plate, the jaw set a hair wider than the plate so the pads slide into its grooves:

    record(robot, layout)            where the arm is now, as the taught records are kept
    refine_yaw(cell, grip, read)     the module seat's angle by force: where clamping the seated plate does not twist the module
    fit_hotel(layout)                the hotel as a rigid box through the six taught slots: three shifts, a turn and two leans

The yaw sweep is what found the long-edge seat on 2026-11-10 (0.75 deg off by hand; the module's motor was at full torque and the plate
bucked on release). The six slots are the fewest that reproduced the 15-slot answer to 0.2 mm in depth on that workcell's own records
(three corners: 1.1 mm, four: 0.4 mm).
"""
from __future__ import annotations

import math
import time
from typing import Callable, Dict, List

import numpy as np

SLOTS = ["A1", "A9", "B1", "B9", "C1", "C9"]
COLUMN = {"A": 1, "B": 0, "C": -1}                  # column A is +y, as in fixtures.hotel.grid
SLIDE_ON_MM = 0.1                                   # the jaw this much wider than the plate while the arm is loose (22.48 around the 22.38 plate, 1.54 around 1.46)
STEP_DEG, REACH_DEG, MAX_CLOSES = 0.25, 10.0, 8      # the smallest correction; how far from the hand's angle the seat may turn out to be; closes before giving up
PUSHED_DEG, FREE_TORQUE = 0.05, 0.5                 # the module counts as twisted when it stands this far off its mark or its motor holds with this share of its torque
MAX_TURN_DEG, MAX_OFF_MM = 0.3, 1.5                 # a hotel turned more than this against the arm, or a slot this far off the box: refused


class PlacesError(Exception):
    pass


def jaw_for(layout: dict, grip: str) -> float:
    return round(float(layout["gripper"]["plate_width"][grip]) + SLIDE_ON_MM, 2)


def module_reading(layout: dict, deg: int) -> float:
    a = layout["polar"]["ext_axis"]
    return float(a["zero_reading"]) + float(a["direction"]) * deg


def record(robot, layout: dict) -> dict:
    pose, joints = robot.pose(), robot.joints()
    return {"pose": [round(float(v), 4) for v in pose], "joints_at_teach": [round(float(v), 3) for v in joints[:4]],
            "jaw_at_teach": round(float(robot.jaw()), 3),
            "stage_axis_at_teach": round(float(robot.ext_axis(int(layout["polar"]["ext_axis"]["index"]))), 3),
            "taught_at": time.strftime("%Y-%m-%d %H:%M:%S")}


def plate_ahead(layout: dict, grip: str) -> float:
    """The plate's centre ahead of the wrist along the claw, for this grip (layout.json volumes.held)."""
    x = layout["volumes"]["held"][grip]["x"]
    return (float(x[0]) + float(x[1])) / 2.0


def refine_yaw(robot, layout: dict, grip: str, read: Callable[[str, float], dict], say: Callable[[str], None] = lambda s: None) -> dict:
    """The arm stands locked at the hand-taught module seat, the plate lies in the module's clips. At each trial angle: jaw open,
    module exactly on its mark, the wrist turned about the plate, force close, read the module. Returns the seat (the wrist pose at
    the angle where the module is not twisted, found by measuring how far each close pushed it) and the rows. Leaves the jaw
    open at that seat."""
    g, polar = layout["gripper"], layout["fixtures"]["polar"]
    deg = 0 if grip == "short_edges" else 90
    node = polar["pose_at_0" if deg == 0 else "pose_at_90"]
    seat, ahead = list(node["pose"]), plate_ahead(layout, grip)
    index, mark = int(layout["polar"]["ext_axis"]["index"]), module_reading(layout, deg)
    open_to = (polar.get("open_width") or {}).get(grip) or g["open"][grip]
    cx, cy = seat[0] + ahead * math.cos(math.radians(seat[5])), seat[1] + ahead * math.sin(math.radians(seat[5]))

    def at(yaw: float) -> List[float]:
        a = math.radians(yaw)
        return [round(cx - ahead * math.cos(a), 3), round(cy - ahead * math.sin(a), 3), seat[2], 0.0, 0.0, round(yaw, 3)]

    def trial(yaw: float) -> dict:
        robot.set_speed(0.1)
        robot.jaw_to(open_to)
        robot.ext_axis_to(index, mark)
        robot.move_linear(at(yaw))
        r = robot.clamp(float(g["closed"][grip]), float(g.get("clamp_limit", 0.5)), int(g.get("clamp_speed", 20)))
        pushed = robot.ext_axis(index) - mark
        off = abs(pushed)
        t = read(layout["robot"]["ip"], 1.2)
        row = {"yaw": round(yaw, 3), "width": r["width"], "module_torque": t["module"][0], "module_off_deg": round(off, 3), "pushed": pushed,
               "free": off <= PUSHED_DEG and abs(t["module"][0]) < FREE_TORQUE}
        say(f"yaw {row['yaw']:.2f}: module torque {row['module_torque']:+.2f}, off its mark {row['module_off_deg']:.2f} deg" + ("" if row["free"] else "  (twisted)"))
        return row

    # Measure, then correct. A gripper that closes at the wrong angle turns the module by about that angle (2026-11-11, long-edge
    # seat: 2.54 deg off its mark at a hand angle 2.4 deg wrong, 1.36 at 1.4, 0.41 at 0.4). So: close at the hand's angle; if the module
    # was pushed, turn the wrist by how far, the way that makes it less, and close again, until the module is left alone. Then one
    # step further the same way if that is free too, so the seat is not on the edge (on the short-edge seat the module is twisted on
    # one side only, so there is no middle to take).
    def better(a: dict, than: dict) -> bool:
        return a["free"] or a["module_off_deg"] < than["module_off_deg"] - 0.02 or (
            abs(a["module_off_deg"] - than["module_off_deg"]) <= 0.02 and abs(a["module_torque"]) < abs(than["module_torque"]) - 0.1)

    def size(row: dict) -> float:
        return max(row["module_off_deg"], STEP_DEG)

    rows = [trial(seat[5])]
    cur, way = rows[0], 0
    if not cur["free"]:
        for sgn in (-1, 1):                              # which way makes it less: the module's reading does not say on its own
            rows.append(trial(seat[5] + sgn * size(rows[0])))
            if better(rows[-1], rows[0]):
                cur, way = rows[-1], sgn
                break
        if not way:
            robot.jaw_to(open_to)
            robot.ext_axis_to(index, mark)
            raise PlacesError("turning the gripper either way did not take the twist off the module: the plate is not lying flat in the "
                              "module's clips, or the gripper is not on the plate's edges. Seat the plate and fit the gripper again")
        while not cur["free"] and len(rows) < MAX_CLOSES and abs(cur["yaw"] - seat[5]) < REACH_DEG:
            nxt = trial(cur["yaw"] + way * size(cur))
            rows.append(nxt)
            if not better(nxt, cur):                     # went past it: back by half
                way, nxt = -way, trial(cur["yaw"] + way * size(cur) / 2.0)
                rows.append(nxt)
            cur = nxt
    if cur["free"] and way:
        rows.append(trial(cur["yaw"] + way * STEP_DEG))
        if rows[-1]["free"]:
            cur = rows[-1]
    robot.jaw_to(open_to)
    robot.ext_axis_to(index, mark)
    if not cur["free"]:
        raise PlacesError(f"after {len(rows)} closes the module is still twisted (last at {cur['yaw']:.2f} deg, {cur['module_off_deg']:.2f} deg off its "
                          "mark). Seat the plate flat in the module's clips and fit the gripper again")
    free = [r["yaw"] for r in rows if r["free"]]
    robot.move_linear(at(cur["yaw"]))
    return {"pose": at(cur["yaw"]), "free_from": min(free), "free_to": max(free), "taught_yaw": seat[5], "rows": rows}


def _seen(layout: dict, slot: str) -> np.ndarray:
    """Where the plate was when the slot was taught, as the wrist pose of a claw square to the hotel (the hand leaves a few
    degrees of yaw; the record is turned to yaw 0 about the plate, as utils/fit_hotel_grid.py does)."""
    h = layout["fixtures"]["hotel"]
    p = h["taught_detail"][slot]["pose"]
    yaw, pivot = math.radians(p[5]), float(h["grid"]["pivot_mm"])
    return np.array([p[0] + pivot * (math.cos(yaw) - 1.0), p[1] + pivot * math.sin(yaw), p[2]])


def _shape(layout: dict, slot: str) -> np.ndarray:
    """The slot's seat inside the hotel, from B1: the hotel's own shape, the same on every workcell."""
    g = layout["fixtures"]["hotel"]["grid"]
    return np.array([float(g["x_column_mm"][slot[0]]), float(g["column_pitch"]) * COLUMN[slot[0]], float(g["row_pitch"]) * (int(slot[1:]) - 1)])


def _design(u: np.ndarray) -> np.ndarray:
    # unknowns: B1's seat x y z, the hotel's turn about the vertical, its lean in x and in y per mm of height
    return np.array([[1, 0, 0, -u[1], u[2], 0], [0, 1, 0, u[0], 0, u[2]], [0, 0, 1, 0, 0, 0]], dtype=float)


def fit_hotel(layout: dict, slots: List[str] = SLOTS) -> Dict[str, object]:
    """Every seat of fixtures.hotel.taught from the taught records of `slots`. Changes `layout`; the caller saves it."""
    h = layout["fixtures"]["hotel"]
    missing = [s for s in slots if s not in h.get("taught_detail", {})]
    if missing:
        raise PlacesError(f"not taught yet: {', '.join(missing)}")
    A = np.vstack([_design(_shape(layout, s)) for s in slots])
    b = np.concatenate([_seen(layout, s) - _shape(layout, s) for s in slots])
    p = np.linalg.lstsq(A, b, rcond=None)[0]
    off = {s: float(np.abs(_design(_shape(layout, s)) @ p + _shape(layout, s) - _seen(layout, s)).max()) for s in slots}
    turn = math.degrees(p[3])
    worst = max(off, key=off.get)
    if off[worst] > MAX_OFF_MM:
        raise PlacesError(f"slot {worst} was taught {off[worst]:.1f} mm away from where the other five put it: teach {worst} again "
                          "(the plate fully in its slot, the pads in the grooves)")
    if abs(turn) > MAX_TURN_DEG:
        raise PlacesError(f"the hotel stands turned {turn:+.2f} deg against the arm; the arm slides plates in straight and has "
                          f"{MAX_TURN_DEG} deg to spare. Loosen the hotel's screws, square it to the platform, and teach the six slots again")
    for s in h["taught"]:
        v = _design(_shape(layout, s)) @ p + _shape(layout, s)
        h["taught"][s] = [round(float(v[0]), 3), round(float(v[1]), 3), round(float(v[2]), 3), 0.0, -0.0, 0.0]
    h["grid"].update({"x": round(float(p[0]), 3), "y_B": round(float(p[1]), 3), "z_row1": round(float(p[2]), 3), "x_lean_per_mm": round(float(p[4]), 5),
                      "y_lean_per_mm": round(float(p[5]), 5), "turn_deg": round(turn, 3), "fitted_from": list(slots),
                      "fitted_at": time.strftime("%Y-%m-%d %H:%M:%S")})
    h["pose_source"] = "grid"
    return {"B1": [round(float(v), 2) for v in p[:3]], "turn_deg": round(turn, 2), "lean_x_mm_per_100": round(float(p[4]) * 100, 2),
            "lean_y_mm_per_100": round(float(p[5]) * 100, 2), "worst_slot_off_mm": round(off[worst], 2)}
