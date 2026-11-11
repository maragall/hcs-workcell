"""The down camera's viewing pose over a station, taught with the operator's hand.

  python utils/view_pose.py go polar       from the module's hover to the candidate view pose (the camera, 115 mm ahead of the wrist, over
                                           the seat centre, 186.5 mm ahead of the taught pose; 30 mm above the hover), then the arm goes
                                           loose: adjust it by hand while watching the gripper camera in the window
  python utils/view_pose.py save polar     read where the arm is, lock it, store fixtures.polar.view (pose, and the offset from the
                                           station's taught pose in the claw's frame: forward, left, up), keep a picture

The offset in the claw's frame is what carries over: `deduce microscope` writes the microscope's view pose as its taught seat plus the
module's offset, for the operator to confirm the same way.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
os.chdir(HERE)
from utils.devices import nova  # noqa: E402
from run import positions  # noqa: E402
from run.cycle import Cycle  # noqa: E402
from run.motion import offset  # noqa: E402
from run.planner.safety import SpeedProfile  # noqa: E402
from run.state import StateStore  # noqa: E402
from run.telemetry import Telemetry  # noqa: E402
from vision.view import from_tool_frame, to_view, tool_frame  # noqa: E402,F401

CAMERA_AHEAD_MM = 115.0          # the down camera looks this far ahead of the wrist (hardware.json claw.camera_ahead_of_wrist_mm)
SEAT_AHEAD_MM = 186.5            # the plate's centre ahead of the wrist at a short-edge seat (hardware.json claw.module_wrist_to_plate_mm)
ABOVE_HOVER_MM = 30.0

STATION = {"polar": ("polar.pose_at_0", lambda F: F["polar"]["pose_at_0"]["pose"], lambda F: F["polar"]),
           "microscope": ("microscope", lambda F: F["microscope"]["pose"], lambda F: F["microscope"])}


def cell():
    layout = positions.load()
    runs = os.path.join(positions.RUNS, "view_pose")
    tele = Telemetry(runs)
    robot, st = nova.connect_readonly(layout, tele.log)
    return layout, runs, tele, robot, st


def go(station):
    layout, runs, tele, robot, st = cell()
    if st.drag:
        robot.drag(False)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    store = StateStore(os.path.join(positions.ROOT, "state_microscope.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, positions.REFS)
    cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
    cyc.corr.ask = lambda q, t=0: "y"
    if store.s.grip != "none":
        sys.exit(f"the ledger says the claw holds a plate ({store.s.grip}): not with a plate in the claw")
    name, seat_of, fixture_of = STATION[station]
    F = layout["fixtures"]
    p0, fx = seat_of(F), fixture_of(F)
    hover = offset(p0, [0, 0, fx["hover_mm"]])
    view = fx.get("view", {}).get("tool_offset")
    fwd, left, up = view if view else (SEAT_AHEAD_MM - CAMERA_AHEAD_MM, 0.0, fx["hover_mm"] + ABOVE_HOVER_MM)
    zmax = float(layout["workspace_box"]["z"][1]) - 3.0 if isinstance(layout.get("workspace_box", {}).get("z"), list) else None
    if zmax is not None and p0[2] + up > zmax:                                 # the module's height carried to a higher seat can pass the arm's reach
        print(f"the view would be at z {p0[2] + up:.0f}, above the workspace ({zmax:.0f}): using {zmax - p0[2]:.0f} mm above the seat instead", flush=True)
        up = zmax - p0[2]
    target = from_tool_frame(p0, fwd, left, up)
    try:
        cyc.move_to(name)
        print(f"view: forward {fwd:.1f}, left {left:.1f}, up {up:.1f} mm from the seat pose", flush=True)
        to_view(cyc, station, target)
        print("at", [round(v, 1) for v in robot.pose()[:3]], "yaw", round(robot.pose()[5], 1), flush=True)
        robot.drag(True)
        print("arm loose: adjust it by hand watching the gripper camera; then: python utils/view_pose.py save", station, flush=True)
    finally:
        robot.disconnect()
        tele.close()


def save(station):
    layout, runs, tele, robot, st = cell()
    name, seat_of, fixture_of = STATION[station]
    F = layout["fixtures"]
    p0 = seat_of(F)
    try:
        pose = robot.pose()
        if st.drag:
            robot.drag(False)
        robot.enable()
        off = tool_frame(p0, pose)
        L = positions.load()
        fixture_of(L["fixtures"])["view"] = {"pose": [round(v, 3) for v in pose], "tool_offset": off, "taught_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                                             "_note": "the down camera's viewing pose over this station: taught by hand (utils/view_pose.py); tool_offset = forward, left, up from the station's taught pose, in the claw's frame"}
        positions.save(L)
        print(f"saved fixtures.{station}.view: pose {[round(v, 1) for v in pose[:3]]} yaw {pose[5]:.1f}; offset from the seat pose: forward {off[0]}, left {off[1]}, up {off[2]} mm", flush=True)
        try:
            v = layout["vision"]
            jpg = nova.vision_box(layout, tele.log).frame(v["down_camera"], 1280, 960, 90, None, int(v.get("led", 0)))
            d = os.path.join(positions.ROOT, "vision", "reference_images", "view")
            os.makedirs(d, exist_ok=True)
            p = os.path.join(d, f"{station}_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
            open(p, "wb").write(jpg)
            print("picture", p, flush=True)
        except Exception as e:
            print("no picture:", e, flush=True)
    finally:
        robot.disconnect()
        tele.close()


def deduce(station):
    """The station's view pose from the module's offset in the claw's frame, written as a candidate for `go` to take the arm to."""
    layout = positions.load()
    F = layout["fixtures"]
    src = F["polar"].get("view", {}).get("tool_offset")
    if not src:
        sys.exit("fixtures.polar.view is not taught yet: python utils/view_pose.py go polar, then save polar")
    name, seat_of, fixture_of = STATION[station]
    L = positions.load()
    fixture_of(L["fixtures"])["view"] = {"tool_offset": list(src), "deduced_from": "polar", "taught_at": None,
                                         "_note": "candidate: the module's viewing offset applied to this station's seat; confirm with go/save"}
    positions.save(L)
    print(f"fixtures.{station}.view candidate: forward {src[0]}, left {src[1]}, up {src[2]} mm from the seat pose. Now: python utils/view_pose.py go {station}")


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in ("go", "save", "deduce") or sys.argv[2] not in STATION:
        sys.exit(__doc__)
    {"go": go, "save": save, "deduce": deduce}[sys.argv[1]](sys.argv[2])
