"""
`python cli.py teach [NAME]`: teach an instrument's seat by hand and plan its routes to the rotation module.

  1. the arm goes loose;
  2. the user moves it by hand until the gripper grabs the plate (in the instrument's plate holder) by its short edges;
  3. Enter. The seat goes into layout.json fixtures.<NAME>, the routes seat <-> module into preset/routes.json. READY.

Nothing here is new motion code: the arm is never commanded to move. The pieces are the tested ones: the loose arm with the jaw left
as it is (setup/hand.py loosen, the setup's "microscope seat" step), the record (record below, without the camera
frames), the seat's hover and planner box (setup/sections.py stage_hover, microscope_box), the planning (tools/plan_preset_routes.py).
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Callable, List

from run import positions
from run.motion import offset
from planner.safety import Safety
from planner import plan as P
from planner.volumes import Volumes
from planner import fk

ROUTES = os.path.join(positions.ROOT, "preset", "routes.json")
MODULE = "polar.pose_at_0"          # the rotation module's short-edge seat: the grip the instrument's seat is taught with
WRIST_WOUND_DEG = 180.0             # setup/sections.py: a seat dragged with joint 4 beyond this hit the soft limit on the first move to it


SCOPE_LONG_MM, SCOPE_SHORT_MM = 330.0, 220.0      # the microscope as a box (operator, 2026-11-03): long edges along the plate's long edges


def microscope_box(L: dict, seat: List[float]) -> dict:
    """The microscope for the planner, from the seat alone: a box whose top is the stage, centred under the plate. The plate is gripped on
    its short edges there, so the plate's long edges (and the box's 33 cm) run across the claw, the 22 cm along it. The plate's centre is
    where the short-edge grip holds it (volumes.held.short_edges). The box top is set under the fingertips at the seat, so the grip itself
    is not inside it: the stage's own holder above that is where the claw must go anyway."""
    v = L["volumes"]
    hx = v["held"]["short_edges"]["x"]
    ahead = (hx[0] + hx[1]) / 2.0
    yaw = math.radians(seat[5])
    ux, uy = math.cos(yaw), math.sin(yaw)                  # along the claw
    vx, vy = -uy, ux                                       # across it
    cx, cy = seat[0] + ahead * ux, seat[1] + ahead * uy
    a, b = SCOPE_SHORT_MM / 2.0, SCOPE_LONG_MM / 2.0
    xs = [cx + i * a * ux + j * b * vx for i in (-1, 1) for j in (-1, 1)]
    ys = [cy + i * a * uy + j * b * vy for i in (-1, 1) for j in (-1, 1)]
    top = seat[2] + float(v["claw"]["z"][0]) - float(v.get("margin_mm", 10.0)) - 5.0
    return {"x": [round(min(xs), 1), round(max(xs), 1)], "y": [round(min(ys), 1), round(max(ys), 1)], "z": [-50.0, round(top, 1)], "links": False,
            "_note": f"from the seat taught {time.strftime("%Y-%m-%d %H:%M:%S")}: {SCOPE_LONG_MM:.0f} x {SCOPE_SHORT_MM:.0f} mm box centred under the plate, top {5.0 + float(v.get('margin_mm', 10.0)):.0f} mm "
                     "under the fingertips at the seat (the stage holder above it is where the claw grips); the bounding box of the rotated rectangle"}


def plan_instrument_routes(layout: dict, name: str) -> dict:
    """The four preset routes between the module's short-edge hover and the instrument's hover: "module-><name>|short_edges",
    "<name>->module|short_edges" and the "|empty" pair, with the fields tools/plan_preset_routes.py writes for the hotel. The elbow
    side is the one the seat was taught on; the way back starts with the joints the way there ends with. A route that does not plan
    is an entry with "error". No hardware."""
    F = layout["fixtures"]
    seat = F[name]
    start = offset(list(F["polar"]["pose_at_0"]["pose"]), [0, 0, F["polar"]["hover_mm"]])
    hover = offset(list(seat["pose"]), [0, 0, seat["hover_mm"]])
    branch = fk.elbow_sign(seat["joints_at_teach"])
    j0 = fk.ik(start, elbow=branch)
    out = {}
    for holding in ("short_edges", None):
        tag = holding or "empty"
        common = {"branch": branch, "entry": hover, "approach": list(seat["pose"]), "start": start}
        r = None
        try:
            r = P.plan(layout, Safety(layout, "supervised", lambda: list(j0)), list(start), list(hover), MODULE, name, holding=holding)
            out[f"module->{name}|{tag}"] = dict(common, start_joints=j0, poses=r.poses, joint_targets={str(k): v for k, v in r.joint_targets.items()})
        except Exception as e:
            out[f"module->{name}|{tag}"] = dict(common, error=f"{type(e).__name__}: {e}")
        try:
            if r is None:
                raise ValueError("the route to this seat was not planned, so the joints it ends with are unknown")
            at = P.end_joints(r, list(start), j0, Volumes(layout, Safety(layout, "supervised", lambda: list(j0)), holding).lim)
            back = P.plan(layout, Safety(layout, "supervised", lambda at=at: list(at)), list(hover), list(start), name, MODULE, holding=holding)
            out[f"{name}->module|{tag}"] = dict(common, start_joints=at, poses=back.poses, joint_targets={str(k): v for k, v in back.joint_targets.items()})
        except Exception as e:
            out[f"{name}->module|{tag}"] = dict(common, error=f"{type(e).__name__}: {e}")
    return out


def store(entries: dict) -> None:
    """Merge route entries into preset/routes.json (the hotel routes there are kept)."""
    data = {"routes": {}}
    if os.path.exists(ROUTES):
        with open(ROUTES, encoding="utf-8") as f:
            data = json.load(f)
    data.setdefault("routes", {}).update(entries)
    os.makedirs(os.path.dirname(ROUTES), exist_ok=True)
    with open(ROUTES, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1)


def record(robot, layout: dict, name: str) -> dict:
    """Where the arm is now becomes fixtures.<name>; its other keys stay.
    As the setup's seat step does (setup/sections.py stage_hover), the stored hover and the instrument's box for the planner follow."""
    pose, joints, jaw, e2 = robot.pose(), robot.joints(), robot.jaw(), robot.ext_axis(int(layout["polar"]["ext_axis"]["index"]))
    if abs(joints[3]) > WRIST_WOUND_DEG:
        raise ValueError(f"the wrist is wound (joint 4 at {joints[3]:.0f} deg): lift the claw off the plate, turn it one full turn the other "
                         "way, set it back around the plate and run again")
    node = layout["fixtures"].setdefault(name, {})
    node.update({"pose": [round(v, 4) for v in pose], "joints_at_teach": [round(v, 3) for v in joints[:4]],
                 "jaw_at_teach": round(jaw, 3), "stage_axis_at_teach": round(e2, 3), "taught_at": time.strftime("%Y-%m-%d %H:%M:%S")})
    node.pop("_todo", None)
    if "hover_mm" not in node:
        raise ValueError(f"fixtures.{name}.hover_mm is not set in layout.json: the height the arm stands above this seat before lowering")
    node["hover"] = {"pose": offset(node["pose"], [0, 0, float(node["hover_mm"])]), "derived_from_seat": node["taught_at"]}
    layout.setdefault("volumes", {}).setdefault("obstacles", {})[name] = microscope_box(layout, node["pose"])
    return node


def run(name: str, make_robot: Callable[[dict], object], ask: Callable[[str], str] = input) -> int:
    layout = positions.load()
    robot = make_robot(layout)
    try:
        robot.connect()
        st = robot.state()
    except Exception as e:
        print(f"the arm is not reachable ({e}): close BioyondWorks and any other program that holds it, then run again")
        return 2
    try:
        if st.moving:
            print("the arm is moving: wait until it stands still, then run again")
            return 2
        robot.drag(True)                                  # the jaw is left as it is (setup/hand.py loosen): it must stand open around the plate
        print(f"the arm is loose (jaw at {robot.jaw():.1f}): move it by hand until the gripper grabs the plate by its short edges")
        ask("press Enter when the gripper holds the plate: ")
        layout = positions.load()
        try:
            node = record(robot, layout, name)
        except ValueError as e:
            print(f"{e}. The arm is still loose")
            return 2
        positions.save(layout)
        print(f"seat fixtures.{name} = {[round(v, 1) for v in node['pose'][:3]]} yaw {node['pose'][5]:.2f}, jaw {node['jaw_at_teach']:.2f}, stored in layout.json")
        robot.drag(False)
        if not robot.enable():
            print("the seat is stored, but the arm could not be enabled again: python cli.py recover")
            return 2
        print("the arm is locked")
    finally:
        robot.disconnect()
    entries = plan_instrument_routes(layout, name)
    bad: List[str] = [f"{k}: {v['error'].splitlines()[0]}" for k, v in entries.items() if "error" in v]
    if bad:
        print(f"the seat is stored, but {len(bad)} of {len(entries)} routes to the rotation module did not plan ({'; '.join(bad)})")
        return 2
    store(entries)
    print(f"{len(entries)} routes {name} <-> rotation module planned")
    print("routes and seat stored in preset/; upload to the arm is not built yet")
    print("READY")
    return 0
