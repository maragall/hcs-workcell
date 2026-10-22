"""
Teach: one list, one Record per row. A Record stores the pose where the arm is (cli record-pose), the photo the
checks will compare against at that place, and the route dragged since the previous Record (cli trace +
normalise, sampled while the arm is loose). Where the vision box can find the plate under the hovering gripper
(start_vision_teach) the grip pose comes from it; otherwise the recorded pose is stored as it is.
"""
from __future__ import annotations

import glob
import os
import threading
import time
from typing import List, Optional

from perception.lower_check import LowerCheck
from procedures.preflight import preflight
from procedures.safety import Safety
from routes import trajectory

from gui import bench

SLOW = 0.05                      # travel speed fraction for Replay slowly
SAMPLE_S = 0.1
VENDOR_PICK = {"short_edges": "long", "long_edges": "wide"}      # the box's words for where the fingers hold


class Tracer:
    """Samples joints and pose while the arm is loose; each Record takes what was dragged since the last one."""

    def __init__(self, robot):
        self.robot, self.samples, self.on = robot, [], False
        self._stop = threading.Event()

    def start(self):
        self._stop.clear()
        self.samples, self.on = [], True
        threading.Thread(target=self._loop, daemon=True, name="tracer").start()

    def _loop(self):
        t0 = time.time()
        while not self._stop.is_set():
            self.samples.append((self.robot.joints(), self.robot.pose(), time.time() - t0))
            self._stop.wait(SAMPLE_S)

    def take(self) -> Optional[trajectory.Trace]:
        s, self.samples = self.samples, []
        return trajectory.Trace("", [j for j, _, _ in s], [p for _, p, _ in s], [t for _, _, t in s]) if len(s) > 2 else None

    def stop(self):
        self._stop.set()
        self.on = False


def F_taught(L: dict, scope: str) -> bool:
    return bool(L["fixtures"][scope].get("pose"))


def _teach(cell):
    """Per-cell teaching state, created on first use."""
    if not hasattr(cell, "teach"):
        cell.teach = {"tracer": Tracer(cell.robot), "last_family": None}
    return cell.teach


def _family(row_id: str) -> str:
    kind, name = row_id.split(":", 1)
    return "hotel" if kind == "slot" else name.split(".")[0].split("@")[0]


# ------------------------------------------------------------------------------------------------- the list
def rows(cell) -> dict:
    L, lc = cell.layout(), LowerCheck(cell.ref_dir)
    need = L["vision"]["lower_check"]["min_refs"]
    scopes = [s for s in bench.microscopes(L) if s == "microscope" or F_taught(L, s)]
    problems = preflight(L, cell.ref_dir, scopes)
    h, F = L["fixtures"]["hotel"], L["fixtures"]
    top = max(h["rows"])

    def pics(gate, state):
        n = lc.count(gate, state)
        return f"picture {n} of {need}" if n < need else "pictures"

    def row(rid, label, put, done, level, action="record", needs=()):
        bad = any(p == n or (n.endswith((":", "/")) and p.startswith(n)) for p in problems for n in needs)
        return {"id": rid, "label": label, "put": put, "done": done, "level": "bad" if bad else level, "action": action}

    out = []
    for s in scopes:
        fx, name = F[s], bench.label(s)
        pose_words = "pose from the camera" if fx.get("pose_from_camera") else "pose"
        out.append(row(f"hover:{s}", f"{name}, hover", "above the holder, a hand's width up, with a plate in the holder",
                       f"{pose_words}, {pics(s, 'plate_present')}" if fx.get("pose") else "nothing yet",
                       "ok" if fx.get("pose") else "bad", needs=(f"pose not taught: {s}", f"reference images {s}/plate_present:")))
        out.append(row(f"empty:{s}", f"{name}, hover, empty", "same place, take the plate out",
                       pics(s, "empty") if lc.count(s, "empty") else "nothing yet", "ok", needs=(f"reference images {s}/empty:",)))
        out.append(row(f"marker:{s}", f"{name}, marker", "screw the marker tab onto the holder, hover so the down camera sees it whole",
                       "marker seen" if fx.get("aruco_reference") else "no marker yet", "ok" if fx.get("aruco_reference") else "warn"))
    P = F["polar"]
    routes = [r for r in L["trajectories"] if r.endswith("_to_polar") and L["trajectories"][r]]
    route_words = ", ".join(f"route from {bench.label(r[:-len('_to_polar')]).lower()}" for r in routes)
    for deg, grip in ((0, "short edges"), (90, "long edges")):
        node = P[f"pose_at_{deg}"]
        put = "drag over to the rotation module, plate seated, short edges in the fingers" if deg == 0 else "turn the module, long edges in the fingers"
        done = ", ".join(x for x in ("pose", route_words if deg == 0 else "", pics(f"polar@{deg}", "plate_present")) if x)
        out.append(row(f"seat:polar.pose_at_{deg}", f"Rotation module at {deg}, seat", put, done if node.get("pose") else "nothing yet",
                       "ok" if node.get("pose") else "bad",
                       needs=(f"pose not taught: polar.pose_at_{deg}", f"reference images polar@{deg}/plate_present:") +
                       (("trajectory not taught:",) if deg == 0 else ())))
        out.append(row(f"empty:polar@{deg}", f"Rotation module at {deg}, empty", "same place, take the plate off the module",
                       pics(f"polar@{deg}", "empty") if lc.count(f"polar@{deg}", "empty") else "nothing yet", "ok",
                       needs=(f"reference images polar@{deg}/empty:",)))
    out.append(row("marker:polar", "Rotation module, marker", "marker tab on the module, hover so the down camera sees it whole",
                   "marker seen" if P.get("aruco_reference") else "no marker yet", "ok" if P.get("aruco_reference") else "warn"))
    for slot in (f"A1", f"A{top}", "C1", f"C{top}"):
        taught = slot in h["taught"]
        extra = ", route from the rotation module" if slot == "A1" and L["trajectories"].get("polar_to_hotel") else ""
        out.append(row(f"slot:{slot}", f"Hotel {slot}", f"drag to the hotel, plate seated in {slot}",
                       f"pose{extra}" if taught else "nothing yet", "ok" if taught else "bad",
                       needs=("trajectory not taught: polar_to_hotel",) if slot == "A1" else ()))
    g = L["gripper"]["closed"]
    words = [f"{k.split('_')[0]} {v:.2f} mm" if v else f"{k.split('_')[0]} not yet" for k, v in g.items()]
    out.append(row("grip", "Grip", "plate between the fingers, short edges, then long edges", ", ".join(words),
                   "ok" if all(g.values()) else "bad", action="measure", needs=("grip width not measured:", "reference images gripper/")))
    z = L.get("routes", {}).get("safe_z_mm")
    out.append(row("travel", "Travel height", "lift the arm to a height that clears everything",
                   f"{z:.0f} mm" if z else "nothing yet", "ok" if z else "bad"))
    t = _teach(cell)
    return {"rows": out, "dragging": t["tracer"].on, "problems": problems}


# -------------------------------------------------------------------------------------------------- actions
def drag(cell, on: bool) -> None:
    """Hand-guiding. On: the jaw closes (on the plate or on itself), the arm goes loose, the route recorder starts.
    Off: the recorder stops, the arm locks and is enabled again."""
    t = _teach(cell)
    if on:
        try:
            cell.robot.jaw_to(0.0)
        except Exception:
            pass                                   # the fingers met the plate: that is a closed jaw too
        cell.robot.drag(True)
        t["tracer"].start()
        cell.note("arm loose: move it by hand")
    else:
        t["tracer"].stop()
        cell.robot.drag(False)
        cell.note("arm locked")


def _snapshot(cell, L: dict, rid: str) -> dict:
    """Both cameras now, saved under reference_images/teach/<row>/ as the cli does. {"down": bytes, "front": bytes}."""
    d = os.path.join(cell.ref_dir, "teach", rid.replace(":", "_").replace(".", "_"))
    os.makedirs(d, exist_ok=True)
    stamp, out = time.strftime("%Y%m%d_%H%M%S"), {}
    for label in ("down", "front"):
        got = cell.frames.by_label(label)
        if got:
            out[label] = got[1]
            with open(os.path.join(d, f"{stamp}_{label}.jpg"), "wb") as f:
                f.write(got[1])
    return out


def _vision_teach(cell, L: dict, fixture: dict, pose: List[float]) -> Optional[List[float]]:
    """The grip pose from the marker on the practice plate under the hovering gripper, or None when the box cannot."""
    cam, led = L["vision"]["down_camera"], L["vision"]["led"]
    pick = VENDOR_PICK[fixture.get("grip", "short_edges")]
    try:
        r = cell.vision.start_vision_teach(pose, cam, False, pick, True, led)
        for _ in range(20):
            if r["finished"] or r["has_error"]:
                break
            time.sleep(0.5)
            r = cell.vision.fine_tune_vision_teach(pose, cam, False, True)
    except Exception as e:
        cell.note(f"camera could not find the plate: {e}")
        return None
    if r["has_error"] or not r["finished"] or not any(r["pose"][:3]):
        cell.note(f"camera could not find the plate: {r.get('error') or 'not finished'}")
        return None
    return [round(v, 4) for v in r["pose"]]


def _vendor_teach(cell, L: dict, fixture: dict, name: str, pose: List[float]) -> None:
    """The box's own record of this seat (cli record-pose): it is the system of record for its no-teach moves."""
    ws, pick = L["vision"]["workspace"], VENDOR_PICK[fixture.get("grip", "long_edges" if "90" in name else "short_edges")]
    vt = fixture.get("vendor_teach") or {"name": f"{name.replace('.pose_at_', '')}-{'short' if pick == 'long' else 'wide'}"}
    try:
        if vt["name"] in {p["name"] for p in cell.vision.positions(ws, "teach")}:
            cell.vision.delete_position(vt["name"], "teach", ws)
        cell.vision.add_teach(vt["name"], pose, pick, ws)
        stored = {p["name"]: p for p in cell.vision.positions(ws, "teach")}.get(vt["name"])
        fixture["vendor_teach"] = {"name": vt["name"], "pick_method": pick, "id": stored["id"] if stored else None, "workspace": ws}
    except Exception as e:
        cell.note(f"vision box did not store the point: {e}")


def _marker(cell, L: dict, fixture: dict, family: str, pose: List[float]) -> None:
    """The marker view now becomes the fixture's reference (set_reference_aruco_pose), plus the box's trigger and
    relation when it has a teach point for this fixture (cli record-trigger)."""
    v, cam = L["vision"], L["vision"]["down_camera"]
    ref = fixture.get("aruco_reference") or f"{family}_ref"
    mid = fixture.get("aruco_marker_id", 1)
    cell.vision.set_reference(cam, ref, pose, mid, None, v["led"])
    fixture.update(aruco_reference=ref, aruco_marker_id=mid, marker_taught_at=time.strftime("%Y-%m-%d %H:%M:%S"))
    vt = fixture.get("vendor_teach")
    if vt:
        trig, ws = f"{ref}-trigger", v["workspace"]
        if trig in {p["name"] for p in cell.vision.positions(ws, "trigger")}:
            cell.vision.delete_position(trig, "trigger", ws)
        cell.vision.add_trigger(trig, pose, cam, "aruco", 150, vt["pick_method"], ws, v["led"])
        cell.vision.add_relation(trig, vt["name"], f"{trig}-rel", vt["pick_method"], ws)
        fixture["vendor_triggers"] = [{"name": trig, "relation": f"{trig}-rel", "pose": [round(x, 4) for x in pose], "caltab_mm": 150}]


def _route(cell, L: dict, trace: Optional[trajectory.Trace], frm: Optional[str], to: str) -> Optional[str]:
    """Store the dragged path as the route between the two families, when they are a pair the cycle uses."""
    if trace is None or not frm or frm == to:
        return None
    known = {f"{m}_to_polar" for m in bench.microscopes(L)} | {"polar_to_hotel"}
    name, rev = f"{frm}_to_{to}", f"{to}_to_{frm}"
    if name not in known and rev in known:
        name, trace = rev, trajectory.Trace("", trace.joints[::-1], trace.poses[::-1], trace.t)
    if name not in known:
        return None
    trace.name = name
    os.makedirs(cell.runs_dir, exist_ok=True)
    n = len(glob.glob(os.path.join(cell.runs_dir, f"trace_{name}_*.json"))) + 1
    trace.save(os.path.join(cell.runs_dir, f"trace_{name}_{n:02d}.json"))
    traces = [trajectory.Trace.load(f) for f in sorted(glob.glob(os.path.join(cell.runs_dir, f"trace_{name}_*.json")))]
    try:
        wp = trajectory.normalise(traces, max_joint_step=L["limits"]["max_waypoint_joint_step_deg"])
        safety = Safety(L, "supervised")
        for i, p in enumerate(wp["poses"], 1):
            safety.check_target(p, f"{name} waypoint {i}")
    except Exception as e:
        cell.note(f"route not stored: {e}")
        return None
    L["trajectories"][name] = wp
    return name


def record(cell, rid: str) -> None:
    kind, name = rid.split(":", 1)
    t, L = _teach(cell), cell.layout()
    robot, F = cell.robot, L["fixtures"]
    pose = [round(v, 4) for v in robot.pose()]
    here = {"joints_at_teach": [round(v, 3) for v in robot.joints()[:4]], "jaw_at_teach": round(robot.jaw(), 3),
            "stage_axis_at_teach": round(robot.ext_axis(int(L["polar"]["ext_axis"]["index"])), 3),
            "taught_at": time.strftime("%Y-%m-%d %H:%M:%S")}
    shots = _snapshot(cell, L, rid)
    lc = LowerCheck(cell.ref_dir)
    family = _family(rid)
    route = _route(cell, L, t["tracer"].take() if t["tracer"].on else None, t["last_family"], family)
    t["last_family"] = family
    said = []
    if kind == "hover":
        fx = F[name]
        seat = _vision_teach(cell, L, fx, pose)
        fx.update(pose=seat or pose, pose_from_camera=bool(seat), **here)
        if seat:
            fx["hover_mm"] = round(pose[2] - seat[2], 1)
        fx.pop("_todo", None)
        _vendor_teach(cell, L, fx, name, fx["pose"])
        if "down" in shots:
            lc.add_reference(name, "plate_present", shots["down"])
        said.append("pose from the camera" if seat else "pose")
    elif kind == "empty":
        if "down" in shots:
            lc.add_reference(name, "empty", shots["down"])
    elif kind == "marker":
        _marker(cell, L, F[name], family, pose)
        said.append("marker seen")
    elif kind == "seat":
        node = F["polar"][name.split(".")[1]]
        node.update(pose=pose, **here)
        node.pop("_todo", None)
        _vendor_teach(cell, L, node, name, pose)
        if "down" in shots:
            lc.add_reference(f"polar@{name[-2:].lstrip('_')}", "plate_present", shots["down"])
        said.append("pose")
    elif kind == "slot":
        h = F["hotel"]
        h["taught"][name] = pose
        h.setdefault("taught_detail", {})[name] = dict(pose=pose, **here)
        top = max(h["rows"])
        if name == f"A{top}" and "A1" in h["taught"]:
            h["row_vector_from"] = ["A1", name, top - 1]
        said.append("pose")
    elif kind == "travel":
        L.setdefault("routes", {})["safe_z_mm"] = pose[2]
        said.append(f"{pose[2]:.0f} mm")
    if shots:
        said.append("picture")
    if route:
        said.append(f"route {route.replace('_', ' ')}")
    cell.save(L)
    cell.note(f"recorded {rid.split(':', 1)[1].replace('.', ' ')}: {', '.join(said)}")


def measure(cell, grip: str) -> None:
    """Close in steps on the plate between the fingers until the jaw stops following; store the width and the front
    camera's picture of the held plate (cli measure-grip + the gripper reference)."""
    L = cell.layout()
    g, robot = L["gripper"], cell.robot
    v, contact = robot.jaw(), None
    while v > 0.2:
        v = round(v - 0.25, 3)
        try:
            robot.jaw_to(v)
        except Exception:
            contact = robot.jaw()
            break
        time.sleep(0.3)
        read = robot.jaw()
        if read - v > g["stall_lag"]:
            contact = read
            break
    if contact is None:
        robot.jaw_to(g["open"].get(grip) or 10.0)
        cell.note("the fingers closed fully without meeting a plate; nothing measured")
        return
    width = round(contact, 2)
    g["closed"][grip] = width
    if not g["open"].get(grip):
        g["open"][grip] = round(width + 2.0, 2)
    if all(g["closed"].values()):
        g.pop("_todo", None)
    cell.save(L)
    got = cell.frames.by_label("front")
    if got:
        LowerCheck(cell.ref_dir).add_reference("gripper", grip, got[1])
    cell.note(f"{grip.replace('_', ' ')}: {width:.2f} mm")


def replay(cell) -> None:
    """Every taught route forward and back at low speed, each from where it starts; stops at the first gap."""
    L = cell.layout()
    cyc = cell.cycle(tag="replay")
    cyc.safety.profile.travel = SLOW
    for name, wp in L["trajectories"].items():
        if not wp or name.endswith(("_demo_full", "_fast")):
            continue
        here, first = cell.robot.pose(), wp["poses"][0]
        gap = max(abs(here[i] - first[i]) for i in range(3))
        if gap > 15.0:
            cell.note(f"{name.replace('_', ' ')}: the arm is {gap:.0f} mm from its start; move it there first")
            return
        cell.note(f"replaying {name.replace('_', ' ')}")
        cyc._waypoints(name)
        if cell.stop_event.is_set():
            return
        cyc._waypoints(name, reverse=True)
        if cell.stop_event.is_set():
            return
    cell.note("replay done")
