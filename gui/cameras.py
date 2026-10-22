"""
Cameras: what the vendor's calibration tool does, from this page, once, when the arm is new or a camera moved.
Read the marker with both cameras; capture the dotted plate (the arm drives a plan around it) and solve each
camera; capture again with the plate still for where each camera sits on the hand; upload to the box, let it
restart, check; then the box's own teach-free link (grip the practice plate: Add; open and hover: Add; Link),
short edges first, then long.

Results live in the positions file under vision.calibration; the files under <ref_dir>/calibration/<camera>/.
The `calibration` package does the work; until it lands (and always on --sim) gui/calibration_fake.py stands in.
"""
from __future__ import annotations

import importlib
import importlib.util
import os
import time
from typing import List

from procedures.safety import Safety

from gui import calibration_fake

LIMIT_PX, LIMIT_MM = 0.5, 1.0            # the vendor's: redo above these
PLAN_MM, PLAN_UP_MM = 20.0, 30.0         # the capture plan: a 3x3 grid around the plate, then four corners higher up
GRIPS = ("short_edges", "long_edges")    # the teach-free link is made once per grip, in this order
PICK = {"short_edges": "long", "long_edges": "wide"}   # the box's words for where the fingers hold


def package(cell):
    if cell.sim:
        return calibration_fake
    try:
        return importlib.import_module("calibration")
    except ImportError:
        return calibration_fake


def _cams(L: dict) -> List[tuple]:
    return [("down", L["vision"]["down_camera"]), ("front", L["vision"]["front_camera"])]


def _node(L: dict, camera: str) -> dict:
    return L["vision"].setdefault("calibration", {}).setdefault(camera, {})


def _dir(cell, camera: str, what: str = "") -> str:
    d = os.path.join(cell.ref_dir, "calibration", camera, what)
    os.makedirs(d, exist_ok=True)
    return d


def _stamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def plan_around(L: dict, here: List[float]) -> List[List[float]]:
    """Poses around where the arm is, each checked against the safe box; ends where it started."""
    s = Safety(L, "supervised")
    out = []
    for dz, step in ((0.0, PLAN_MM), (PLAN_UP_MM, PLAN_MM)):
        for dx in (-step, 0.0, step):
            for dy in (-step, 0.0, step):
                if dz and 0.0 in (dx, dy):
                    continue
                p = [here[0] + dx, here[1] + dy, here[2] + dz] + list(here[3:])
                try:
                    s.check_target(p, "capture plan")
                    out.append(p)
                except Exception:
                    pass
    return out + [list(here)]


def state(cell) -> dict:
    L = cell.layout()
    cams = {label: _node(L, cam) for label, cam in _cams(L)}
    tf = L["vision"].get("teach_free", {})
    grip = next((g for g in GRIPS if not tf.get(g, {}).get("linked_at")), None)
    return {"cameras": cams, "limits": {"px": LIMIT_PX, "mm": LIMIT_MM}, "fake": package(cell) is calibration_fake,
            "installed": importlib.util.find_spec("calibration") is not None,     # the flow shows only with the real package
            "calibrated_on": L["vision"].get("calibrated_on"),
            "marker_ok": all(n.get("marker") == "ok" for n in cams.values()),
            "teach_free": {"grip": grip, "current": tf.get(grip, {}) if grip else {}, "done": [g for g in GRIPS if tf.get(g, {}).get("linked_at")]}}


# ------------------------------------------------------------------------------------------------- steps
def read_marker(cell) -> None:
    """Both cameras read the marker on the practice plate under the hand, or the row says what to do."""
    L = cell.layout()
    said = []
    for label, cam in _cams(L):
        try:
            seen = any(cell.vision.aruco_pose(cam, 1, None, L["vision"]["led"]).pixel)
        except Exception:
            seen = False
        cell.edit(lambda L, cam=cam, seen=seen: _node(L, cam).update(marker="ok" if seen else "not read", marker_at=_stamp()))
        said.append(f"{label} camera: {'reads it' if seen else 'cannot read the marker: clean the lens, move the plate closer'}")
    cell.note("; ".join(said))


def capture_plate(cell, by_hand: bool) -> None:
    """Pictures of the dotted plate from around it, each camera solved. Redo above LIMIT_PX."""
    calib, L = package(cell), cell.layout()
    here = cell.robot.pose()
    for label, cam in _cams(L):
        if cell.stop_event.is_set():
            return
        folder = _dir(cell, cam, "plate")
        n = calib.capture.run_plan(None if by_hand else plan_around(L, here), cam, cell.robot, cell.vision, folder)
        cp = calib.intrinsics.solve(folder)
        cp.write(os.path.join(_dir(cell, cam), "camparam.dat"))
        px = round(float(cp.reprojection_px), 3)
        cell.edit(lambda L, cam=cam: _node(L, cam).update(plate_pictures=n, plate_px=px, plate_at=_stamp()))
        cell.note(f"{label} camera: {n} pictures, error {px} px: {'ok' if px <= LIMIT_PX else f'redo, above {LIMIT_PX} px'}")


def capture_hand(cell, by_hand: bool) -> None:
    """Plate still, arm moving: where each camera sits on the hand. Redo above LIMIT_MM."""
    calib, L = package(cell), cell.layout()
    here = cell.robot.pose()
    for label, cam in _cams(L):
        if cell.stop_event.is_set():
            return
        if not os.path.exists(os.path.join(_dir(cell, cam), "camparam.dat")):
            cell.note(f"{label} camera: capture the plate first")
            return
        folder = _dir(cell, cam, "hand")
        n = calib.capture.run_plan(None if by_hand else plan_around(L, here), cam, cell.robot, cell.vision, folder)
        ct = calib.handeye.solve(folder, calib.intrinsics.solve(_dir(cell, cam, "plate")))
        ct.write(os.path.join(_dir(cell, cam), "camintool.dat"))
        mm = round(float(ct.mean_error_m) * 1000.0, 3)
        cell.edit(lambda L, cam=cam: _node(L, cam).update(hand_pictures=n, hand_mm=mm, hand_at=_stamp()))
        cell.note(f"{label} camera: {n} pictures, error {mm} mm: {'ok' if mm <= LIMIT_MM else f'redo, above {LIMIT_MM:.0f} mm'}")


def upload(cell) -> None:
    """Both cameras' files to the box, then the box restarts (about a minute)."""
    calib, L = package(cell), cell.layout()
    for label, cam in _cams(L):
        d = _dir(cell, cam)
        if not all(os.path.exists(os.path.join(d, f)) for f in ("camparam.dat", "camintool.dat")):
            cell.note(f"{label} camera: capture the plate and the hand first")
            return
        calib.upload.push(L["vision"]["host"], cam, os.path.join(d, "camparam.dat"), os.path.join(d, "camintool.dat"))
        cell.edit(lambda L, cam=cam: _node(L, cam).update(uploaded_at=_stamp(), checked_mm=None))
    calib.upload.reboot(L["vision"]["host"])
    cell.note("uploaded; the box is restarting")
    deadline = time.time() + 180
    while time.time() < deadline and not cell.stop_event.is_set():
        try:
            cell.vision.status()
            cell.note("the box is back")
            return
        except Exception:
            time.sleep(3)
    cell.note("the box did not come back; check its power and cable")


def check(cell) -> None:
    """After the upload: how far the box's new numbers put the marker from where the arm says it is."""
    calib, L = package(cell), cell.layout()
    for label, cam in _cams(L):
        if not _node(L, cam).get("uploaded_at"):
            cell.note(f"{label} camera: upload first")
            return
        mm = round(float(calib.verify.check(cell.vision, cam)), 3)
        cell.edit(lambda L, cam=cam: _node(L, cam).update(checked_mm=mm, checked_at=_stamp()))
        cell.note(f"{label} camera: {mm} mm: {'ok' if mm <= LIMIT_MM else f'too far, above {LIMIT_MM:.0f} mm'}")


def teach_free(cell, step: str) -> None:
    """The box's own link for the current grip: 'teach' = fingers on the plate, put down flat; 'trigger' = open,
    hovering above it; 'link' ties the two on the box. Short edges first, then long."""
    L = cell.layout()
    grip = state(cell)["teach_free"]["grip"]
    if grip is None:
        cell.note("both grips are linked")
        return
    pose = [round(v, 4) for v in cell.robot.pose()]
    node = L["vision"].setdefault("teach_free", {}).setdefault(grip, {})
    if step in ("teach", "trigger"):
        node[f"{step}_pose"] = pose
        cell.save(L)
        cell.note(f"{grip.replace('_', ' ')}: {'grip' if step == 'teach' else 'hover'} point recorded")
        return
    if not node.get("teach_pose") or not node.get("trigger_pose"):
        cell.note("record the grip point and the hover point first")
        return
    package(cell).teach_free.add(cell.vision, PICK[grip], node["teach_pose"], node["trigger_pose"])
    node["linked_at"] = _stamp()
    cell.save(L)
    nxt = state(cell)["teach_free"]["grip"]
    cell.note(f"{grip.replace('_', ' ')} linked" + (f"; now the {nxt.replace('_', ' ')}" if nxt else "; both grips linked"))
