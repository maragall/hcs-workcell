"""
Cameras: what the vendor's calibration tool does, from this page, once, when the arm is new or a camera moved.
Read the marker with both cameras; capture (the arm drives the plan around the marker and one solve per camera
gives its lens numbers and where it sits on the hand); upload to the box, let it restart, check; then the box's
own teach-free link (grip the practice plate: Add; open and hover: Add; Link), short edges first, then long.

Results live in the positions file under vision.calibration; the files under <ref_dir>/calibration/<camera>/.
The `calibration` package does the work; on --sim gui/calibration_fake.py stands in.
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
CHECK_MOVE_MM = 20.0                     # the check reads the marker from here and from this far along x
GRIPS = ("short_edges", "long_edges")    # the teach-free link is made once per grip, in this order
PICK = {"short_edges": "long", "long_edges": "wide"}   # the box's words for where the fingers hold
VENDOR_START = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vendor_data",
                            "CalibrationData", "IniIntrinsic", "CamParam.dat")   # the solve's start when a camera has no file yet


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


def _files(cell, camera: str):
    d = _dir(cell, camera)
    return os.path.join(d, "CamParam.dat"), os.path.join(d, "CamInTool.dat")


def _stamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def plan_around(cell, L: dict, here: List[float]) -> List[List[float]]:
    """The package's plan around where the arm is, each pose checked against the safe box; ends where it started."""
    s = Safety(L, "supervised")
    return package(cell).plan.around(here, lambda p: s.check_target(p, "capture plan"))


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


def capture(cell, by_hand: bool) -> None:
    """Pictures of the marker from around it, one solve per camera: lens numbers and where the camera sits on the
    hand. Redo above LIMIT_PX or LIMIT_MM."""
    calib, L = package(cell), cell.layout()
    here = cell.robot.pose()
    for label, cam in _cams(L):
        if cell.stop_event.is_set():
            return
        folder = _dir(cell, cam, "capture")
        n = calib.capture.run_plan(None if by_hand else plan_around(cell, L, here), cam, cell.robot, cell.vision, folder,
                                   L["vision"]["led"], stop=cell.stop_event)
        camparam, _ = _files(cell, cam)
        start = camparam if os.path.exists(camparam) else VENDOR_START
        res = calib.solve.run(folder, calib.halcon.CamParam.read(start), L["vision"]["marker_mm"] / 1000.0)
        res.write(_dir(cell, cam))
        px, mm = round(float(res.reprojection_px), 3), round(float(res.mean_error_m) * 1000.0, 3)
        cell.edit(lambda L, cam=cam: _node(L, cam).update(pictures=res.images, px=px, mm=mm, captured_at=_stamp()))
        verdict = "ok" if px <= LIMIT_PX and mm <= LIMIT_MM else f"redo, above {LIMIT_PX} px or {LIMIT_MM:.0f} mm"
        cell.note(f"{label} camera: {res.images} pictures, error {px} px, {mm} mm: {verdict}")


def upload(cell) -> None:
    """Both cameras' files to the box, then the box restarts (about a minute)."""
    calib, L = package(cell), cell.layout()
    creds = calib.upload.credentials(L)
    for label, cam in _cams(L):
        camparam, camintool = _files(cell, cam)
        if not (os.path.exists(camparam) and os.path.exists(camintool)):
            cell.note(f"{label} camera: capture first")
            return
        calib.upload.push(L["vision"]["host"], cam, camparam, camintool, creds)
        cell.edit(lambda L, cam=cam: _node(L, cam).update(uploaded_at=_stamp(), checked_mm=None))
    calib.upload.reboot(L["vision"]["host"], creds)
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
    """After the upload: the box reads the marker from here and from CHECK_MOVE_MM further along x; with the new
    numbers it reports the same place both times."""
    calib, L = package(cell), cell.layout()
    here = cell.robot.pose()
    there = [here[0] + CHECK_MOVE_MM] + list(here[1:])
    for label, cam in _cams(L):
        if not _node(L, cam).get("uploaded_at"):
            cell.note(f"{label} camera: upload first")
            return
        mm = round(float(calib.verify.check(cell.vision, cam, cell.robot, here, there, 1, L["vision"]["led"])), 3)
        cell.edit(lambda L, cam=cam: _node(L, cam).update(checked_mm=mm, checked_at=_stamp()))
        cell.note(f"{label} camera: {mm} mm: {'ok' if mm <= LIMIT_MM else f'too far, above {LIMIT_MM:.0f} mm'}")
    cell.robot.move_linear(here)


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
    package(cell).teach_free.add(cell.vision, PICK[grip], node["teach_pose"], node["trigger_pose"],
                                 L["vision"]["down_camera"], grip.replace("_", "-"))
    node["linked_at"] = _stamp()
    cell.save(L)
    nxt = state(cell)["teach_free"]["grip"]
    cell.note(f"{grip.replace('_', ' ')} linked" + (f"; now the {nxt.replace('_', ' ')}" if nxt else "; both grips linked"))
