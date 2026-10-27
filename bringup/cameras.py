"""
Cameras: what the vendor's calibration tool does, from the setup program, once, when the arm is new or a camera moved.
Read the marker, take the pictures (the arm drives the plan around the marker), solve (lens numbers and where the camera
sits on the hand), upload to the box and let it restart, check from two poses.

Results live in the positions file under vision.calibration; the files under <ref_dir>/calibration/<camera>/.
The `calibration` package does the work; on --sim bringup/calibration_fake.py stands in.
"""
from __future__ import annotations

import importlib
import importlib.util
import os
import time
from typing import List

from procedures.safety import Safety

from bringup import calibration_fake

LIMIT_PX, LIMIT_MM = 0.5, 1.0            # the vendor's: redo above these
CHECK_MOVE_MM = 20.0                     # the check reads the marker from here and from this far along x
VENDOR_START = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vendor_data",
                            "CalibrationData", "IniIntrinsic", "CamParam.dat")   # the solve's start when a camera has no file yet


def package(cell):
    if cell.sim:
        return calibration_fake
    try:
        pkg = importlib.import_module("calibration")
        for sub in ("capture", "halcon", "marker", "plan", "solve", "upload", "verify"):
            importlib.import_module(f"calibration.{sub}")      # the package's __init__ imports none of them
        return pkg
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


# ------------------------------------------------------------------------------------------------- steps
def _picked(L: dict, camera) -> List[tuple]:
    """The one camera a step is for ("down" or "front"). The two cameras look in different directions on the hand,
    so a marker is never in both views at once: each is read and captured on its own, the plate moved between."""
    cams = _cams(L)
    if camera is None:
        return cams
    got = [c for c in cams if c[0] == camera]
    if not got:
        raise ValueError(f"there is no camera called {camera!r}; use down or front")
    return got


def read_marker(cell, camera=None) -> None:
    """One camera reads the marker on the practice plate, or the row says what to do."""
    L = cell.layout()
    said = []
    for label, cam in _picked(L, camera):
        try:
            seen = any(cell.vision.aruco_pose(cam, 1, None, L["vision"]["led"]).pixel)
        except Exception:
            seen = False
        cell.edit(lambda L, cam=cam, seen=seen: _node(L, cam).update(marker="ok" if seen else "not read", marker_at=_stamp()))
        said.append(f"{label} camera: {'reads it' if seen else 'cannot read the marker: clean the lens, move the plate closer'}")
    cell.note("; ".join(said))


def pictures(cell, by_hand: bool, camera=None) -> dict:
    """Pictures of the marker from around it for one camera (the arm drives the plan, or the user moves it by hand).
    Returns {camera name: number of pictures}."""
    calib, L = package(cell), cell.layout()
    here = cell.robot.pose()
    got = {}
    for label, cam in _picked(L, camera):
        if cell.stop_event.is_set():
            break
        got[cam] = calib.capture.run_plan(None if by_hand else plan_around(cell, L, here), cam, cell.robot, cell.vision,
                                          _dir(cell, cam, "capture"), L["vision"]["led"], stop=cell.stop_event)
    return got


def solve(cell, camera=None) -> dict:
    """One joint solve per camera from its pictures: lens numbers and where the camera sits on the hand. Redo above
    LIMIT_PX or LIMIT_MM. Returns {camera name: {"px", "mm", "pictures", "ok"}}."""
    calib, L = package(cell), cell.layout()
    out = {}
    for label, cam in _picked(L, camera):
        camparam, _ = _files(cell, cam)
        start = camparam if os.path.exists(camparam) else VENDOR_START
        res = calib.solve.run(_dir(cell, cam, "capture"), calib.halcon.CamParam.read(start), L["vision"]["marker_mm"] / 1000.0)
        res.write(_dir(cell, cam))
        px, mm = round(float(res.reprojection_px), 3), round(float(res.mean_error_m) * 1000.0, 3)
        ok = px <= LIMIT_PX and mm <= LIMIT_MM
        cell.edit(lambda L, cam=cam: _node(L, cam).update(pictures=res.images, px=px, mm=mm, captured_at=_stamp()))
        cell.note(f"{label} camera: {res.images} pictures, error {px} px, {mm} mm: "
                  f"{'ok' if ok else f'redo, above {LIMIT_PX} px or {LIMIT_MM:.0f} mm'}")
        out[cam] = {"px": px, "mm": mm, "pictures": res.images, "ok": ok}
    return out


def capture(cell, by_hand: bool, camera=None) -> None:
    """Pictures, then the solve (the two steps in one call)."""
    pictures(cell, by_hand, camera)
    if not cell.stop_event.is_set():
        solve(cell, camera)


def upload(cell, camera=None) -> None:
    """The cameras' files (one camera, or both) to the box, then the box restarts (about a minute)."""
    calib, L = package(cell), cell.layout()
    creds = calib.upload.credentials(L)
    for label, cam in _picked(L, camera):
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


def check(cell, camera=None) -> None:
    """After the upload: the box reads the marker from here and from CHECK_MOVE_MM further along x; with the new
    numbers it reports the same place both times."""
    calib, L = package(cell), cell.layout()
    here = cell.robot.pose()
    there = [here[0] + CHECK_MOVE_MM] + list(here[1:])
    for label, cam in _picked(L, camera):
        if not _node(L, cam).get("uploaded_at"):
            cell.note(f"{label} camera: upload first")
            return
        mm = round(float(calib.verify.check(cell.vision, cam, cell.robot, here, there, 1, L["vision"]["led"])), 3)
        cell.edit(lambda L, cam=cam: _node(L, cam).update(checked_mm=mm, checked_at=_stamp()))
        cell.note(f"{label} camera: {mm} mm: {'ok' if mm <= LIMIT_MM else f'too far, above {LIMIT_MM:.0f} mm'}")
    cell.robot.move_linear(here)
