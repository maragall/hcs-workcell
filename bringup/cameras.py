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
from typing import List, Optional

from procedures.safety import Safety, SafetyViolation
from routes.volumes import Volumes

from bringup import calibration_fake

LIMIT_PX, LIMIT_MM = 0.5, 1.0            # the vendor's: redo above these
def limit_mm(L: dict) -> float:
    """The error the setup accepts, in mm: the same limit for the solve and for the check after the upload."""
    return float((L["vision"].get("calibration_limits") or {}).get("mm", LIMIT_MM))


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


def plan_around(cell, L: dict, here: List[float], view: Optional[dict] = None) -> List[List[float]]:
    """The package's plan around where the arm is. Every pose is checked against the safe box and against the claw
    and links (routes/volumes.py: the fingers are 205 mm ahead of the wrist), and so is every leg between them, which
    the arm makes as straight moves; ends where it started."""
    s = Safety(L, "supervised", cell.robot.joints)
    vol = Volumes(L, s)

    def ok(p):
        if view and list(p) != list(here) and not package(cell).plan.tag_in_view(p, here, view["tag_xy"], view["dist_mm"]):
            raise SafetyViolation("the tag would not be whole in the picture")
        s.check_target(p, "capture plan")
        if vol.active:
            vol.check_pose(p, "capture plan")

    if vol.active:
        vol.settle(here)
    plan_mod = package(cell).plan
    if view and hasattr(plan_mod, "around_tag"):
        raw = plan_mod.around_tag(here, view["tag_xy"], view["dist_mm"])
        poses = []
        for p in raw:
            try:
                ok(p)
                poses.append(p)
            except Exception:
                continue
    else:
        poses = plan_mod.around(here, ok)
    if vol.active:
        prev = here
        for i, p in enumerate(list(poses) + [here], 1):
            vol.check_leg(prev, list(p), f"capture plan leg {i}/{len(poses) + 1}")
            prev = list(p)
    return poses


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


def _local_reads(cell, cam: str, led) -> bool:
    """Our own detector (calibration/marker.py) on a full frame of that camera."""
    try:
        import cv2
        import numpy as np
        from calibration import marker
        data = cell.vision.frame_full(cam, led)
        marker.detect(cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE), None)
        return True
    except Exception:
        return False


def read_marker(cell, camera=None) -> None:
    """One camera reads the marker on the practice plate, or the row says what to do."""
    L = cell.layout()
    said = []
    for label, cam in _picked(L, camera):
        try:
            seen = any(cell.vision.aruco_pose(cam, 1, None, L["vision"]["led"]).pixel)
        except Exception:
            seen = False
        if not seen:                                       # the box's reader misses the plate's tag in glare: ours flattens the light
            seen = _local_reads(cell, cam, L["vision"]["led"])
        cell.edit(lambda L, cam=cam, seen=seen: _node(L, cam).update(marker="ok" if seen else "not read", marker_at=_stamp()))
        said.append(f"{label} camera: {'reads it' if seen else 'cannot read the marker: clean the lens, move the plate closer'}")
    cell.note("; ".join(said))


def pictures(cell, by_hand: bool, camera=None, view: Optional[dict] = None) -> dict:
    """Pictures of the marker from around it for one camera (the arm drives the plan, or the user moves it by hand).
    Returns {camera name: number of pictures}."""
    calib, L = package(cell), cell.layout()
    here = cell.robot.pose()
    got = {}
    for label, cam in _picked(L, camera):
        if cell.stop_event.is_set():
            break
        folder = _dir(cell, cam, "capture")
        if not by_hand:                                    # a plan is one set: pictures of an earlier run (another arm state) would mix in
            for name in os.listdir(folder):
                if name.endswith((".png", ".json")):
                    os.remove(os.path.join(folder, name))
        got[cam] = calib.capture.run_plan(None if by_hand else plan_around(cell, L, here, view), cam, cell.robot, cell.vision,
                                          folder, L["vision"]["led"], stop=cell.stop_event)
    return got


# The tag tab (42 x 60 x 2 mm, 40 x 40 mm black frame) sits in a 2.2 mm pocket of the plate: its face is 0.2 mm below the plate's top
# (drawings 视觉标定板带aruco v3.0 and aruco标定板).
TAG_RECESS_MM = 0.2


def marker_height_m(L: dict) -> Optional[float]:
    """The practice plate's marker above the base plane: the module's top from the CAD (hardware.json) in the robot's z
    (volumes.z_offset_k) plus the practice plate's height. None where the cell has no CAD volumes (the simulator), and
    the solve is then the general one."""
    try:
        from procedures import cad_poses
        top = cad_poses.load_hardware()["cad"]["rotation_module"]["top_y_c"]
        return (top + L["volumes"]["z_offset_k"] + L["plate"]["height_practice"] - TAG_RECESS_MM) / 1000.0
    except (KeyError, TypeError, OSError):
        return None


def solve(cell, camera=None) -> dict:
    """One solve per camera from its pictures: lens numbers and where the camera sits on the hand. A solve above the
    limits (vision.calibration_limits px / mm in the positions file, else LIMIT_PX / LIMIT_MM) writes no file and no
    number into the camera's record except `last_failed`, and the next solve starts from the vendor's numbers or the
    last calibration that was accepted, never from a refused one. Returns {camera: {"px", "mm", "pictures", "ok"}}."""
    calib, L = package(cell), cell.layout()
    limits = L["vision"].get("calibration_limits") or {}
    lim_px, lim_mm = float(limits.get("px", LIMIT_PX)), float(limits.get("mm", LIMIT_MM))
    out = {}
    for label, cam in _picked(L, camera):
        camparam, _ = _files(cell, cam)                     # exists only for an accepted calibration
        start = camparam if os.path.exists(camparam) else VENDOR_START
        res = calib.solve.run(_dir(cell, cam, "capture"), calib.halcon.CamParam.read(start), L["vision"]["marker_mm"] / 1000.0,
                              marker_z_m=marker_height_m(L))
        px, mm = round(float(res.reprojection_px), 3), round(float(res.mean_error_m) * 1000.0, 3)
        ok = px <= lim_px and mm <= lim_mm
        if ok:
            res.write(_dir(cell, cam))
            cell.edit(lambda L, cam=cam: _node(L, cam).update(pictures=res.images, px=px, mm=mm, captured_at=_stamp()))
        else:
            cell.edit(lambda L, cam=cam: _node(L, cam).update(last_failed={"pictures": res.images, "px": px, "mm": mm, "at": _stamp()}))
        import hashlib
        src = [getattr(calib, m).__file__ for m in ("solve", "marker") if hasattr(getattr(calib, m, None), "__file__")]
        tag = ", ".join(f"{os.path.basename(f)} {hashlib.md5(open(f, 'rb').read()).hexdigest()[:8]}" for f in src)
        total = len([n for n in os.listdir(_dir(cell, cam, "capture")) if n.endswith(".png")])
        cell.note(f"{label} camera: solver {os.path.dirname(src[0]) if src else '?'} ({tag}); {total} captures, "
                  f"{len(res.skipped)} left out ({', '.join(res.skipped) or 'none'}): not read, a tag corner within "
                  f"{getattr(calib.solve, 'EDGE_MARGIN_PX', '?')} px of the edge, repeated pose, tag too large, or fitted worse than 3x the median; "
                  f"tag shape corners {[round(v, 2) for v in getattr(res, 'marker_shape_mm', ())]} mm")
        cell.note(f"{label} camera: {res.images} pictures, error {px} px, {mm} mm: "
                  f"{'ok' if ok else f'redo, above {lim_px:g} px or {lim_mm:g} mm; nothing was written'}")
        out[cam] = {"px": px, "mm": mm, "pictures": res.images, "ok": ok}
    return out


def capture(cell, by_hand: bool, camera=None) -> None:
    """Pictures, then the solve (the two steps in one call)."""
    pictures(cell, by_hand, camera)
    if not cell.stop_event.is_set():
        solve(cell, camera)


def calib_upload_errors():
    """The exception that means "no login for the box" (calibration/upload.py)."""
    from calibration import upload as _up
    return _up.NoCredentials


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
    gone = time.time() + 45                         # the box answers for a few seconds after the reboot command: wait for it to go down first
    while time.time() < gone and not cell.stop_event.is_set():
        try:
            cell.vision.status()
            time.sleep(2)
        except Exception:
            break
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
        cell.note(f"{label} camera: {mm} mm: {'ok' if mm <= limit_mm(L) else f'too far, above {limit_mm(L):g} mm'}")
    cell.robot.move_linear(here)
