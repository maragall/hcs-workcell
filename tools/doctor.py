"""
Environment and cell check: is this PC ready to run `python cli.py setup`? One line per check, PASS, WARN or FAIL, and the
fix under each FAIL or WARN. Read-only: it moves nothing, writes nothing but one temporary file in runs/, never prints the
credentials in layout.json, and never connects an SDK client (the controller takes one at a time): the arm is looked for
on its BRCAPI port only, and the SDK port is probed only when asked (--probe-sdk-port, with nothing else connected).

    python tools/doctor.py [--sim] [--probe-sdk-port]     (also: python cli.py doctor)

--sim checks the PC and the files only, for a machine with no cell attached. Exit code 0 when nothing FAILs.
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import shutil
import sys
import tempfile
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MIN_PYTHON = (3, 10)
CAMERAS_NEEDED = 2                                   # the down camera and the front camera

LAYOUT_KEYS = [
    ("robot", "ip"), ("robot", "port"),
    ("vision", "host"), ("vision", "port"), ("vision", "down_camera"), ("vision", "front_camera"), ("vision", "marker_mm"),
    ("workspace_box", "x"), ("workspace_box", "y"), ("workspace_box", "z"),
    ("regions", "microscope", "box"), ("regions", "hotel", "box"),
    ("fixtures", "hotel", "taught"), ("fixtures", "hotel", "columns"), ("fixtures", "polar", "pose_at_0", "pose"),
    ("fixtures", "microscope", "pose"), ("routes", "nest", "pose"),
    ("volumes", "obstacles", "hotel"), ("volumes", "claw", "fingers"), ("limits", "min_disk_free_gb"),
]
HARDWARE_KEYS = [("cad", "platform"), ("cad", "hotel"), ("cad", "rotation_module"),
                 ("claw", "camera_ahead_of_wrist_mm"), ("claw", "module_wrist_to_plate_mm")]


@dataclass
class Result:
    status: str                 # PASS | WARN | FAIL
    name: str
    detail: str
    fix: str = ""


@dataclass
class Env:
    """What the doctor looks at the world with; tests replace these."""
    import_module: Callable[[str], object] = importlib.import_module
    arm: Optional[Callable[[dict, bool], Dict[str, str]]] = None
    vision_box: Optional[Callable[[dict], Dict[str, str]]] = None
    squid: Optional[Callable[[dict], Dict[str, str]]] = None
    port_open: Optional[Callable[..., bool]] = None
    disk_free_gb: Callable[[str], float] = field(default=lambda p: shutil.disk_usage(p).free / 1e9)

    def discover(self):
        """The network helpers of the setup itself (bringup/discover.py), imported only when needed."""
        if self.arm is None or self.vision_box is None or self.squid is None or self.port_open is None:
            sys.path.insert(0, HERE)
            from bringup import discover
            self.arm = self.arm or discover.arm
            self.vision_box = self.vision_box or discover.vision_box
            self.squid = self.squid or discover.squid
            self.port_open = self.port_open or discover.port_open
        return self


def dig(d, path: Sequence[str]):
    for p in path:
        if not isinstance(d, dict) or p not in d:
            raise KeyError(".".join(path))
        d = d[p]
    return d


def _load(path: str):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


# ------------------------------------------------------------------------------------------------ the checks
def check_python(out: List[Result]) -> None:
    v = sys.version_info
    ok = (v.major, v.minor) >= MIN_PYTHON
    out.append(Result("PASS" if ok else "FAIL", "python", f"{v.major}.{v.minor}.{v.micro}",
                      "" if ok else f"install Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]} or newer (python.org), then run install.ps1 again"))


def check_packages(out: List[Result], env: Env) -> None:
    for module, pip, needed in (("numpy", "numpy", True), ("scipy", "scipy", True), ("cv2", "opencv-contrib-python-headless", True),
                                ("PIL", "pillow", True), ("paramiko", "paramiko", False)):
        try:
            m = env.import_module(module)
        except Exception as e:                                        # a missing or broken install, whatever the error
            out.append(Result("FAIL" if needed else "WARN", module, f"not importable ({type(e).__name__})",
                              f"pip install -r requirements.txt   (or: pip install {pip})"
                              + ("" if needed else "; only the camera-file upload needs it")))
            continue
        detail = str(getattr(m, "__version__", "ok"))
        if module == "cv2" and not hasattr(m, "aruco"):
            out.append(Result("FAIL", "cv2", f"{detail} has no aruco module", "pip uninstall opencv-python opencv-python-headless; "
                              "pip install opencv-contrib-python-headless"))
            continue
        out.append(Result("PASS", module, detail))


def check_sdk(out: List[Result], env: Env, sim: bool) -> None:
    if sim:
        out.append(Result("PASS", "arm SDK", "not needed for the simulator"))
        return
    try:
        env.import_module("clr")
        sdk = env.import_module("BioyondSDK")
        getattr(sdk, "RobotSDKFactory")
    except Exception as e:
        out.append(Result("FAIL", "arm SDK", f"cannot load the vendor SDK ({type(e).__name__})",
                          "pip install pythonnet; install the .NET 8 runtime; .\\install.ps1 -SdkWheel <the vendor .whl>"))
        return
    out.append(Result("PASS", "arm SDK", "vendor SDK loads (nothing connected)"))


def check_files(out: List[Result], layout_path: str, hardware_path: str):
    layout = hardware = None
    for label, path, keys in (("layout.json", layout_path, LAYOUT_KEYS), ("hardware.json", hardware_path, HARDWARE_KEYS)):
        try:
            data = _load(path)
        except FileNotFoundError:
            out.append(Result("FAIL", label, f"not found at {path}", "run the doctor from the program folder, or pass --layout/--hardware"))
            continue
        except ValueError as e:
            out.append(Result("FAIL", label, f"is not valid JSON ({e})", "restore the file from git: git checkout -- " + os.path.basename(path)))
            continue
        missing = []
        for k in keys:
            try:
                dig(data, k)
            except KeyError as e:
                missing.append(str(e.args[0]))
        if missing:
            out.append(Result("FAIL", label, "missing " + ", ".join(missing), f"restore the file from git, or add the keys named ({label})"))
        else:
            out.append(Result("PASS", label, f"{len(keys)} required keys present"))
        if label == "layout.json":
            layout = data
        else:
            hardware = data
    return layout, hardware


def check_nest(out: List[Result], layout: Optional[dict]) -> None:
    if layout is None:
        return
    try:
        pose = dig(layout, ("routes", "nest", "pose"))
        box = layout["workspace_box"]
        assert len(pose) == 6 and all(isinstance(v, (int, float)) for v in pose)
        for axis, i in (("x", 0), ("y", 1), ("z", 2)):
            lo, hi = box[axis]
            assert lo <= pose[i] <= hi, f"{axis} {pose[i]} is outside the workspace {lo}..{hi}"
    except (KeyError, AssertionError, TypeError, ValueError) as e:
        out.append(Result("FAIL", "nest", f"routes.nest.pose is not usable ({e})",
                          "drag the arm to a safe pose in front of the hotel and record it as routes.nest.pose (6 numbers)"))
        return
    out.append(Result("PASS", "nest", "pose " + ", ".join(f"{v:g}" for v in pose[:3]) + f", yaw {pose[5]:g}"))


def check_credentials(out: List[Result], layout: Optional[dict]) -> None:
    ssh = (layout or {}).get("vision", {}).get("ssh") or {}
    if ssh.get("user") and ssh.get("password"):
        out.append(Result("PASS", "box upload login", "set (not shown)"))
    else:
        out.append(Result("WARN", "box upload login", "vision.ssh is not set: the camera files cannot be uploaded to the box",
                          "set vision.ssh.user and vision.ssh.password in layout.json (never commit them); only the camera "
                          "calibration upload needs them"))


def check_cell(out: List[Result], layout: Optional[dict], env: Env, probe_sdk_port: bool) -> None:
    if layout is None:
        return
    env.discover()
    try:
        arm = env.arm(layout, not probe_sdk_port)
    except Exception as e:
        arm = {"error": f"{type(e).__name__}: {e}"}
    if "ip" in arm:
        what = "BRCAPI answers" if not probe_sdk_port else "BRCAPI and the SDK port answer"
        out.append(Result("PASS", "arm", f"controller at {arm['ip']} ({what})"))
    else:
        out.append(Result("FAIL", "arm", arm.get("error", "no controller answers"),
                          "power the arm and check the network cable; close RobStudio and anything else connected (one client at a time)"))
    try:
        box = env.vision_box(layout)
    except Exception as e:
        box = {"error": f"{type(e).__name__}: {e}"}
    if "cameras" in box:
        n = int(box["cameras"])
        if n >= CAMERAS_NEEDED:
            out.append(Result("PASS", "vision box", f"{box['host']}:{box['port']}, {n} cameras"))
        else:
            out.append(Result("FAIL", "vision box", f"{box['host']}:{box['port']} reports {n} camera(s), {CAMERAS_NEEDED} are needed",
                              "check both camera cables, then restart the box"))
    else:
        out.append(Result("FAIL", "vision box", box.get("error", "no vision box answers"),
                          "power the box and check the network cable; wait two minutes after a restart"))
    try:
        sq = env.squid(layout)
    except Exception as e:
        sq = {"error": f"{type(e).__name__}: {e}"}
    if "module" in sq:
        out.append(Result("PASS", "squid software", f"found at {sq.get('folder', 'installed')}"))
    else:
        out.append(Result("WARN", "squid software", sq.get("error", "not found"),
                          "set squid_software.folder in layout.json to the Squid software folder; until then the setup uses the "
                          "practice Squid (no motors, nothing is imaged)"))


def check_runs(out: List[Result], layout: Optional[dict], runs_dir: str, env: Env) -> None:
    try:
        os.makedirs(runs_dir, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=runs_dir, prefix="doctor_", delete=True):
            pass
        out.append(Result("PASS", "runs folder", f"{runs_dir} is writable"))
    except OSError as e:
        out.append(Result("FAIL", "runs folder", f"{runs_dir} is not writable ({e})",
                          "make the folder writable, or point runs/ at another drive"))
    need = float(((layout or {}).get("limits") or {}).get("min_disk_free_gb", 5.0))
    try:
        free = env.disk_free_gb(runs_dir if os.path.isdir(runs_dir) else HERE)
    except OSError as e:
        out.append(Result("WARN", "disk space", f"cannot read ({e})"))
        return
    if free >= need:
        out.append(Result("PASS", "disk space", f"{free:.0f} GB free (needs {need:g})"))
    else:
        out.append(Result("FAIL", "disk space", f"{free:.1f} GB free, {need:g} needed", "free some space, or move runs/ to a bigger drive"))
    for marker in ("STOP", "COLLISION"):
        if os.path.exists(os.path.join(runs_dir, marker)):
            out.append(Result("WARN", f"runs/{marker}", f"the file {marker} is present",
                              {"STOP": "delete runs/STOP: the setup ends at its next step while it exists",
                               "COLLISION": "a run stopped after a collision: look at the cell, then delete runs/COLLISION; "
                                            "the unattended loop will not start while it exists"}[marker]))


def check(layout_path: Optional[str] = None, hardware_path: Optional[str] = None, runs_dir: Optional[str] = None,
          sim: bool = False, probe_sdk_port: bool = False, env: Optional[Env] = None) -> List[Result]:
    layout_path = layout_path or os.path.join(HERE, "layout.json")
    hardware_path = hardware_path or os.path.join(os.path.dirname(layout_path), "hardware.json")
    runs_dir = runs_dir or os.path.join(os.path.dirname(layout_path), "runs")
    env = env or Env()
    out: List[Result] = []
    check_python(out)
    check_packages(out, env)
    check_sdk(out, env, sim)
    layout, _ = check_files(out, layout_path, hardware_path)
    check_nest(out, layout)
    check_credentials(out, layout)
    check_runs(out, layout, runs_dir, env)
    if not sim:
        check_cell(out, layout, env, probe_sdk_port)
    return out


def report(results: List[Result], say: Callable[[str], None] = print) -> int:
    for r in results:
        say(f"{r.status:4}  {r.name}: {r.detail}")
        if r.fix and r.status != "PASS":
            say(f"      fix: {r.fix}")
    fails = sum(r.status == "FAIL" for r in results)
    warns = sum(r.status == "WARN" for r in results)
    say("")
    say(f"{'READY: python cli.py setup' if not fails else str(fails) + ' check(s) FAIL: fix them, then run the doctor again'}"
        + (f" ({warns} warning{'s' if warns != 1 else ''})" if warns else ""))
    return 1 if fails else 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Is this PC and cell ready to run the setup? Read-only.")
    p.add_argument("--sim", action="store_true", help="the PC and the files only, no cell attached")
    p.add_argument("--probe-sdk-port", action="store_true", help="also probe the arm's SDK port (only with nothing else connected)")
    p.add_argument("--layout", help="layout.json (default: next to this program)")
    p.add_argument("--hardware", help="hardware.json (default: next to layout.json)")
    p.add_argument("--runs", help="the runs folder (default: next to layout.json)")
    a = p.parse_args(argv)
    return report(check(a.layout, a.hardware, a.runs, a.sim, a.probe_sdk_port))


if __name__ == "__main__":
    sys.exit(main())
