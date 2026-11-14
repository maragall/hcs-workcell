"""
Environment and cell check: is this PC ready to run `python cli.py setup`? One line per check, PASS, WARN or FAIL, and the
fix under each FAIL or WARN. Read-only: it moves nothing, writes nothing but one temporary file in runs/, never prints the
contents of layout.json beyond what a check names, and never connects an SDK client (the controller takes one at a time): the arm is looked for
on its BRCAPI port only, and the SDK port is probed only when asked (--probe-sdk-port, with nothing else connected).

    python setup/doctor.py [--sim] [--probe-sdk-port]     (also: python cli.py doctor)

--sim checks the PC and the files only, for a machine with no cell attached. Exit code 0 when nothing FAILs.
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MIN_PYTHON = (3, 10)
DOTNET_MAJOR = 8                                     # the vendor package loads on the .NET Desktop Runtime 8 or newer
DOTNET_URL = "https://dotnet.microsoft.com/download/dotnet/8.0"

LAYOUT_KEYS = [
    ("robot", "ip"), ("robot", "port"),
    ("workspace_box", "x"), ("workspace_box", "y"), ("workspace_box", "z"),
    ("regions", "microscope", "box"), ("regions", "hotel", "box"),
    ("fixtures", "hotel", "taught"), ("fixtures", "hotel", "columns"), ("fixtures", "polar", "pose_at_0", "pose"),
    ("fixtures", "microscope", "pose"),
    ("volumes", "obstacles", "hotel"), ("volumes", "claw", "fingers"), ("limits", "min_disk_free_gb"),
]
HARDWARE_KEYS = [("cad", "platform"), ("cad", "hotel"), ("cad", "rotation_module"),
                 ("claw", "module_wrist_to_plate_mm")]


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
    squid: Optional[Callable[[dict], Dict[str, dict]]] = None
    dotnet_runtimes: Optional[Callable[[], Optional[List[str]]]] = None      # the lines of `dotnet --list-runtimes`, None: no dotnet command
    lua_dll: Optional[Callable[[Optional[dict]], Optional[str]]] = None
    windows: bool = field(default_factory=lambda: sys.platform == "win32")
    bits: int = field(default_factory=lambda: struct.calcsize("P") * 8)
    port_open: Optional[Callable[..., bool]] = None
    disk_free_gb: Callable[[str], float] = field(default=lambda p: shutil.disk_usage(p).free / 1e9)

    def discover(self):
        """The network helpers of the setup itself (setup/discover.py), imported only when needed."""
        if self.arm is None or self.squid is None or self.port_open is None:
            sys.path.insert(0, HERE)
            from setup import discover
            self.arm = self.arm or discover.arm
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
                      "" if ok else f"install Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]} or newer (python.org), then run setup\\install.bat again"))


def check_platform(out: List[Result], env: Env, sim: bool) -> None:
    """The real arm is driven through the vendor's .NET package: Windows, 64 bit. The simulator runs anywhere."""
    if sim:
        return
    if not env.windows:
        out.append(Result("FAIL", "windows", f"this is {sys.platform}", "the arm's vendor package runs on 64-bit Windows only; elsewhere use --sim"))
    elif env.bits != 64:
        out.append(Result("FAIL", "windows", f"Python is {env.bits} bit", "install the 64-bit Python from python.org (Windows installer (64-bit)), then run setup\\install.bat again"))
    else:
        out.append(Result("PASS", "windows", "64-bit Windows, 64-bit Python"))


def check_packages(out: List[Result], env: Env) -> None:
    for module, pip in (("numpy", "numpy"),):
        try:
            m = env.import_module(module)
        except Exception as e:                                        # a missing or broken install, whatever the error
            out.append(Result("FAIL", module, f"not importable ({type(e).__name__})",
                              f"run setup\\install.bat again   (or: pip install -r setup/requirements.txt, or: pip install {pip})"))
            continue
        out.append(Result("PASS", module, str(getattr(m, "__version__", "ok"))))


def _dotnet_runtimes() -> Optional[List[str]]:
    try:
        r = subprocess.run(["dotnet", "--list-runtimes"], capture_output=True, text=True, errors="replace", timeout=20)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.stdout.splitlines()


def check_dotnet(out: List[Result], env: Env, sim: bool) -> None:
    """The .NET Desktop Runtime the vendor package needs (Microsoft.WindowsDesktop.App, version 8 or newer)."""
    if sim:
        return
    lines = (env.dotnet_runtimes or _dotnet_runtimes)()
    fix = f"install the .NET Desktop Runtime {DOTNET_MAJOR} (x64) from {DOTNET_URL}, then open a new terminal"
    if lines is None:
        out.append(Result("WARN", ".NET runtime", "the `dotnet` command is not found, so the runtime cannot be listed",
                          f"if the arm SDK check below fails: {fix}"))
        return
    have = []
    for line in lines:
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "Microsoft.WindowsDesktop.App" and parts[1].split(".")[0].isdigit():
            have.append(parts[1])
    good = [v for v in have if int(v.split(".")[0]) >= DOTNET_MAJOR]
    if good:
        out.append(Result("PASS", ".NET runtime", f"Desktop Runtime {max(good, key=lambda v: [int(x) if x.isdigit() else 0 for x in v.split('.')])}"))
    else:
        out.append(Result("FAIL", ".NET runtime", "no Desktop Runtime " + (f"{DOTNET_MAJOR} or newer (found {', '.join(have)})" if have else "installed"), fix))


def check_sdk(out: List[Result], env: Env, sim: bool) -> None:
    """The vendor package as utils/devices/arm.py loads it, each missing part with its own fix. Nothing connects."""
    if sim:
        out.append(Result("PASS", "arm SDK", "not needed for the simulator"))
        return
    try:
        env.import_module("pythonnet")
    except Exception as e:
        out.append(Result("FAIL", "arm SDK", f"pythonnet is not installed ({type(e).__name__})", "run setup\\install.bat again   (or: pip install -r setup/requirements.txt)"))
        return
    try:
        sdk = env.import_module("BioyondSDK")
        getattr(sdk, "RobotSDKFactory")
    except ModuleNotFoundError as e:
        if getattr(e, "name", "") in ("BioyondSDK", None, ""):
            out.append(Result("FAIL", "arm SDK", "the vendor package (BioyondSDK) is not installed",
                              "setup\\install.bat <path to bioyondsdk-0.1.1-py3-none-any.whl>   (or put the .whl into setup\\vendor and run setup\\install.bat)"))
        else:
            out.append(Result("FAIL", "arm SDK", f"the vendor package needs {e.name}, which is missing", "run setup\\install.bat again"))
        return
    except Exception as e:
        out.append(Result("FAIL", "arm SDK", f"the vendor package is installed but does not load ({type(e).__name__}: {str(e)[:120]})",
                          f"install the .NET Desktop Runtime {DOTNET_MAJOR} (x64) from {DOTNET_URL}; use a 64-bit Python; then open a new terminal"))
        return
    out.append(Result("PASS", "arm SDK", "vendor package loads (nothing connected)"))


def check_lua(out: List[Result], layout: Optional[dict], env: Env, sim: bool) -> None:
    """BioyondWorks' own Lua: the setup parses the arm's program with it before the upload, and refuses to upload unparsed."""
    if sim:
        return
    if env.lua_dll is None:
        sys.path.insert(0, HERE)
        from run import lua
        env.lua_dll = lua.find_dll
    path = env.lua_dll(layout)
    if path:
        out.append(Result("PASS", "BioyondWorks Lua", path))
    else:
        out.append(Result("FAIL", "BioyondWorks Lua", "lua54.dll of BioyondWorks is not found on this PC",
                          'install BioyondWorks (then close it), or put the full path of its lua54.dll into layout.json as "lua": {"dll": "..."}'))


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
                          (arm.get("fix") or "power the arm and check the network cable")
                          + "; close BioyondWorks, RobStudio and anything else connected (one client at a time)"))
    try:
        scopes = env.squid(layout)
    except Exception as e:
        scopes = {"squid": {"at": "?", "error": f"{type(e).__name__}: {e}"}}
    if not scopes:
        out.append(Result("WARN", "squid", "no microscope is listed in layout.json (squid)",
                          "nothing to do for an arm without a microscope; with one, layout.json squid.<name> names it"))
    for name, got in scopes.items():
        if "error" in got:
            out.append(Result("WARN", f"squid {name}", got["error"],
                              "start Squid on this PC, then in the Squid window: Settings > Enable MCP Control Server. The setup's connect "
                              "section asks it again; teach and the plate moves cannot run without it"))
        else:
            out.append(Result("PASS", f"squid {name}", f"answers at {got['at']}" + ("" if got.get("python_exec") is not False else
                              "; Python Exec is off (Squid asks for it when teach or a plate move needs it: click Yes there)")))

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
    layout_path = layout_path or os.path.join(HERE, "run", "layout.json")
    hardware_path = hardware_path or os.path.join(os.path.dirname(layout_path), "hardware.json")
    runs_dir = runs_dir or os.path.join(HERE, "runs")
    env = env or Env()
    out: List[Result] = []
    check_python(out)
    check_platform(out, env, sim)
    check_packages(out, env)
    check_dotnet(out, env, sim)
    check_sdk(out, env, sim)
    layout, _ = check_files(out, layout_path, hardware_path)
    check_lua(out, layout, env, sim)
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
