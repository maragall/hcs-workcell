"""The doctor (setup/doctor.py): a read-only check that this PC and cell are ready for `python cli.py setup`. On the real files with
no cell attached, with parts missing, and with the cell answering through fakes."""
from __future__ import annotations

import json
import os
import shutil
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)

from setup import doctor  # noqa: E402


@pytest.fixture
def cell_files(tmp_path):
    """layout.json and hardware.json copied into a temporary folder, with a runs/ beside them."""
    for name in ("layout.json", "hardware.json"):
        shutil.copy(os.path.join(ROOT, "run", name), tmp_path / name)
    return tmp_path


def run(folder, **kw):
    return doctor.check(str(folder / "layout.json"), str(folder / "hardware.json"), str(folder / "runs"), **kw)


def by_name(results):
    return {r.name: r for r in results}


def edit_layout(folder, fn):
    path = folder / "layout.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    fn(data)
    path.write_text(json.dumps(data), encoding="utf-8")


DESKTOP_8 = ["Microsoft.NETCore.App 8.0.11 [C:\\Program Files\\dotnet\\shared\\Microsoft.NETCore.App]",
             "Microsoft.WindowsDesktop.App 8.0.11 [C:\\Program Files\\dotnet\\shared\\Microsoft.WindowsDesktop.App]"]
SQUID_OK = {"microscope": {"at": "127.0.0.1:5050", "acquiring": False, "python_exec": True}}


class FakeSdk:
    RobotSDKFactory = object


def fakes(arm=None, squid=None, imports=None, free=500.0, runtimes=DESKTOP_8, lua="C:/Bioyond/RobStudio/lua54.dll", windows=True, bits=64,
          sdk_error=None):
    """An Env that looks like a ready Windows PC with the cell answering; each argument breaks one thing."""
    seen = {"arm_connected_flag": []}

    def arm_fn(layout, connected=False):
        seen["arm_connected_flag"].append(connected)
        return arm if arm is not None else {"ip": "192.168.1.220", "port": 52525}

    def import_fn(name):
        if imports and name in imports:
            raise ModuleNotFoundError(f"No module named {name!r}", name=name)
        if name == "BioyondSDK":
            if sdk_error:
                raise sdk_error
            return FakeSdk
        if name == "pythonnet":
            return object()
        return __import__(name)
    env = doctor.Env(import_module=import_fn, arm=arm_fn, squid=lambda L: SQUID_OK if squid is None else squid,
                     port_open=lambda *a, **k: True, disk_free_gb=lambda p: free, dotnet_runtimes=lambda: runtimes,
                     lua_dll=lambda L: lua, windows=windows, bits=bits)
    return env, seen


def test_a_ready_pc_and_cell_pass_every_check(cell_files):
    env, _ = fakes()
    results = run(cell_files, env=env)
    assert [r.name for r in results if r.status != "PASS"] == []
    assert {"windows", "numpy", ".NET runtime", "arm SDK", "BioyondWorks Lua", "arm", "squid microscope"} <= set(by_name(results))
    assert doctor.report(results, say=lambda s: None) == 0


def test_no_check_mentions_a_camera(cell_files):
    env, _ = fakes()
    words = " ".join(f"{r.name} {r.detail} {r.fix}" for r in run(cell_files, env=env)).lower()
    assert "camera" not in words and "vision" not in words


def test_a_missing_package_fails_with_the_command_that_fixes_it(cell_files):
    env, _ = fakes(imports={"numpy"})
    r = by_name(run(cell_files, sim=True, env=env))["numpy"]
    assert r.status == "FAIL" and "install.bat" in r.fix and "pip install" in r.fix


def test_the_arm_sdk_is_checked_for_a_real_cell_and_not_for_the_simulator(cell_files):
    env, _ = fakes(imports={"BioyondSDK"})
    assert by_name(run(cell_files, sim=True, env=env))["arm SDK"].status == "PASS"
    real = by_name(run(cell_files, sim=False, env=env))["arm SDK"]
    assert real.status == "FAIL" and "not installed" in real.detail and ".whl" in real.fix


def test_each_missing_part_of_the_arm_sdk_gets_its_own_fix(cell_files):
    env, _ = fakes(imports={"pythonnet"})
    r = by_name(run(cell_files, env=env))["arm SDK"]
    assert r.status == "FAIL" and "pythonnet" in r.detail
    env, _ = fakes(sdk_error=RuntimeError("BR.PC.BYSDK could not be loaded"))       # the package is there, .NET is not
    r = by_name(run(cell_files, env=env))["arm SDK"]
    assert r.status == "FAIL" and "does not load" in r.detail and ".NET Desktop Runtime 8" in r.fix


def test_the_dotnet_desktop_runtime_8_is_looked_for(cell_files):
    only_core = ["Microsoft.NETCore.App 8.0.11 [C:\\x]", "Microsoft.WindowsDesktop.App 6.0.36 [C:\\x]"]
    env, _ = fakes(runtimes=only_core)
    r = by_name(run(cell_files, env=env))[".NET runtime"]
    assert r.status == "FAIL" and "6.0.36" in r.detail and "dotnet.microsoft.com" in r.fix
    env, _ = fakes(runtimes=None)                                                    # no `dotnet` command: cannot tell, the SDK check decides
    assert by_name(run(cell_files, env=env))[".NET runtime"].status == "WARN"
    env, _ = fakes(runtimes=["Microsoft.WindowsDesktop.App 9.0.1 [C:\\x]"])
    assert by_name(run(cell_files, env=env))[".NET runtime"].status == "PASS"


def test_a_pc_that_is_not_64_bit_windows_fails_for_the_real_arm_only(cell_files):
    env, _ = fakes(windows=False)
    assert by_name(run(cell_files, env=env))["windows"].status == "FAIL"
    assert "windows" not in by_name(run(cell_files, sim=True, env=env))
    env, _ = fakes(bits=32)
    r = by_name(run(cell_files, env=env))["windows"]
    assert r.status == "FAIL" and "64-bit Python" in r.fix


def test_bioyondworks_lua_missing_fails_because_the_setup_cannot_upload_without_it(cell_files):
    env, _ = fakes(lua=None)
    r = by_name(run(cell_files, env=env))["BioyondWorks Lua"]
    assert r.status == "FAIL" and '"lua"' in r.fix


def test_a_broken_or_missing_file_fails_with_a_fix(cell_files):
    (cell_files / "hardware.json").write_text("{not json", encoding="utf-8")
    (cell_files / "layout.json").unlink()
    results = by_name(run(cell_files, sim=True))
    assert results["hardware.json"].status == "FAIL" and "git checkout" in results["hardware.json"].fix
    assert results["layout.json"].status == "FAIL" and "not found" in results["layout.json"].detail


def test_a_cell_that_answers_passes_and_the_sdk_port_is_left_alone_by_default(cell_files):
    env, seen = fakes()
    results = by_name(run(cell_files, env=env))
    assert [results[k].status for k in ("arm", "squid microscope")] == ["PASS"] * 2
    assert seen["arm_connected_flag"] == [True]                    # connected=True: only BRCAPI is probed, never the SDK port
    env, seen = fakes()
    run(cell_files, env=env, probe_sdk_port=True)
    assert seen["arm_connected_flag"] == [False]


def test_a_dead_arm_fails_with_what_to_do_and_a_silent_squid_warns(cell_files):
    env, _ = fakes(arm={"error": "no controller answers on ports 52525 or 56788", "fix": "set this PC's adapter to 192.168.1.x"},
                   squid={"microscope": {"at": "127.0.0.1:5050", "error": "microscope: Squid does not answer on 127.0.0.1:5050"}})
    results = by_name(run(cell_files, env=env))
    assert results["arm"].status == "FAIL" and "192.168.1.x" in results["arm"].fix and "one client at a time" in results["arm"].fix
    assert results["squid microscope"].status == "WARN" and "Enable MCP Control Server" in results["squid microscope"].fix


def test_squid_with_python_exec_off_passes_and_says_so(cell_files):
    env, _ = fakes(squid={"microscope": {"at": "127.0.0.1:5050", "acquiring": False, "python_exec": False}})
    r = by_name(run(cell_files, env=env))["squid microscope"]
    assert r.status == "PASS" and "Python Exec is off" in r.detail


def test_low_disk_space_fails(cell_files):
    env, _ = fakes(free=1.0)
    assert by_name(run(cell_files, sim=True, env=env))["disk space"].status == "FAIL"


def test_leftover_stop_and_collision_files_are_warned_about(cell_files):
    (cell_files / "runs").mkdir()
    (cell_files / "runs" / "STOP").write_text("")
    (cell_files / "runs" / "COLLISION").write_text("")
    results = by_name(run(cell_files, sim=True))
    assert results["runs/STOP"].status == "WARN" and results["runs/COLLISION"].status == "WARN"
