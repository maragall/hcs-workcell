"""The doctor (tools/doctor.py): a read-only check that this PC and cell are ready for `python cli.py setup`. On the real files with
no cell attached, with parts missing, with the cell answering through fakes, and with credentials that must never be printed."""
from __future__ import annotations

import json
import os
import shutil
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from tools import doctor  # noqa: E402


@pytest.fixture
def cell_files(tmp_path):
    """layout.json and hardware.json copied into a temporary folder, with a runs/ beside them."""
    for name in ("layout.json", "hardware.json"):
        shutil.copy(os.path.join(ROOT, name), tmp_path / name)
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


def fakes(arm=None, box=None, squid=None, imports=None, free=500.0):
    seen = {"arm_connected_flag": []}

    def arm_fn(layout, connected=False):
        seen["arm_connected_flag"].append(connected)
        return arm if arm is not None else {"ip": "192.168.1.220", "port": 52525}

    def import_fn(name):
        if imports and name in imports:
            raise ImportError(name)
        return __import__(name)
    env = doctor.Env(import_module=import_fn, arm=arm_fn,
                     vision_box=lambda L: box if box is not None else {"host": "192.168.3.100", "port": 9000, "version": "v", "cameras": 2},
                     squid=lambda L: squid if squid is not None else {"module": "control.microscope", "folder": "C:/Squid"},
                     port_open=lambda *a, **k: True, disk_free_gb=lambda p: free)
    return env, seen


def test_the_real_files_pass_with_no_cell_attached(cell_files):
    results = run(cell_files, sim=True)
    assert not [r for r in results if r.status == "FAIL"], [(r.name, r.detail) for r in results if r.status == "FAIL"]
    assert by_name(results)["nest"].status == "PASS" and by_name(results)["layout.json"].status == "PASS"


def test_a_missing_package_fails_with_the_command_that_fixes_it(cell_files):
    env, _ = fakes(imports={"cv2"})
    r = by_name(run(cell_files, sim=True, env=env))["cv2"]
    assert r.status == "FAIL" and "pip install" in r.fix


def test_the_optional_upload_package_is_only_a_warning(cell_files):
    env, _ = fakes(imports={"paramiko"})
    assert by_name(run(cell_files, sim=True, env=env))["paramiko"].status == "WARN"


def test_the_arm_sdk_is_checked_for_a_real_cell_and_not_for_the_simulator(cell_files):
    env, _ = fakes(imports={"clr", "BioyondSDK"})
    assert by_name(run(cell_files, sim=True, env=env))["arm SDK"].status == "PASS"
    real = by_name(run(cell_files, sim=False, env=env))["arm SDK"]
    assert real.status == "FAIL" and "SdkWheel" in real.fix


def test_a_layout_without_the_nest_fails_and_says_which_key(cell_files):
    edit_layout(cell_files, lambda L: L["routes"].pop("nest"))
    results = by_name(run(cell_files, sim=True))
    assert results["layout.json"].status == "FAIL" and "routes.nest.pose" in results["layout.json"].detail
    assert results["nest"].status == "FAIL"


def test_a_nest_outside_the_workspace_fails(cell_files):
    edit_layout(cell_files, lambda L: L["routes"]["nest"].update(pose=[126.0, 4.0, 999.0, 0, 0, 0]))
    assert by_name(run(cell_files, sim=True))["nest"].status == "FAIL"


def test_a_broken_or_missing_file_fails_with_a_fix(cell_files):
    (cell_files / "hardware.json").write_text("{not json", encoding="utf-8")
    (cell_files / "layout.json").unlink()
    results = by_name(run(cell_files, sim=True))
    assert results["hardware.json"].status == "FAIL" and "git checkout" in results["hardware.json"].fix
    assert results["layout.json"].status == "FAIL" and "not found" in results["layout.json"].detail


def test_a_cell_that_answers_passes_and_the_sdk_port_is_left_alone_by_default(cell_files):
    env, seen = fakes()
    results = by_name(run(cell_files, env=env))
    assert [results[k].status for k in ("arm", "vision box", "squid software")] == ["PASS"] * 3
    assert seen["arm_connected_flag"] == [True]                    # connected=True: only BRCAPI is probed, never the SDK port
    env, seen = fakes()
    run(cell_files, env=env, probe_sdk_port=True)
    assert seen["arm_connected_flag"] == [False]


def test_a_dead_arm_and_a_one_camera_box_fail_and_a_missing_squid_only_warns(cell_files):
    env, _ = fakes(arm={"error": "no controller answers on ports 52525 or 56788"},
                   box={"host": "192.168.3.100", "port": 9000, "version": "v", "cameras": 1},
                   squid={"error": "can't find the Squid software folder", "browse": True})
    results = by_name(run(cell_files, env=env))
    assert results["arm"].status == "FAIL" and "one client at a time" in results["arm"].fix
    assert results["vision box"].status == "FAIL" and "camera" in results["vision box"].detail
    assert results["squid software"].status == "WARN"


def test_a_box_that_does_not_answer_fails(cell_files):
    env, _ = fakes(box={"error": "no vision box answers: 192.168.3.100:9000 timed out"})
    r = by_name(run(cell_files, env=env))["vision box"]
    assert r.status == "FAIL" and "timed out" in r.detail


def test_low_disk_space_fails(cell_files):
    env, _ = fakes(free=1.0)
    assert by_name(run(cell_files, sim=True, env=env))["disk space"].status == "FAIL"


def test_leftover_stop_and_collision_files_are_warned_about(cell_files):
    (cell_files / "runs").mkdir()
    (cell_files / "runs" / "STOP").write_text("")
    (cell_files / "runs" / "COLLISION").write_text("")
    results = by_name(run(cell_files, sim=True))
    assert results["runs/STOP"].status == "WARN" and results["runs/COLLISION"].status == "WARN"


def test_the_credentials_in_the_layout_are_never_printed(cell_files, capsys):
    def put(L):
        L["vision"]["ssh"] = {"user": "SECRET-USER", "password": "SECRET-PASSWORD"}
    edit_layout(cell_files, put)
    results = run(cell_files, sim=True)
    doctor.report(results)
    shown = capsys.readouterr().out
    assert "SECRET" not in shown and by_name(results)["box upload login"].status == "PASS"


def test_the_exit_code_and_the_last_line_follow_the_fails(cell_files, capsys):
    lines = []
    assert doctor.report(run(cell_files, sim=True), lines.append) == 0 and lines[-1].startswith("READY")
    env, _ = fakes(imports={"cv2"})
    assert doctor.report(run(cell_files, sim=True, env=env), lines.append) == 1 and "FAIL" in lines[-1]
