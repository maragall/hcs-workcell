"""The setup program on the simulator: endpoints answer, done-states follow the data, a Record writes the pose,
the frame buffer serves frames and feeds the recorder off the checks' path, STOP sets the stop."""
from __future__ import annotations

import json
import os
import sys
import time
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from gui import server, sim  # noqa: E402
from gui.cell import Cell  # noqa: E402
from procedures import positions  # noqa: E402
from procedures.recorder import Recorder  # noqa: E402


@pytest.fixture(scope="module")
def gui():
    cell = Cell(*sim.build(), sim=True)
    cell.connect()
    cell.discover()
    srv = server.serve(cell, 0)
    url = f"http://127.0.0.1:{srv.server_address[1]}"
    for _ in range(50):
        if cell.frames.by_label("down"):
            break
        time.sleep(0.1)
    yield cell, url
    srv.shutdown()


def get(url, path):
    with urllib.request.urlopen(url + path, timeout=10) as r:
        return r.status, r.headers.get("Content-Type", ""), r.read()


def post(url, path, body=None):
    req = urllib.request.Request(url + path, data=json.dumps(body or {}).encode(), headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def state(url):
    return json.loads(get(url, "/api/state")[2])


def wait_idle(cell, timeout=60):
    t0 = time.time()
    while cell.busy and time.time() - t0 < timeout:
        time.sleep(0.1)
    assert cell.busy is None, f"still busy: {cell.busy}"


def row(s, rid):
    return next(r for r in s["teach"]["rows"] if r["id"] == rid)


def test_endpoints_answer(gui):
    cell, url = gui
    code, ctype, body = get(url, "/")
    assert code == 200 and "text/html" in ctype and b"Test everything" in body
    s = state(url)
    assert s["sim"] and s["arm"]["ok"] and {"teach", "test", "cameras", "devices"} <= set(s)
    assert [d["name"] for d in s["devices"]] == ["Arm", "Vision box", "Rotation module", "Microscope"]


def test_frame_buffer_serves_frames(gui):
    cell, url = gui
    code, ctype, body = get(url, "/frame/down")
    assert code == 200 and ctype == "image/jpeg" and body[:2] == b"\xff\xd8"
    assert cell.frames.frame("auto_teach_camera", 1280, 720, 85, None, 0) == body


def test_recorder_reads_the_buffer_and_never_asks_the_box(gui):
    """The recorder grabs from the frame source, so it adds no load to the box and no latency to a check."""
    cell, url = gui
    calls = {"n": 0}
    real = cell.vision.frame

    def counted(*a, **k):
        calls["n"] += 1
        return real(*a, **k)

    cell.vision.frame = counted
    rec = Recorder(os.path.join(cell.runs_dir, "rec_test"), cell.frames, list(cell.frames.cameras.values()), fps=50, screen=False)
    t0 = time.time()
    rec.start()
    latencies = []
    while time.time() - t0 < 1.0:
        t = time.perf_counter()
        cell.frames.frame("auto_teach_camera")
        latencies.append(time.perf_counter() - t)
        time.sleep(0.01)
    rec.stop()
    cell.vision.frame = real
    assert calls["n"] <= 8, f"the box was asked {calls['n']} times in a second: the poller's 2 Hz x 2 cameras at most"
    assert max(latencies) < 0.02, f"a check waited {max(latencies) * 1000:.1f} ms for a frame while recording"
    assert sum(len(v) for v in rec._index.values()) > 20, "the recorder wrote from the buffer"


def test_done_states_follow_the_data_and_a_record_writes_the_pose(gui):
    cell, url = gui
    assert row(state(url), "slot:A10")["level"] == "bad"
    cell.robot.set_pose_for_test([291.0, 136.9, 410.0, 0, 0, -0.68])
    code, out = post(url, "/api/record", {"row": "slot:A10"})
    assert code == 200 and out["ok"]
    wait_idle(cell)
    L = positions.load(cell.layout_path)
    assert L["fixtures"]["hotel"]["taught"]["A10"] == [291.0, 136.9, 410.0, 0, 0, -0.68]
    assert L["fixtures"]["hotel"]["row_vector_from"] == ["A1", "A10", 9]
    assert row(state(url), "slot:A10")["level"] == "ok"
    assert os.path.isdir(os.path.join(cell.ref_dir, "teach", "slot_A10"))


def test_hover_record_takes_the_grip_pose_from_the_camera(gui):
    cell, url = gui
    cell.robot.set_pose_for_test([120.0, 40.0, 135.0, 0, 0, 0.5])
    post(url, "/api/record", {"row": "hover:microscope"})
    wait_idle(cell)
    fx = positions.load(cell.layout_path)["fixtures"]["microscope"]
    assert fx["pose"][2] == 110.0 and fx["pose_from_camera"] and fx["hover_mm"] == 25.0
    assert row(state(url), "hover:microscope")["done"].startswith("pose from the camera")


def test_bench_edits_go_to_the_positions_file(gui):
    cell, url = gui
    post(url, "/api/loaded", {"slots": ["A1", "C2", "A3"]})
    s = state(url)["test"]["bench"]
    assert s["loaded"] == ["A1", "A3", "C2"] and s["order"] == ["A1", "C2", "A3"]
    post(url, "/api/hotel", {"size": 15})
    s = state(url)["test"]["bench"]
    assert s["size"] == 15 and s["loaded"] == ["A1", "A3"]
    post(url, "/api/hotel", {"size": 30})
    post(url, "/api/loaded", {"slots": ["A1", "C2", "A3"]})


def test_stop_sets_the_stop_and_the_test_ends(gui):
    cell, url = gui
    code, out = post(url, "/api/test")
    assert code == 200 and out["ok"]
    time.sleep(0.5)
    code, out = post(url, "/api/test")
    assert code == 409 and "busy" in out["error"]
    post(url, "/api/stop")
    assert cell.stop_event.is_set()
    wait_idle(cell, 120)
    t = state(url)["test"]["test"]
    assert not t["active"] and t["outcome"].startswith("stopped")


def _events(run_dir):
    with open(os.path.join(run_dir, "events.jsonl"), encoding="utf-8") as f:
        return [json.loads(x) for x in f if x.strip()]


def test_stop_in_the_plate_loop_says_its_cause_and_the_next_test_starts(gui):
    """STOP while plates move: prompt, one-line cause on the page and in events.jsonl, progress shown while it ran.
    (test_the_whole_test_runs_through below is the second Test everything on the same cell.)"""
    cell, url = gui
    assert post(url, "/api/test")[1]["ok"]
    seen = ""
    for _ in range(600):
        t = state(url)["test"]["test"]
        seen = t["progress"] if t else ""
        if seen.startswith("round 1, plate"):
            break
        time.sleep(0.2)
    assert seen.startswith("round 1, plate"), seen
    t0 = time.time()
    post(url, "/api/stop")
    wait_idle(cell, 60)
    assert time.time() - t0 < 20
    s = state(url)
    t = s["test"]["test"]
    assert t["outcome"] == "stopped: STOP pressed" and t["progress"] == t["outcome"]
    assert s["message"]["text"] == "test stopped: STOP pressed"
    assert any(e["kind"] == "run" and e["step"] == "stop" and e["msg"] == t["outcome"]
               for e in _events(cell.checks["test"].run_dir))
    assert cell.run is None


def test_a_fault_ends_the_test_in_one_plain_line(gui, monkeypatch):
    from gui import checks
    cell, url = gui

    def boom(*a):
        raise RuntimeError("the jaw did not open" + chr(10) + "  (stalled)")
    monkeypatch.setattr(checks, "look_at_every_slot", boom)
    assert post(url, "/api/test", {"rounds": 1})[1]["ok"]
    wait_idle(cell, 60)
    t = state(url)["test"]["test"]
    assert t["outcome"] == "stopped: fault: the jaw did not open (stalled)"
    assert state(url)["message"]["text"] == "test stopped: fault: the jaw did not open (stalled)"
    assert any(e["msg"] == t["outcome"] for e in _events(cell.checks["test"].run_dir) if e["step"] == "stop")


def test_record_of_an_unknown_row_is_said_in_words(gui):
    cell, url = gui
    assert post(url, "/api/record", {"row": "nonsense"})[1]["ok"]
    wait_idle(cell)
    assert state(url)["message"]["text"] == "there is no teach row called 'nonsense'"


def test_positions_file_survives_threads_saving_and_loading_at_once():
    import tempfile
    import threading
    path = os.path.join(tempfile.mkdtemp(), "layout.json")
    positions.save({"n": 0}, path)
    errors = []

    def work(i):
        try:
            for k in range(40):
                positions.save({"n": i * 100 + k}, path)
                positions.load(path)
        except Exception as e:
            errors.append(repr(e))
    ts = [threading.Thread(target=work, args=(i,)) for i in range(6)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert not errors, errors[:2]


def test_the_whole_test_runs_through(gui):
    cell, url = gui
    code, out = post(url, "/api/test", {"rounds": 1})        # without rounds it goes on until STOP
    assert code == 200 and out["ok"]
    wait_idle(cell, 300)
    t = state(url)["test"]["test"]
    assert t["outcome"] == "finished", t["log"][-3:]
    texts = [l["text"] for l in t["log"]]
    assert any(x.startswith("slots:") for x in texts) and any(x.startswith("markers:") for x in texts)
    assert any(x.startswith("plates: 3 of 3") for x in texts), texts
    assert t["images"] and all(get(url, "/api/image?p=" + i["path"])[0] == 200 for i in t["images"][:2])


def test_cameras_flow_on_the_fake_package(gui):
    """Read the marker, Capture (the arm drives the plan, one solve per camera), Upload, Check: every row gets its numbers."""
    cell, url = gui
    post(url, "/api/read_marker")
    wait_idle(cell)
    code, out = post(url, "/api/capture", {"hand": False})
    assert code == 200 and out["ok"]
    wait_idle(cell, 120)
    cams = state(url)["cameras"]["cameras"]
    assert all(n["pictures"] >= 6 and n["px"] <= 0.5 and n["mm"] <= 1.0 for n in cams.values()), cams
    assert os.path.exists(os.path.join(cell.ref_dir, "calibration", "auto_teach_camera", "CamInTool.dat"))
    post(url, "/api/upload")
    wait_idle(cell)
    post(url, "/api/check")
    wait_idle(cell)
    cams = state(url)["cameras"]["cameras"]
    assert all(n["uploaded_at"] and n["checked_mm"] <= 1.0 for n in cams.values()), cams


def test_arm_rests_loose_and_a_moving_step_locks_it_then_loosens_again():
    """The arm is hand-guided by default; a button that moves it locks it for the job, then it is loose again."""
    cell = Cell(*sim.build(), sim=True)
    cell.start_up()
    assert cell.robot.state().drag and cell.arm.get("drag")
    assert cell.ready_to_move() is None                      # locked for the step
    assert not cell.robot.state().drag
    seen = []
    assert cell.start("a moving step", lambda: seen.append(cell.robot.state().drag))
    for _ in range(100):
        if not cell.busy:
            break
        time.sleep(0.05)
    assert seen == [False]                                   # locked while it ran
    assert cell.robot.state().drag                           # loose again afterwards
    cell.stop()
