"""
Offline test of the cycle against the simulator and a fake vision box.
Runs without the arm or the Pi.  Exercises: the full outbound and inbound cycle, state persistence,
a refused move, a missed grasp with one retry, and a lower-check block.
"""
from __future__ import annotations

import io
import json
import os
import shutil
import sys
import tempfile

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from nova_cell.corrector import StopSafe  # noqa: E402
from nova_cell.cycle import Cycle  # noqa: E402
from nova_cell.lower_check import LowerCheck  # noqa: E402
from nova_cell.robot import SimFaults, SimRobot  # noqa: E402
from nova_cell.state import StateStore  # noqa: E402
from nova_cell.telemetry import Telemetry  # noqa: E402
from nova_cell.vision import PoseOffset, Status  # noqa: E402


def jpeg(seed: int, noise: float = 0.0) -> bytes:
    rng = np.random.default_rng(seed)
    base = np.zeros((120, 160), np.uint8)
    if seed == 99:
        base[5:40, 100:155] = 230
        base[70:115, 5:60] = 120
    else:
        base[30:90, 40:120] = 200
        base[50:70, 60:100] = 40
    img = base.astype(np.float32) + rng.normal(0, noise, base.shape)
    im = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).resize((640, 480))
    b = io.BytesIO()
    im.save(b, "JPEG", quality=90)
    return b.getvalue()


class FakeVision:
    def __init__(self):
        self.scene = "safe"
        self.offset = [1.0, -0.5, 0, 0, 0, 0]

    def status(self):
        return Status("idle", 1000, 2, 1, 0, 0)

    def frame(self, cam, *a, **k):
        return jpeg(1, 3.0) if self.scene == "safe" else jpeg(99, 60.0)

    def pose_offset(self, *a, **k):
        d = list(self.offset)
        return PoseOffset(d, "robot", (d[0] ** 2 + d[1] ** 2) ** 0.5, 0.0, jpeg(2))

    def live_detection(self, *a, **k):
        return False, jpeg(3)

    def scan(self, *a, **k):
        return ["PLATE-0001"], jpeg(4)


def layout(tmp):
    with open(os.path.join(os.path.dirname(HERE), "layout.json"), "r", encoding="utf-8") as f:
        L = json.load(f)
    L["fixtures"]["microscope"]["pose"] = [120.0, 40.0, 110.0, 0, 0, 0.5]
    L["regions"]["microscope"] = {"box": {"x": [100, 140], "y": [20, 60]}, "z_floor": 108}
    L["fixtures"]["polar"]["pose_at_0"]["pose"] = [125.4, 117.4, 67.9, 0, 0, 1.5632]
    wp = {"joints": [[10, 0, 0, 0, 0, 0, 0, 0], [20, 0, 0, 0, 0, 0, 0, 0]],
          "poses": [[150, 80, 150, 0, 0, 0], [180, 60, 150, 0, 0, 0]], "spread_deg": 0, "max_step_deg": 10, "demos": 1}
    L["trajectories"] = {"microscope_to_polar": wp, "polar_to_hotel": wp}
    L["polar"]["driver"] = "do"
    L["limits"]["health_check_every_s"] = 0
    return L


def make(tmp, faults=None, scene="safe"):
    L = layout(tmp)
    refs = os.path.join(tmp, "refs")
    lc = LowerCheck(refs)
    for fx in ("microscope", "polar"):
        for st in ("plate_present", "empty"):
            for k in range(3):
                lc.add_reference(fx, st, jpeg(1, 3.0))
    tele = Telemetry(os.path.join(tmp, "run"))
    store = StateStore(os.path.join(tmp, "state.json"))
    robot = SimRobot(L["fixtures"]["microscope"]["pose"], faults or SimFaults(), tele.log, jaw=10.0)
    vis = FakeVision()
    vis.scene = scene
    return Cycle(L, robot, vis, tele, store, "unsupervised", os.path.join(tmp, "run"), refs), robot, store, vis


def test_full_cycle():
    tmp = tempfile.mkdtemp()
    cyc, robot, store, vis = make(tmp)
    cyc.run("A1", 1)
    assert store.s.plate_at == "microscope", store.s.plate_at
    assert store.s.step_done
    assert store.s.polar_angle == 0
    assert robot.dos.get("POLAR_ROT") == 0 and robot.dos.get("POLAR_HOME") == 0
    with open(os.path.join(tmp, "run", "events.jsonl")) as f:
        kinds = [json.loads(l)["kind"] for l in f]
    assert "vision" in kinds and "jaw" in kinds and "cycle" in kinds
    cyc.tele.close()
    shutil.rmtree(tmp, ignore_errors=True)
    print("full cycle ok")


def test_refused_move_stops_safe():
    tmp = tempfile.mkdtemp()
    cyc, robot, store, vis = make(tmp, SimFaults(refuse_move_at=5))
    try:
        cyc.run("A1", 1)
        raise AssertionError("expected StopSafe")
    except StopSafe:
        pass
    assert not store.s.step_done
    assert store.s.faults and store.s.faults[-1]["kind"] == "motion_refused"
    cyc.tele.close()
    shutil.rmtree(tmp, ignore_errors=True)
    print("refused move -> safe stop ok")


def test_missed_grasp_retries_once():
    tmp = tempfile.mkdtemp()
    cyc, robot, store, vis = make(tmp, SimFaults(miss_grasp_at=1))
    cyc.run("A1", 1)
    assert any(f["kind"] == "grasp" for f in store.s.faults)
    assert store.s.plate_at == "microscope"
    cyc.tele.close()
    shutil.rmtree(tmp, ignore_errors=True)
    print("missed grasp -> retry ok")


def test_lower_blocked_unsupervised_stops():
    tmp = tempfile.mkdtemp()
    cyc, robot, store, vis = make(tmp, scene="bad")
    try:
        cyc.run("A1", 1)
        raise AssertionError("expected StopSafe")
    except StopSafe as e:
        assert "does not match" in str(e) or "lower blocked" in str(e)
    assert store.s.plate_at == "microscope" and store.s.grip == "none"
    cyc.tele.close()
    shutil.rmtree(tmp, ignore_errors=True)
    print("lower-check block -> safe stop ok")


def test_resume_refuses_half_step():
    tmp = tempfile.mkdtemp()
    cyc, robot, store, vis = make(tmp)
    store.begin("pick:microscope")
    try:
        cyc.run("A1", 1)
        raise AssertionError("expected StopSafe")
    except StopSafe as e:
        assert "did not finish" in str(e)
    cyc.tele.close()
    shutil.rmtree(tmp, ignore_errors=True)
    print("half-finished step -> refuses to resume ok")


if __name__ == "__main__":
    test_full_cycle()
    test_refused_move_stops_safe()
    test_missed_grasp_retries_once()
    test_lower_blocked_unsupervised_stops()
    test_resume_refuses_half_step()
    print("ALL OK")
