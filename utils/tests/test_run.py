"""The asyncio main loop on fakes: two microscopes, one arm. No arm, no Pi, no Squid."""
from __future__ import annotations

import asyncio
import json
import os
import shutil
import sys
import tempfile

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, HERE)

from rig import FakeClock, FakeVision, layout  # noqa: E402

from run import loop as run  # noqa: E402
from utils.devices.fake_arm import SimFaults, SimRobot  # noqa: E402
from utils.devices.fake_squid import FakeSquid  # noqa: E402
from vision.corrector import StopSafe  # noqa: E402
from run.telemetry import Telemetry  # noqa: E402

A, B = "microscope", "microscope_2"
SLOTS = {A: "A1", B: "A3"}


def make_cell(acquisition_s=0.2):
    tmp = tempfile.mkdtemp()
    L = layout()
    run.borrow_fixture(L, B)
    ref_dir = os.path.join(tmp, "refs")
    tele = Telemetry(os.path.join(tmp, "run"))
    start = list(L["fixtures"][A]["pose"])
    start[2] += L["fixtures"][A]["hover_mm"]
    robot = SimRobot(start, SimFaults(), tele.log, jaw=10.0)
    robot.dis = {"P0": 1, "P90": 1}
    robot._ext[1] = L["polar"]["ext_axis"]["zero_reading"]
    for wp in L["trajectories"].values():
        for j, pose in zip(wp["joints"], wp["poses"]):
            robot.joint_pose_map[tuple(round(v, 3) for v in j[:4])] = pose
    scopes = {n: FakeSquid(n, acquisition_s, tele.log) for n in (A, B)}
    cell = run.Workcell(L, robot, FakeVision(), scopes, tele, "unsupervised", os.path.join(tmp, "run"), ref_dir, tmp,
                        poll_s=0.01, clock=FakeClock())
    return cell, tmp


def events(tmp, *parts):
    with open(os.path.join(tmp, "run", *parts, "events.jsonl"), encoding="utf-8") as f:
        return [json.loads(line) for line in f]


def close(cell, tmp):
    cell.close()
    cell.tele.close()
    shutil.rmtree(tmp, ignore_errors=True)


def reopen(cell, tmp):
    """A second Workcell on the same robot, ledgers and run folder: what `python -m run --resume` builds."""
    a = cell.cycles[A]
    cell.close()
    return run.Workcell(a.L, a.robot, a.vision, cell.scopes, cell.tele, "unsupervised", os.path.join(tmp, "run"),
                        os.path.join(tmp, "refs"), tmp, poll_s=0.01)


def test_a_fault_in_one_task_stops_the_other_before_it_moves():
    cell, tmp = make_cell()
    try:
        cell.cycles[A].store.s.plate_at = "polar"            # A cannot resume from here: StopSafe at its first step
        cell.cycles[A].store.save()
        with pytest.raises(StopSafe, match="no automatic resume"):
            asyncio.run(cell.run(SLOTS, cycles=2))
        assert isinstance(cell.fault, StopSafe) and cell.stop.is_set()
        assert cell.scopes[B].acquisitions == 0               # B was queued on the arm and never took it
        assert not any(e["msg"].startswith(B) for e in events(tmp) if e["step"] == "arm")
    finally:
        close(cell, tmp)


def test_a_stage_that_never_parks_keeps_the_arm_out_and_stops():
    cell, tmp = make_cell()
    cell.scopes[A].stage_parked = False
    cell.park_timeout_s = 0.05
    try:
        with pytest.raises(StopSafe, match=f"{A}: stage not parked .* the arm stays out"):
            asyncio.run(cell.run(SLOTS, cycles=1))
        assert cell.cycles[A].robot._moves == 0               # the shared arm never moved, for either microscope
        assert not any(e["step"] == "arm" for e in events(tmp))
        assert all(cell.scopes[n].acquisitions == 0 for n in (A, B))
    finally:
        close(cell, tmp)
