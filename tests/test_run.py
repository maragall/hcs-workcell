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
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from rig import GATES, SAFE, FakeClock, FakeVision, jpeg, layout  # noqa: E402

import run  # noqa: E402
from devices.fake_arm import SimFaults, SimRobot  # noqa: E402
from devices.fake_squid import FakeSquid  # noqa: E402
from perception.lower_check import LowerCheck  # noqa: E402
from procedures.corrector import StopSafe  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402

A, B = "microscope", "microscope_2"
SLOTS = {A: "A1", B: "A3"}


def make_cell(acquisition_s=0.2):
    tmp = tempfile.mkdtemp()
    L = layout()
    run.borrow_fixture(L, B)
    ref_dir = os.path.join(tmp, "refs")
    lc = LowerCheck(ref_dir)
    for gate, state in GATES + [(B, "plate_present"), (B, "empty")]:
        for _ in range(3):
            lc.add_reference(gate, state, jpeg(SAFE))
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


def test_two_microscopes_share_the_arm_and_image_while_the_other_moves():
    cell, tmp = make_cell()
    try:
        assert cell.plates() == {A: A, B: B}                 # new ledgers: every plate on its microscope
        asyncio.run(cell.run(SLOTS, cycles=1))
        assert cell.plates() == {A: A, B: B}
        assert all(cell.scopes[n].acquisitions == 1 for n in (A, B))
        msgs = [e["msg"] for e in events(tmp) if e["step"] in ("arm", "scope")]
        arm = [m for m in msgs if "the arm" in m]
        assert arm == [f"{A} takes the arm", f"{A} returns the arm", f"{B} takes the arm", f"{B} returns the arm"]
        # A images while B has the arm: that is the point of the loop. A's acquisition starts the moment it returns
        # the arm and is still running when B takes it (a fake plate change is quicker than the 0.2 s fake acquisition)
        done_a = next(i for i, m in enumerate(msgs) if m.startswith(f"{A}: acquisition done"))
        assert msgs.index(f"{B} takes the arm") < done_a
        for n in (A, B):                                      # each microscope ran its own route, in its own folder
            steps = {e["step"] for e in events(tmp, n)}
            assert {f"pick:{n} lower", f"place:{n} lower", f"place:hotel:{SLOTS[n]} lower"} <= steps
    finally:
        close(cell, tmp)


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
