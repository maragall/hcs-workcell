"""
Offline tests of the cycle against the simulator and a fake vision box. No arm, no Pi.
Each test maps to a row of DESIGN.md section 6.
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
from nova_cell.safety import SafetyViolation  # noqa: E402
from nova_cell.state import StateStore  # noqa: E402
from nova_cell.telemetry import Telemetry  # noqa: E402
from nova_cell.vision import PoseOffset, Status, VisionError  # noqa: E402

SAFE, BAD = 1, 99
GATES = [("microscope", "plate_present"), ("microscope", "empty"), ("polar@0", "plate_present"), ("polar@0", "empty"),
         ("polar@90", "plate_present"), ("polar@90", "empty"), ("gripper", "short_edges"), ("gripper", "long_edges")]


def jpeg(seed: int, noise: float = 3.0) -> bytes:
    rng = np.random.default_rng(seed)
    base = np.zeros((120, 160), np.uint8)
    if seed == BAD:
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
        self.scene = {"auto_teach_camera": SAFE, "scan_camera": SAFE}
        self.offset = [1.0, -0.5, 0, 0, 0, 0]
        self.offset_error = None
        self.slot_occupied = False
        self.status_error = None
        self.on_status = None

    def status(self):
        if self.on_status:
            self.on_status()
        if self.status_error:
            raise self.status_error
        return Status("idle", 1000, 2, 1, 0, 0)

    def frame(self, cam, *a, **k):
        return jpeg(self.scene[cam])

    def pose_offset(self, *a, **k):
        if self.offset_error:
            raise self.offset_error
        d = list(self.offset)
        return PoseOffset(d, "robot", (d[0] ** 2 + d[1] ** 2) ** 0.5, 0.0, jpeg(2))

    def live_detection(self, *a, **k):
        return self.slot_occupied, jpeg(3)

    def scan(self, *a, **k):
        return ["PLATE-0001"], jpeg(4)


def layout():
    with open(os.path.join(os.path.dirname(HERE), "layout.json"), "r", encoding="utf-8") as f:
        L = json.load(f)
    L["fixtures"]["microscope"]["pose"] = [120.0, 40.0, 110.0, 0, 0, 0.5]
    L["fixtures"]["microscope"]["hover_mm"] = 25
    L["regions"]["microscope"] = {"box": {"x": [100, 140], "y": [20, 60]}, "z_floor": 108, "no_swing": True,
                                  "max_lateral_mm": 2.0, "max_yaw_change_deg": 0.5}
    L["fixtures"]["polar"]["pose_at_0"]["pose"] = [125.4, 117.4, 67.9, 0, 0, 1.5632]
    scope_hover = [120.0, 40.0, 135.0, 0, 0, 0.5]
    wp_mp = {"joints": [[10, 0, 0, 0, 0, 0, 0, 0], [20, 0, 0, 0, 0, 0, 0, 0]],
             "poses": [scope_hover, [180, 60, 150, 0, 0, 0]], "spread_deg": 0, "max_step_deg": 10, "demos": 1}
    wp_ph = {"joints": [[30, 0, 0, 0, 0, 0, 0, 0], [40, 0, 0, 0, 0, 0, 0, 0]],
             "poses": [[150, 80, 150, 0, 0, 0], [180, 60, 150, 0, 0, 0]], "spread_deg": 0, "max_step_deg": 10, "demos": 1}
    L["trajectories"] = {"microscope_to_polar": wp_mp, "polar_to_hotel": wp_ph}
    L["polar"]["driver"] = "ext_axis"
    L["polar"]["ext_axis"]["zero_reading"] = 84.5
    L["limits"]["health_check_every_s"] = 0
    L["limits"]["min_disk_free_gb"] = 0
    L["gripper"]["closed"] = {"short_edges": 2.92, "long_edges": 2.92}
    L["gripper"]["open"] = {"short_edges": 10.0, "long_edges": 10.0}
    L["polar"]["ext_axis"]["direction"] = -1
    return L


class Rig:
    def __init__(self, faults=None, refs=GATES, mode="unsupervised"):
        self.tmp = tempfile.mkdtemp()
        self.L = layout()
        ref_dir = os.path.join(self.tmp, "refs")
        lc = LowerCheck(ref_dir)
        for gate, state in refs:
            for _ in range(3):
                lc.add_reference(gate, state, jpeg(SAFE))
        self.tele = Telemetry(os.path.join(self.tmp, "run"))
        self.store = StateStore(os.path.join(self.tmp, "state.json"))
        start = list(self.L["fixtures"]["microscope"]["pose"]); start[2] += self.L["fixtures"]["microscope"]["hover_mm"]
        self.robot = SimRobot(start, faults or SimFaults(), self.tele.log, jaw=10.0)
        self.robot.dis = {"P0": 1, "P90": 1}
        for wp in self.L["trajectories"].values():
            for j, pose in zip(wp["joints"], wp["poses"]):
                self.robot.joint_pose_map[tuple(round(v, 3) for v in j[:4])] = pose
        self.vis = FakeVision()
        self.cyc = Cycle(self.L, self.robot, self.vis, self.tele, self.store, mode, os.path.join(self.tmp, "run"), ref_dir)

    def run(self, slot="A1", cycles=1):
        return self.cyc.run(slot, cycles)

    def expect_stop(self, text, slot="A1"):
        try:
            self.run(slot)
        except (StopSafe, SafetyViolation) as e:
            assert text in str(e), f"expected '{text}' in '{e}'"
            return e
        raise AssertionError(f"expected a safe stop containing '{text}'")

    def events(self):
        with open(os.path.join(self.tmp, "run", "events.jsonl"), encoding="utf-8") as f:
            return [json.loads(line) for line in f]

    def close(self):
        self.tele.close()
        shutil.rmtree(self.tmp, ignore_errors=True)


def test_full_cycle():
    r = Rig()
    r.run()
    s = r.store.s
    assert s.plate_at == "microscope" and s.step_done and s.polar_angle == 0
    kinds = {e["kind"] for e in r.events()}
    assert {"vision", "jaw", "cycle", "move", "health"} <= kinds
    for kind, j in r.robot.trace:
        if kind == "AJ":
            assert j[4:] == r.robot.joints()[4:], "waypoint replay drove an external axis"
    r.close()


def test_refused_move_stops_without_retry():
    r = Rig(SimFaults(refuse_move_at=5))
    r.expect_stop("refus")
    assert not r.store.s.step_done
    assert r.robot._moves == 5
    r.close()


def test_missed_grasp_retries_once_then_ok():
    r = Rig(SimFaults(miss_grasp_at=1))
    r.run()
    assert any(f["kind"] == "grasp" for f in r.store.s.faults)
    assert r.store.s.plate_at == "microscope"
    r.close()


def test_obstructed_grasp_twice_stops():
    r = Rig()
    orig = r.robot.jaw_to

    def jaw_to(v):
        orig(v)
        if v <= 3.0:
            r.robot._jaw = 3.6
    r.robot.jaw_to = jaw_to
    r.expect_stop("grasp failed")
    assert sum(1 for f in r.store.s.faults if f["kind"] == "grasp") == 2
    r.close()


def test_lower_blocked_stops():
    r = Rig()
    r.vis.scene["auto_teach_camera"] = BAD
    r.expect_stop("does not match")
    assert r.store.s.plate_at == "microscope" and r.store.s.grip == "none"
    r.close()


def test_lower_gate_never_falls_through():
    r = Rig()
    r.L["vision"]["lower_check"]["retries"] = 0
    r.vis.scene["auto_teach_camera"] = BAD
    r.expect_stop("does not match")
    r.close()


def test_plate_not_held_stops():
    r = Rig()
    r.vis.scene["scan_camera"] = BAD
    r.expect_stop("plate in gripper")
    assert any(f["kind"] == "not_held" for f in r.store.s.faults)
    r.close()


def test_fixture_moved_stops():
    r = Rig()
    r.vis.offset = [30.0, 0, 0, 0, 0, 0]
    r.expect_stop("exceeds")
    r.close()


def test_aruco_missing_stops_unsupervised():
    r = Rig()
    r.vis.offset_error = VisionError("1002", "Operation Failed", "", "get_pose_offset")
    r.expect_stop("get_pose_offset")
    r.close()


def test_stall_is_detected():
    r = Rig(SimFaults(stall_at=3))
    r.expect_stop("possible collision")
    assert r.store.s.faults[-1]["kind"] == "pose_deviation"
    r.close()


def test_lost_enable_stops_without_motion_retry():
    r = Rig(SimFaults(drop_enable_at=3))
    r.expect_stop("not enabled")
    assert r.robot._moves == 3
    r.close()


def test_polar_axis_lands_off_target_retries_once_then_stops():
    r = Rig(SimFaults(ext_error=2.0))
    r.expect_stop("polar")
    assert sum(1 for f in r.store.s.faults if f["kind"] == "rotate") == 2
    r.close()


def test_polar_axis_is_commanded_from_zero_reference():
    r = Rig()
    r.run()
    assert abs(r.robot.ext_axis(1) - 84.5) < 1e-6      # back at 0 deg = zero reading after the inbound leg
    r.close()


def test_hotel_slot_occupied_stops():
    r = Rig()
    r.vis.slot_occupied = True
    r.expect_stop("occupied")
    r.close()


def test_vision_box_down_stops():
    r = Rig()
    r.vis.status_error = OSError("unreachable")
    r.expect_stop("vision box unreachable")
    r.close()


def test_drag_on_mid_run_stops():
    r = Rig()
    calls = {"n": 0}

    def flip():
        calls["n"] += 1
        if calls["n"] == 3:
            r.robot.faults.drag_on = True
    r.vis.on_status = flip
    r.expect_stop("drag mode")
    r.close()


def test_joint_move_inside_microscope_zone_refused():
    r = Rig()
    r.robot.set_pose_for_test([160.0, 40.0, 140.0, 0, 0, 0.5])      # outside the zone, 40 mm from the hover
    r.L["trajectories"]["microscope_to_polar"]["poses"][0] = [160.0, 40.0, 140.0, 0, 0, 0.5]
    r.robot.joint_pose_map[(10.0, 0.0, 0.0, 0.0)] = [160.0, 40.0, 140.0, 0, 0, 0.5]
    r.store.s.plate_at = "hotel:A1"
    r.store.s.hotel["A1"] = ""
    r.store.s.last_safe_pose = None
    r.store.save()
    r.expect_stop("no-swing zone")
    r.close()


def test_invalid_slot_rejected():
    r = Rig()
    r.expect_stop("does not exist", slot="A2")
    assert r.robot._moves == 0
    r.close()


def test_resume_refuses_half_step():
    r = Rig()
    r.store.begin("pick:microscope")
    r.expect_stop("did not finish")
    r.close()


def test_resume_refuses_unknown_plate_location():
    r = Rig()
    r.store.s.plate_at = "polar"
    r.store.save()
    r.expect_stop("no automatic resume")
    assert r.robot._moves == 0
    r.close()


def test_resume_refuses_moved_arm():
    r = Rig()
    r.store.s.last_safe_pose = [300.0, 0.0, 300.0, 0, 0, 0]
    r.store.save()
    r.expect_stop("jog it back")
    r.close()


def test_drift_warning_is_logged():
    r = Rig()
    for _ in range(10):
        r.cyc.drift.observe("jaw:closed:short_edges", 2.92)
    orig = r.robot.jaw_to

    def jaw_to(v):
        orig(v)
        if v <= 3.0:
            r.robot._jaw = 3.2
    r.robot.jaw_to = jaw_to
    r.run()
    assert any(e["kind"] == "predict" for e in r.events())
    r.close()


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok ", t.__name__)
    print(f"ALL OK ({len(tests)} tests)")
