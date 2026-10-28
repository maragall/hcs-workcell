"""The offline rig shared by the tests: fake vision box, synthetic frames, a layout patched for the simulator."""
from __future__ import annotations

import io
import json
import os
import time

import numpy as np
from PIL import Image

from devices.vision_box import ArucoPose, PoseOffset, Status

HERE = os.path.dirname(os.path.abspath(__file__))
SAFE, BAD = 1, 99
GATES = [("microscope", "plate_present"), ("microscope", "empty"), ("polar@0", "plate_present"), ("polar@0", "empty"),
         ("polar@90", "plate_present"), ("polar@90", "empty"), ("gripper", "short_edges"), ("gripper", "long_edges")]


def jpeg(seed: int, noise: float = 3.0) -> bytes:
    """A synthetic camera frame; seed BAD draws a different scene so the correlation gate fails."""
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


class FakeClock:
    """time() and sleep() for Cycle: sleep advances the clock instead of waiting, so the fixed settle
    waits of the procedures cost nothing on the simulator. Starts at the real time."""

    def __init__(self):
        self.now = time.time()
        self.slept = 0.0

    def time(self) -> float:
        return self.now

    def sleep(self, s: float) -> None:
        self.now += s
        self.slept += s


class FakeVision:
    def __init__(self):
        self.scene = {"auto_teach_camera": SAFE, "scan_camera": SAFE}
        self.offset = [1.0, -0.5, 0, 0, 0, 0]
        self.offset_error = None
        self.slot_occupied = False
        self.status_error = None
        self.frame_error = None          # raised by frame(): the camera gives nothing
        self.live_error = None           # raised by live_detection()
        self.on_status = None

    def status(self):
        if self.on_status:
            self.on_status()
        if self.status_error:
            raise self.status_error
        return Status("idle", 1000, 2, 1, 0, 0)

    def frame(self, cam, *a, **k):
        if self.frame_error:
            raise self.frame_error
        return jpeg(self.scene[cam])

    def pose_offset(self, *a, **k):
        if self.offset_error:
            raise self.offset_error
        d = list(self.offset)
        return PoseOffset(d, "robot", (d[0] ** 2 + d[1] ** 2) ** 0.5, 0.0, jpeg(2))

    def live_detection(self, *a, **k):
        if self.live_error:
            raise self.live_error
        return self.slot_occupied, jpeg(3)

    def scan(self, *a, **k):
        return ["PLATE-0001"], jpeg(4)

    # the teaching side of the box, as the GUI uses it
    def version(self):
        return "fake vision box"

    def aruco_pose(self, *a, **k):
        return ArucoPose([640.0, 360.0, 0.0, 0.0], [0.0] * 6, [0.0] * 6, "robot", jpeg(6))

    def set_reference(self, *a, **k):
        return []

    def positions(self, *a, **k):
        return []

    def add_teach(self, *a, **k):
        return []

    def add_trigger(self, *a, **k):
        return []

    def add_relation(self, *a, **k):
        return []


def layout() -> dict:
    """layout.json with the taught values replaced by ones the simulator can reach."""
    with open(os.path.join(os.path.dirname(HERE), "layout.json"), "r", encoding="utf-8") as f:
        L = json.load(f)
    L["fixtures"]["microscope"]["pose"] = [120.0, 40.0, 110.0, 0, 0, 0.5]
    L["fixtures"]["microscope"]["hover_mm"] = 25
    L["regions"]["microscope"] = {"box": {"x": [100, 140], "y": [20, 60]}, "z_floor": 108, "no_swing": True,
                                  "max_lateral_mm": 2.0, "max_yaw_change_deg": 0.5, "needs_parked": True, "free_above_z": 300}
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
    for fx in ("microscope", "polar"):
        L["fixtures"][fx].pop("grasp_width", None)           # learned on the real cell; the simulator uses the nominal widths
    L["gripper"]["open"] = {"short_edges": 10.0, "long_edges": 10.0}
    L["polar"]["ext_axis"]["direction"] = -1
    L["routes"] = {"safe_z_mm": 160.0, "positions": ["microscope", "polar.pose_at_0", "polar.pose_at_90"],
                   "nest": {"pose": [150.0, 80.0, 150.0, 0, 0, 0]}}
    L["volumes"] = None                                         # the CAD volumes and poses are in the real cell's frame; the fakes' world is their own
    L.pop("setup", None)                                        # what the real cell's setup found (tag, offsets, hotel size) is not the rig's
    L["fixtures"]["hotel"]["pose_source"] = "taught"
    L["fixtures"]["hotel"]["rows"] = list(range(1, 11))         # the rig's bench is its own, not whatever the live file says
    L["fixtures"]["hotel"]["loaded"] = {"A": [1, 3, 5, 7, 9], "C": [2, 4, 6, 8]}   # row 10 extrapolates above the rig's box
    return L
