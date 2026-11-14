"""The offline rig shared by the tests: a clock that does not wait, and a layout patched for the simulator."""
from __future__ import annotations

import json
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))


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


def layout() -> dict:
    """layout.json with the taught values replaced by ones the simulator can reach."""
    with open(os.path.join(os.path.dirname(os.path.dirname(HERE)), "run", "layout.json"), "r", encoding="utf-8") as f:
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
