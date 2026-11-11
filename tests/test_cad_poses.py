"""Hotel poses computed from the manufacturer CAD agree with the ones taught by hand to about a millimetre."""
from __future__ import annotations

import copy
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from run.cad_poses import CadPoses  # noqa: E402
from run.hotel import Hotel  # noqa: E402

ROOT = os.path.dirname(HERE)
LAYOUT = json.load(open(os.path.join(ROOT, "layout.json"), encoding="utf-8"))
HARDWARE = json.load(open(os.path.join(ROOT, "hardware.json"), encoding="utf-8"))


def test_the_cad_poses_agree_with_the_taught_hotel_poses_to_a_couple_of_millimetres():
    c = CadPoses(LAYOUT, HARDWARE)
    assert c.rms_mm() < 2.0
    assert max(abs(v) for r in c.compare() for v in r["diff"][:3]) < 3.0
    assert abs(c.fit["theta_deg"]) < 0.5                          # the CAD's axes are the robot's to a fraction of a degree


def test_the_hotel_answers_with_the_taught_pose_unless_the_layout_says_cad():
    class Stub(Hotel):
        def __init__(self, L):
            self.L = L
    L = copy.deepcopy(LAYOUT)
    L["fixtures"]["hotel"]["pose_source"] = "taught"
    taught = Stub(L).hotel_slot_pose("A1")
    assert taught == L["fixtures"]["hotel"]["taught"]["A1"]
    L["fixtures"]["hotel"]["pose_source"] = "cad"
    cad = Stub(L).hotel_slot_pose("A1")
    assert cad != taught and max(abs(x - y) for x, y in zip(cad[:3], taught[:3])) < 3.0
