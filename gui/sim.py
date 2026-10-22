"""--sim: the fakes the tests use, so a developer can click through without hardware. The positions file is a
patched copy in a temp folder (tests/rig.layout), the references are seeded as exercise.py seeds them."""
from __future__ import annotations

import os
import sys
import tempfile

from devices import nova
from perception.lower_check import LowerCheck
from procedures import positions
from procedures.motion import offset

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rig():
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    import rig as r
    return r


def build():
    """(layout_path, ref_dir, state_dir, runs_dir, robot, vision) on the fakes."""
    r = rig()
    tmp = tempfile.mkdtemp(prefix="gui_sim_")
    L = r.layout()
    layout_path = os.path.join(tmp, "layout.json")
    positions.save(L, layout_path)
    ref_dir = os.path.join(tmp, "reference_images")
    lc = LowerCheck(ref_dir)
    for gate, state in r.GATES:
        for _ in range(3):
            lc.add_reference(gate, state, r.jpeg(r.SAFE))
    robot = nova.arm(L, sim=True)
    robot.follow_layout(L)
    m = L["fixtures"]["microscope"]
    robot.set_pose_for_test(offset(m["pose"], [0, 0, m["hover_mm"]]))      # the cycle starts at the hover
    return layout_path, ref_dir, os.path.join(tmp, "state"), os.path.join(tmp, "runs"), robot, r.FakeVision()
