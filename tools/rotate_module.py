"""Turn the rotation module to an angle, the claw empty: the jaw opens, the hand rises to the module's clear height, the module turns.
    python tools/rotate_module.py 0       (the loop must be stopped)"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from run.cycle import Cycle  # noqa: E402
from run.state import StateStore  # noqa: E402
from run.safety import SpeedProfile  # noqa: E402
from run.telemetry import Telemetry  # noqa: E402

deg = int(sys.argv[1])
layout = R.load_layout()
runs = os.path.join(R.RUNS, "rotate_module")
tele = Telemetry(runs)
robot, st = R.connect_robot_readonly(layout, tele.log)
if st.drag:
    robot.drag(False)
robot.enable()
cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, StateStore(os.path.join(runs, "state.json")), "supervised", runs, R.REFS)
cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
cyc.corr.ask = lambda q, t=0: "y"
F = layout["fixtures"]["polar"]
try:
    print("module reads", round(cyc.polar.read_angle(), 2), flush=True)
    robot.jaw_to(layout["gripper"]["open"]["short_edges"])
    here = robot.pose()
    P = F["pose_at_0"]["pose"]
    cyc._polar_rotate(deg, P)
    print("module reads", round(cyc.polar.read_angle(), 2), flush=True)
finally:
    robot.disconnect()
    tele.close()
