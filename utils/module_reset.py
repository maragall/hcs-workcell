"""Undo a plate placed at the 90 deg pose while the module axis was somewhere else (15.56 deg, 2026-11-02): turn the module back to that angle
(the plate returns to where the hand put it), pick the plate by the long edges at the 90 deg pose, turn the empty module to 90, place it.
    python utils/module_reset.py 15.56      (the angle the module read when the plate was put down; the loop must be stopped)"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from run import positions  # noqa: E402
from utils.devices import nova  # noqa: E402
from run.cycle import Cycle  # noqa: E402
from run.state import StateStore  # noqa: E402
from run.planner.safety import SpeedProfile  # noqa: E402
from run.telemetry import Telemetry  # noqa: E402

back = float(sys.argv[1])
layout = positions.load()
runs = os.path.join(positions.RUNS, "module_reset")
tele = Telemetry(runs)
robot, st = nova.connect_readonly(layout, tele.log)
if st.drag:
    robot.drag(False)
robot.enable()
store = StateStore(os.path.join(runs, "state.json"))
cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, positions.REFS)
cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
cyc.corr.ask = lambda q, t=0: "y"
s = store.s
s.plate_at, s.grip, s.step_done, s.hotel = "polar", "none", True, {}
store.save()
F = layout["fixtures"]["polar"]
P0, P90 = F["pose_at_0"]["pose"], F["pose_at_90"]["pose"]
try:
    print("module reads", round(cyc.polar.read_angle(), 2), flush=True)
    robot.jaw_to(layout["gripper"]["open"]["short_edges"])
    cyc._polar_rotate(back, P0)
    print("module back at", round(cyc.polar.read_angle(), 2), flush=True)
    cyc.move_to("polar.pose_at_90")
    cyc._pick("polar", P90, F, "long_edges", None, gate="polar@90")
    print(f"PICKED by the long edges: jaw {robot.jaw():.2f}", flush=True)
    cyc._polar_rotate(90, P90)
    print("module at", round(cyc.polar.read_angle(), 2), flush=True)
    cyc._place("polar", P90, F, "polar", None, gate="polar@90")
    print(f"PLACED at 90: jaw {robot.jaw():.2f}, module {cyc.polar.read_angle():.2f}", flush=True)
finally:
    robot.disconnect()
    tele.close()
