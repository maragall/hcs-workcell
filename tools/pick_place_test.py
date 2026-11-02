"""One real pick and place of the plate on the rotation module at 0 deg through the production code (short edges), jaw readings printed.
The loop must be stopped.    python tools/pick_place_test.py"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.safety import SpeedProfile  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402

layout = R.load_layout()
runs = os.path.join(R.RUNS, "pick_place_test")
tele = Telemetry(runs)
robot, st = R.connect_robot_readonly(layout, tele.log)
if st.drag:
    robot.drag(False)
robot.enable()
store = StateStore(os.path.join(runs, "state.json"))
cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, R.REFS)
cyc.safety.profile = SpeedProfile(**layout["speeds"]["supervised"])
cyc.corr.ask = lambda q, t=0: "y"
s = store.s
s.plate_at, s.grip, s.step_done, s.hotel = "polar", "none", True, {}
store.save()
F = layout["fixtures"]["polar"]
P0 = F["pose_at_0"]["pose"]
try:
    cyc._pick("polar", P0, F, "short_edges", None, gate="polar@0")
    print(f"PICKED: jaw {robot.jaw():.2f}, band {layout['gripper']['accept_band']['short_edges']}, pose z {robot.pose()[2]:.1f}", flush=True)
    cyc._place("polar", P0, F, "polar", None, gate="polar@0")
    print(f"PLACED BACK: jaw {robot.jaw():.2f}", flush=True)
finally:
    robot.disconnect()
    tele.close()
