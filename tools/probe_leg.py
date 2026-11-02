"""
Offline check of one planned leg: from a pose to a pose for a claw holding something, on each elbow branch (the arm is not moved).

    python tools/probe_leg.py polar.pose_at_0 microscope short_edges
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402
from routes import plan as P  # noqa: E402
from routes_opt import fk  # noqa: E402


def main(argv) -> int:
    a, b, holding = argv[0], argv[1], (argv[2] if len(argv) > 2 and argv[2] != "none" else None)
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "probe")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, StateStore(os.path.join(runs, "state.json")), "supervised", runs, R.REFS)
    P.GRAPH = None
    pos = lambda n: list(layout['routes']['nest']['pose']) if n == 'nest' else cyc._position(n)
    pa, pb = pos(a), pos(b)
    ja = list(robot.joints())
    print("arm joints now", [round(v, 1) for v in ja[:4]], "branch", fk.elbow_sign(ja), "pose", [round(v, 1) for v in robot.pose()[:3]])
    cases = [("actual joints", ja[:4])]
    for br in (1, -1):
        cases.append((f"branch {br:+d}", list(fk.ik(pa, elbow=br))))
    for name, sj in cases:
        cyc.safety.joints = lambda sj=sj: list(sj) + list(ja[4:])
        P._ROUTES.clear()
        try:
            r = P.plan(layout, cyc.safety, list(pa), list(pb), None, None, holding=holding)
            print(f"{name:14s} -> route {r.name}, {len(r.poses)} waypoints, joint legs {sorted(r.joint_targets)}")
        except Exception as e:
            print(f"{name:14s} -> NO ROUTE: {str(e)[:150]}")
    robot.disconnect()
    tele.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
