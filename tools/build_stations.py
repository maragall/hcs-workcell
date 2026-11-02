"""
Plans, checks and stores every leg the shuffle drives between stations (routes/graph.py), offline: the arm is not moved. Each leg
is planned from the station's pose with the joints the arm has there (inverse kinematics on the elbow branch the arm is on now),
against the hotel's parts, the pillar and the microscope zone. A pair with no route is listed: that is a leg the cell could not
drive today, found before a run and not during it.

    python tools/build_stations.py [--redo] [--only polar.pose_at_90]
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402
from routes import plan as P  # noqa: E402
from routes_opt import fk  # noqa: E402


def pairs(cyc):
    slots = sorted(s for s in cyc.L["fixtures"]["hotel"]["taught"] if s[1:].isdigit())
    mod = "polar.pose_at_90"
    out = []
    for s in slots:
        out += [("long_edges", mod, f"mouth.{s}"), ("long_edges", f"mouth.{s}", mod),
                (None, mod, f"approach.{s}"), (None, f"approach.{s}", mod)]
    out += [(None, "nest", "polar.pose_at_0"), (None, "polar.pose_at_0", "nest")]
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--redo", action="store_true", help="plan legs that are already stored")
    ap.add_argument("--only", default=None, help="only legs that start or end at this station")
    a = ap.parse_args(argv)
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "stations")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, StateStore(os.path.join(runs, "state.json")), "supervised", runs, R.REFS)
    ja = list(robot.joints())
    graph, sig = P.GRAPH, P.layout_sig(layout)
    print(f"{len(graph.legs)} legs stored; layout {sig[:12]}; both elbow branches", flush=True)
    done = failed = skipped = 0
    bad = []
    for branch, (holding, na, nb) in ((b, q) for q in pairs(cyc) for b in (1, -1)):
        if a.only and a.only not in (na, nb):
            continue
        sa, sb = graph.stations(sig, holding), None
        if na not in sa or nb not in sa:
            continue
        pa, pb = sa[na], sa[nb]
        key = graph.key(sig, na, nb, holding, branch)
        if key in graph.legs and not a.redo:
            skipped += 1
            continue
        try:
            sj = list(fk.ik(pa, elbow=branch, j2_ref=ja[1], j3_ref=ja[2], j4_ref=ja[3]))
        except Exception as e:
            bad.append((branch, holding, na, nb, f"no joints at the start: {e}"))
            failed += 1
            continue
        cyc.safety.joints = lambda sj=sj: sj + list(ja[4:])
        t0 = time.time()
        try:
            route = P.plan(layout, cyc.safety, list(pa), list(pb), None, None, holding=holding)
            done += 1
            print(f"[{branch:+d}] {holding or 'empty':10s} {na} -> {nb}: {len(route.poses)} waypoints, {sum(1 for _ in route.joint_targets)} joint legs, {time.time() - t0:.0f} s", flush=True)
        except Exception as e:
            failed += 1
            bad.append((branch, holding, na, nb, str(e)[:100]))
            print(f"[{branch:+d}] {holding or 'empty':10s} {na} -> {nb}: NO ROUTE ({str(e)[:100]})", flush=True)
    print(f"\n{done} planned, {skipped} already stored, {failed} without a route", flush=True)
    for b in bad:
        print("  no route:", *b, flush=True)
    robot.disconnect()
    tele.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
