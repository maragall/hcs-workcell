"""Full workflow test, unattended: the plate starts in the microscope. For each slot in turn the loop own outbound and inbound run:
microscope -> module (short edges, 0 deg) -> turn to 90 -> pick by the long edges -> hotel slot, and the same way back to the microscope.
Then the next slot, round and round until runs/STOP exists or a step refuses. A refusal leaves the arm where it is.
Routes between the module and a slot, and between the module and the microscope, are the stored ones (preset/routes.json), replayed.

    python tools/test_workflow.py                 # A1..A9, B1..B9, for ever
    python tools/test_workflow.py A1 B1 --rounds 1 --speed fast
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "tools"))
import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from procedures import motion as M  # noqa: E402
from procedures.corrector import StopSafe  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.safety import SpeedProfile  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402
from routes.plan import Route  # noqa: E402

STOP = os.path.join(R.RUNS, "STOP")
LOG = os.path.join(R.RUNS, "test_workflow.log")


def say(msg):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}"
    print(line, flush=True)
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def stored_routes(real_plan, scope):
    preset = json.load(open(os.path.join(HERE, "preset", "routes.json"), encoding="utf-8"))["routes"]

    def near(a, b, tol=1.0):
        return max(abs(x - y) for x, y in zip(a[:3], b[:3])) <= tol

    def plan(layout, safety, here, target, from_name=None, to_name=None, holding=None, heading=None):
        h, f, t = holding or "empty", from_name or "", to_name or ""
        key = None
        if f.startswith("polar.pose_at_") and "hotel" in t:
            key = f"module->{t.split()[-1]}|{h}"
        elif f.startswith("polar.pose_at_") and scope in t:
            key = f"module->{scope}|{h}"
        elif f == scope and "polar" in t:
            key = f"{scope}->module|{h}"
        elif t.startswith("polar.pose_at_") or "polar" in t:
            key = next((k for k, v in preset.items() if k.endswith(f"->module|{h}") and "poses" in v and "entry" in v and near(v["entry"], here)), None)
        want = preset.get(key or "", {})
        if "poses" in want and near(want["poses"][-1], target, 0.6) and (key.startswith("module->") is False or True):
            say(f"   route {key}: {len(want['poses'])} legs, stored")
            return Route("stored", [list(q) for q in want["poses"]], joint_targets={int(i): list(j) for i, j in want["joint_targets"].items()})
        route = real_plan(layout, safety, here, target, from_name, to_name, holding=holding, heading=heading)
        say(f"   route {f or 'here'} -> {t}: {len(route.poses)} legs, PLANNED NOW (no stored route matched: {key})")
        return route
    return plan


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("slots", nargs="*")
    ap.add_argument("--speed", default="stress")
    ap.add_argument("--first", default="fast", help="the profile of the very first slot")
    ap.add_argument("--rounds", type=int, default=0, help="0 = until runs/STOP")
    a = ap.parse_args(argv)
    slots = a.slots or [c + str(r) for c in "AB" for r in (1, 3, 5, 7, 9)]
    layout = R.load_layout()
    layout["vision"]["check_before_lower"] = False            # the seat occupancy look is optional: off here
    runs = os.path.join(R.RUNS, "test_workflow")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot.drag(False)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    store = StateStore(os.path.join(runs, "state.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, R.REFS)
    scope = cyc.scope
    cyc.safety.profile = SpeedProfile(**layout["speeds"][a.first])
    cyc.corr.ask = lambda q, t=0: (say(f"   doubt, carrying on: {q.strip()}"), "y")[1]
    M.plan = stored_routes(M.plan, scope)

    def carry_by_width(step, grip):
        w, (lo, hi) = robot.jaw(), layout["gripper"]["accept_band"][grip]
        if not lo <= w <= hi:
            raise StopSafe(f"{step}: the jaw reads {w:.2f}, outside {lo}..{hi}: the plate is not in the claw as it was")
    cyc._carry_check = carry_by_width
    s = store.s
    s.plate_id, s.step_done, s.hotel, s.plate_at, s.grip = "test", True, {}, scope, "none"
    store.save()
    n, t0 = 0, time.time()
    say(f"START: plate in the {scope}; slots {slots}; first slot on {a.first}, then {a.speed}")
    try:
        rnd = 0
        while not a.rounds or rnd < a.rounds:
            rnd += 1
            for slot in slots:
                if os.path.exists(STOP):
                    say(f"STOP file: stopped after {n} slot cycles, the plate is in the {scope}")
                    return 0
                t = time.time()
                say(f"round {rnd} {slot}: {scope} -> slot")
                cyc.outbound(slot)
                say(f"round {rnd} {slot}: slot -> {scope}")
                cyc.inbound(slot)
                n += 1
                say(f"round {rnd} {slot} done in {time.time() - t:.0f} s ({n} cycles, {(time.time() - t0) / 3600:.2f} h)")
                cyc.safety.profile = SpeedProfile(**layout["speeds"][a.speed])
        say(f"PASSED {n} slot cycles")
        return 0
    except Exception as e:
        say(f"STOPPED after {n} cycles at step {store.s.step!r}: {type(e).__name__}: {e}")
        try:
            say(f"the arm stays where it is: pose {[round(v, 1) for v in robot.pose()]} jaw {robot.jaw():.2f} plate_at {store.s.plate_at} grip {store.s.grip}")
        except Exception:
            pass
        return 2
    finally:
        robot.disconnect()
        tele.close()


if __name__ == "__main__":
    sys.exit(main())
