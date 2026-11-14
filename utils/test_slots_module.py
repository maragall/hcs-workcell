"""Slot <-> rotation module test: the plate starts in the first slot given. For each slot in turn: pick it from the slot, carry it to the
module, set it down at 90 degrees, pick it up again, carry it to the next slot and place it there. After the last slot it goes back to the
first. Every slot is picked from once and placed into once. Uses the loop's own pick and place (run/), nothing new.

A leg from the module to a slot must be the one stored in run/preset/routes.json (utils/plan_preset_routes.py): a different plan stops the test.
Stops at the first refusal and leaves the arm where it is. Stop by hand: create runs/STOP.

    python utils/test_slots_module.py                       # every slot that has preset routes, A then B then C
    python utils/test_slots_module.py A1 A3 --speed fast
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from run import positions  # noqa: E402
from utils.devices import nova  # noqa: E402
from run import motion as M  # noqa: E402
from run.planner.plan import Route  # noqa: E402
from run.corrector import StopSafe  # noqa: E402
from run.cycle import Cycle  # noqa: E402
from run.planner.safety import SpeedProfile  # noqa: E402
from run.state import StateStore  # noqa: E402
from run.telemetry import Telemetry  # noqa: E402

PRESET = json.load(open(os.path.join(HERE, "run", "preset", "routes.json"), encoding="utf-8"))["routes"]
STOP = os.path.join(positions.RUNS, "STOP")


def say(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def guarded(real_plan):
    """Every route the test drives is printed. From the module to a slot it must equal the stored one."""
    def plan(layout, safety, here, target, from_name=None, to_name=None, holding=None, heading=None):
        h = holding or "empty"
        key = None
        if (from_name or "").startswith("polar.pose_at_") and "hotel" in (to_name or ""):     # both module seats share one hover
            key = f"module->{(to_name or '').split()[-1]}|{h}"
        elif (to_name or "").startswith("polar.pose_at_"):                                     # back from a slot's entry point
            for k, v in PRESET.items():
                if k.endswith(f"->module|{h}") and "poses" in v and max(abs(a - b) for a, b in zip(v["entry"][:3], here[:3])) <= 1.0:
                    key = k
        if key is None:
            route = real_plan(layout, safety, here, target, from_name, to_name, holding=holding, heading=heading)
            say(f"   route {from_name or 'here'} -> {to_name}: {len(route.poses)} legs, planned now")
            return route
        want = PRESET.get(key, {})
        if "poses" not in want:
            raise StopSafe(f"no stored route {key}: not driving a route planned on the spot")
        if max(abs(a - b) for a, b in zip(want["poses"][-1][:3], target[:3])) > 0.6:
            raise StopSafe(f"the stored route {key} ends at {want['poses'][-1]}, the move wants {target}: plan the routes again")
        say(f"   route {key}: {len(want['poses'])} legs, stored, replayed")
        return Route("stored", [list(q) for q in want["poses"]], joint_targets={int(i): list(j) for i, j in want["joint_targets"].items()})
    return plan


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("slots", nargs="*")
    ap.add_argument("--speed", default="overdrive", help="a profile of layout speeds: supervised, fast, stress, overdrive")
    ap.add_argument("--from-module", action="store_true", help="the plate lies on the module at 90: carry it to the first slot before the test")
    ap.add_argument("--slow-first", action="store_true", help="drive the first slot on the supervised profile")
    a = ap.parse_args(argv)
    have = sorted({k.split("|")[0].split("->")[1] for k, v in PRESET.items() if k.startswith("module->") and "poses" in v and "|long_edges" in k}, key=lambda s: (s[0], int(s[1:])))
    slots = a.slots or have
    missing = [s for s in slots if s not in have]
    if missing:
        sys.exit(f"no stored routes for {missing}")
    layout = positions.load()
    runs = os.path.join(positions.RUNS, "test_slots_module")
    tele = Telemetry(runs)
    robot, st = nova.connect_readonly(layout, tele.log)
    if st.drag:
        robot.drag(False)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    store = StateStore(os.path.join(runs, "state.json"))
    cyc = Cycle(layout, robot, tele, store, "supervised", runs)
    cyc.safety.profile = SpeedProfile(**layout["speeds"]["fast" if a.slow_first else a.speed])
    cyc.corr.ask = lambda q, t=0: (say(f"   doubt, carrying on: {q.strip()}"), "y")[1]
    M.plan = guarded(M.plan)

    def carry_by_width(step, grip):
        """The carry check without starting the torque probe (3 to 4 s each): the jaw still reads the plate's width."""
        w, lo, hi = robot.jaw(), *layout["gripper"]["accept_band"][grip]
        if not lo <= w <= hi:
            raise StopSafe(f"{step}: the jaw reads {w:.2f}, outside {lo}..{hi}: the plate is not in the claw as it was")
    cyc._carry_check = carry_by_width
    s = store.s
    s.plate_id, s.step_done, s.hotel, s.plate_at, s.grip = "test", True, {slots[0]: "test"}, f"hotel:{slots[0]}", "none"
    if a.from_module:
        s.hotel, s.plate_at = {}, "polar"
        cyc.polar.angle = 90
    store.save()
    F = layout["fixtures"]
    seat = F["polar"]["pose_at_90"]["pose"]
    done = 0
    try:
        cyc.move_to("polar.pose_at_90")
        if a.from_module:
            say(f"plate on the module: carrying it to {slots[0]}")
            cyc._pick("polar", seat, F["polar"], "long_edges", None, gate="polar@90")
            cyc._hotel_place(slots[0])
            cyc.move_to("polar.pose_at_90")
        for i, slot in enumerate(slots):
            nxt = slots[(i + 1) % len(slots)]
            if os.path.exists(STOP):
                say("STOP file: stopping between slots")
                break
            say(f"{slot}: pick from the slot")
            cyc._hotel_pick(slot)
            say(f"{slot}: to the module")
            cyc.move_to("polar.pose_at_90")
            cyc._place("polar", seat, F["polar"], "polar", None, gate="polar@90", check=False)
            say(f"{slot}: on the module, jaw {robot.jaw():.2f}; picking it up again")
            cyc._pick("polar", seat, F["polar"], "long_edges", None, gate="polar@90")
            say(f"{slot} -> {nxt}: place in the slot (held at {robot.jaw():.2f})")
            cyc._hotel_place(nxt)
            cyc.move_to("polar.pose_at_90")
            done += 1
            say(f"{slot} done ({done} of {len(slots)})")
            cyc.safety.profile = SpeedProfile(**layout["speeds"][a.speed])
        say(f"PASSED {done} of {len(slots)} slots" if done == len(slots) else f"stopped after {done} of {len(slots)} slots")
        return 0 if done == len(slots) else 1
    except Exception as e:
        say(f"STOPPED at slot {slots[min(done, len(slots) - 1)]} after {done} done: {type(e).__name__}: {e}")
        say(f"the arm stays where it is: pose {[round(v, 1) for v in robot.pose()]} jaw {robot.jaw():.2f}")
        return 2
    finally:
        robot.disconnect()
        tele.close()


if __name__ == "__main__":
    sys.exit(main())
