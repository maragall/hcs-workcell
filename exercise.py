"""
The generalist exercise: every feature, in one hardcoded script, on the simulator or on the arm.

    python exercise.py --sim                 fake arm and microscopes (the vision box must answer, or pass --fake-vision)
    python exercise.py --sim --fake-vision   nothing real at all
    python exercise.py --mode supervised     the real cell, after preflight passes

Steps, each logged to the run folder and printed:
  1  occupancy probe   visit every loaded slot's approach point; the front camera says empty / occupied / unknown;
                       compare with what the positions file declares
  2  stage probe       rotate the stage 0 -> 90 -> 0 with the camera check before each turn
  3  trip itself       with a plate on the microscope, order a place into a slot the ledger marks occupied; the arm
                       must back out and put the plate in its home slot instead, never stop
  4  interleave        the plate order from the hotel (A1, C2, A3, ...), every microscope sharing it, --cycles changes each
"""
from __future__ import annotations

import asyncio
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import run  # noqa: E402
from procedures import positions  # noqa: E402
from procedures.corrector import StopSafe  # noqa: E402
from procedures.hotel import SlotOccupied, plate_order  # noqa: E402
from procedures.safety import SafetyViolation  # noqa: E402


def along(cyc, route, reverse=False):
    """Travel a taught route, the way the cycle does between its parts."""
    cyc._waypoints(route, reverse=reverse)


def probe_slots(cyc, slots, log):
    """Approach every slot, read the detector, come back. Returns {slot: decision}."""
    out = {}
    for slot in slots:
        step = f"probe:{slot}"
        cyc._hotel_go_to_approach(slot, step)
        sv = cyc._slot_view(step, slot, "occupied" if slot in cyc.store.s.hotel else "empty")
        out[slot] = sv.decision if sv else "no frame"
        cyc._hotel_leave_approach(slot, step)
        log("exercise", step, out[slot], declared="plate" if slot in cyc.store.s.hotel else "empty",
            ready=cyc.slots.ready(slot))
    return out


def probe_stage(cyc, log):
    F = cyc.L["fixtures"]["polar"]
    P0, P90 = F["pose_at_0"]["pose"], F["pose_at_90"]["pose"]
    cyc._polar_rotate(90, P0)
    cyc._polar_rotate(0, P90)
    log("exercise", "probe:stage", "rotated 0 -> 90 -> 0 with the camera check", refs=cyc.lower.count("stage", "to_90"))


def trip_itself(cyc, slot, log):
    """The plate's home slot is taken while it is on the microscope: the place must back out, use another slot
    and the run must not stop."""
    s = cyc.store.s
    home = s.home_slot or slot
    s.hotel[home] = "a plate someone left there"                 # what the camera would find on the real cell
    cyc.store.save()
    try:
        cyc.run(slot, 1)                                          # out: home is occupied -> another slot; in: the plate from `slot`
    except SlotOccupied as e:                                     # must never surface: the place recovers on its own
        raise StopSafe(f"trip test: the place did not recover: {e}")
    log("exercise", "trip", f"home {home} was taken; the plate went elsewhere, the one from {slot} came in", hotel=dict(s.hotel))


def main(argv=None) -> int:
    ap = run.parser()
    ap.add_argument("--fake-vision", action="store_true", help="synthetic frames instead of the vision box (simulator only)")
    args = ap.parse_args(argv)
    layout = positions.load()
    vision, ref_dir = None, positions.REFS
    if args.fake_vision:                                          # nothing real: the simulator rig the tests use
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tests"))
        import rig

        from perception.lower_check import LowerCheck
        vision, layout = rig.FakeVision(), rig.layout()
        ref_dir = tempfile.mkdtemp(prefix="exercise_refs_")
        lc = LowerCheck(ref_dir)
        gates = rig.GATES + [(n, st) for n in ("microscope_2",) for st in ("plate_present", "empty")]
        for gate, state in gates:                                 # three synthetic "clear" photos per gate, as the tests do
            for _ in range(3):
                lc.add_reference(gate, state, rig.jpeg(rig.SAFE))
    state_dir = tempfile.mkdtemp(prefix="exercise_state_") if args.sim else None       # the simulator starts from fresh ledgers
    cell, robot, tele, slot_of = run.build(args, layout, vision=vision, tag="_exercise", ref_dir=ref_dir, state_dir=state_dir)
    if args.sim:
        robot.follow_layout(layout)
    cyc = next(iter(cell.cycles.values()))
    order = plate_order(layout)
    try:
        tele.log("exercise", "start", f"{len(order)} plates: {' '.join(order)}", scopes=list(cell.cycles))
        scope_route = layout["fixtures"][cyc.scope].get("trajectory", "microscope_to_polar")
        along(cyc, scope_route)                                       # microscope hover -> stage hover
        verdicts = probe_slots(cyc, order, tele.log)                  # stage -> hotel slots -> hotel junction
        along(cyc, "polar_to_hotel", reverse=True)                   # junction -> stage hover
        probe_stage(cyc, tele.log)
        along(cyc, scope_route, reverse=True)                        # stage -> microscope hover
        cyc._arrive(cyc._position(cyc.scope), "exercise: back at the microscope", cyc.safety.profile.approach)
        trip_itself(cyc, order[1] if len(order) > 1 else order[0], tele.log)
        asyncio.run(cell.run(slot_of, args.cycles, False, args.stop_file, list(order)))
        tele.log("exercise", "finished", plates=cell.plates(), verdicts=verdicts)
        print("exercise finished:", cell.plates())
        return 0
    except (StopSafe, SafetyViolation) as e:
        tele.log("stop", "safe", str(e), plates=cell.plates())
        print("exercise stopped:", e)
        return 2
    finally:
        cell.close()
        robot.disconnect()
        tele.close()


if __name__ == "__main__":
    sys.exit(main())
