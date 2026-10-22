"""
Test everything, one button: every slot looked at and compared with the clicked matrix (exercise.probe_slots),
the marker read at every taught position (cli marker-check after a goto), the path between the microscope and
the rotation module previewed (routes.plan) and driven slowly (move_to), the module turned with its camera check
(exercise.probe_stage), the trip on a taken home slot on purpose (exercise.trip_itself), then the plates in turn
through the microscope (run.Workcell with the interleaved order). Fresh ledgers, live log and frames on the page.
"""
from __future__ import annotations

import asyncio
import math
import time
from typing import Dict, List

import exercise
from procedures.corrector import StopSafe
from procedures.hotel import plate_order
from procedures.safety import SafetyViolation
from routes.plan import plan

from gui import bench, runs
from gui.teach import SLOW

LEARNED_EVERY_S = 10.0


def state(cell) -> dict:
    c = cell.checks
    if time.time() - c.get("learned_at", 0) > LEARNED_EVERY_S:
        c["learned"], c["learned_at"] = runs.learned(cell), time.time()
    test = c.get("test")
    return {"slots": c.get("slots", {}), "learned": c["learned"], "bench": bench.state(cell.layout()),
            "test": test.state() if test else None}


def _compare(verdicts: Dict[str, str], truth: set) -> Dict[str, str]:
    return {s: "unsure" if v not in ("empty", "occupied") else "agree" if (v == "occupied") == (s in truth) else "disagree"
            for s, v in verdicts.items()}


def _stopped(cell):
    if cell.stop_event.is_set():
        raise StopSafe("STOP pressed")


# ---------------------------------------------------------------------------------------------- the steps
def look_at_every_slot(cell, r, cyc, log) -> None:
    """Arm at the microscope hover in, and out. Each slot of a taught column; an unreachable one is said, not fatal."""
    L = cell.layout()
    truth = set(bench.loaded_slots(L))
    slots = [s for s in bench.slots(L) if s[0] in bench.state(L)["taught_columns"]]
    log("exercise", "slots", f"looking into {len(slots)} slots")
    exercise.along(cyc, r.scope_route)
    for slot in slots:
        _stopped(cell)
        try:
            v = exercise.probe_slots(cyc, [slot], log)[slot]
            r.slot_views.update(_compare({slot: v}, truth))
        except SafetyViolation as e:
            log("exercise", f"probe:{slot}", f"{slot} is out of reach: {e}")
            r.slot_views[slot] = "unreachable"
        cell.checks["slots"] = dict(r.slot_views)
    exercise.along(cyc, "polar_to_hotel", reverse=True)
    exercise.along(cyc, r.scope_route, reverse=True)
    cyc._arrive(cyc._position(cyc.scope), "back at the microscope", cyc.safety.profile.approach)
    n = sum(1 for v in r.slot_views.values() if v == "agree")
    log("exercise", "slots", f"slots: {n} of {len(r.slot_views)} agree with the matrix, "
        f"{sum(1 for v in r.slot_views.values() if v == 'disagree')} disagree")


def markers(L: dict) -> List[dict]:
    """Fixtures with a marker, and the position whose hover looks at it."""
    out = [{"fixture": s, "position": s, "label": bench.label(s)} for s in bench.microscopes(L)
           if L["fixtures"][s].get("aruco_reference") and L["fixtures"][s].get("pose")]
    if L["fixtures"]["polar"].get("aruco_reference"):
        out.append({"fixture": "polar", "position": "polar.pose_at_0", "label": "rotation module"})
    return out


def read_every_marker(cell, cyc, log) -> None:
    """Go to each marker's hover and say how far the position has drifted (cli marker-check)."""
    L = cell.layout()
    v, limit = L["vision"], L["vision"]["max_correction_mm"]
    for m in markers(L):
        _stopped(cell)
        node = L["fixtures"][m["fixture"]]
        if cyc._position_here() != m["position"]:
            cyc.move_to(m["position"])
        off = cell.vision.pose_offset(v["down_camera"], node["aruco_reference"], v["workspace"], node.get("aruco_marker_id", 1), None, v["led"])
        mag = math.sqrt(sum(x * x for x in off.d[:3]))
        log("exercise", "marker", f"marker at the {m['label'].lower()}: shift {mag:.1f} mm, limit {limit:.0f} mm: "
            f"{'ok' if mag <= limit else 'too far, teach it again'}", mag=round(mag, 2))
    if not markers(L):
        log("exercise", "marker", "no markers taught; the arm will use the taught poses as they are")


def path_words(route, here: List[float]) -> str:
    if route.taught:
        return f"taught route, {len(route.poses)} points, each checked"
    legs = []
    for p in route.poses:
        if p[:2] == here[:2]:
            legs.append(f"{'up' if p[2] > here[2] else 'down'} to {p[2]:.0f} mm")
        else:
            legs.append("across")
        here = p
    return ", ".join(legs) + f"; {len(route.poses)} moves, all inside the safe box"


def drive(cell, cyc, log, to: str, label: str) -> None:
    """Preview the path to a position's hover, then drive it slowly."""
    _stopped(cell)
    here = cell.robot.pose()
    if cyc._position_here() == to:
        return
    route = plan(cell.layout(), cyc.safety, here, cyc._position(to), cyc._position_here(), to)
    log("exercise", "path", f"path to the {label}: {path_words(route, here)}")
    travel, approach = cyc.safety.profile.travel, cyc.safety.profile.approach
    cyc.safety.profile.travel = cyc.safety.profile.approach = SLOW
    try:
        cyc.move_to(to)
    finally:
        cyc.safety.profile.travel, cyc.safety.profile.approach = travel, approach
    log("exercise", "path", f"at the {label}")


def drive_to_the_module_and_back(cell, cyc, log) -> None:
    """Microscope -> rotation module slowly, turn the module with its camera check, back to its hover, -> microscope."""
    module = "polar.pose_at_0"
    drive(cell, cyc, log, cyc.scope, "microscope")
    drive(cell, cyc, log, module, "rotation module")
    exercise.probe_stage(cyc, log)
    cyc._move("L", cyc._position(module), "back to the module hover", cyc.safety.profile.approach)
    drive(cell, cyc, log, cyc.scope, "microscope")


# --------------------------------------------------------------------------------------------- the button
def test(cell, cycles: int) -> None:
    L = cell.layout()
    scopes = [s for s in bench.microscopes(L) if L["fixtures"][s].get("pose")] or ["microscope"]
    order = plate_order(L)
    if not order:
        cell.note("click the slots that hold plates first")
        return
    r = runs.Run(cell, scopes, "supervised", "_test", fresh_ledgers=True)
    cell.checks["test"], cell.run = r, r
    cell.checks["slots"] = {}
    cyc = next(iter(r.wc.cycles.values()))
    r.scope_route = L["fixtures"][cyc.scope].get("trajectory", "microscope_to_polar")
    log = r.tele.log
    slot_of = {s: order[i % len(order)] for i, s in enumerate(scopes)}
    try:
        log("exercise", "start", f"{len(order)} plates: {' '.join(order)}", scopes=scopes, cycles=cycles)
        look_at_every_slot(cell, r, cyc, log)
        read_every_marker(cell, cyc, log)
        drive_to_the_module_and_back(cell, cyc, log)
        _stopped(cell)
        exercise.trip_itself(cyc, order[1] if len(order) > 1 else order[0], log)
        log("exercise", "plates", f"now the plates in turn, {cycles} time{'s' if cycles != 1 else ''} each microscope")
        asyncio.run(r.wc.run(slot_of, cycles, False, r.stop_file, list(order)))
        log("exercise", "finished", "test finished", plates=r.wc.plates(), slots=r.slot_views)
        r.close("finished")
        cell.note("test finished")
    except (StopSafe, SafetyViolation) as e:
        r.close(f"stopped: {e}")
        cell.note(f"test stopped: {e}")
    except Exception as e:
        r.close(f"failed: {e!r}")
        raise
    finally:
        cell.run = None
