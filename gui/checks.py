"""
Test: one thing at a time against what the operator sees on the bench. The slot check and the interleaving test
are exercise.py's own steps; the marker check is cli marker-check after a goto; the path preview is routes.plan.
"""
from __future__ import annotations

import asyncio
import math
import time
from typing import List, Optional

import exercise
from procedures.corrector import StopSafe
from procedures.hotel import plate_order
from procedures.motion import offset
from procedures.safety import Safety, SafetyViolation
from routes.plan import plan

from gui import bench, runs
from gui.teach import SLOW

LEARNED_EVERY_S = 10.0


def positions(L: dict) -> List[dict]:
    """The places move_to knows, taught ones only, with their words on screen."""
    out = []
    for name in L.get("routes", {}).get("positions", []):
        node = L["fixtures"]
        try:
            for part in name.split("."):
                node = node[part]
        except KeyError:
            continue
        if node.get("pose"):
            label = bench.label(name) if name.startswith("microscope") else f"Stage at {name.rsplit('_', 1)[1]}"
            out.append({"name": name, "label": label})
    return out


def markers(L: dict) -> List[dict]:
    """Fixtures with a marker reference, and the position whose hover looks at it."""
    out = []
    for name in bench.microscopes(L):
        if L["fixtures"][name].get("aruco_reference") and L["fixtures"][name].get("pose"):
            out.append({"fixture": name, "position": name, "label": bench.label(name)})
    if L["fixtures"]["polar"].get("aruco_reference"):
        out.append({"fixture": "polar", "position": "polar.pose_at_0", "label": "Stage"})
    return out


def state(cell) -> dict:
    L, c = cell.layout(), cell.checks
    if time.time() - c.get("learned_at", 0) > LEARNED_EVERY_S:
        c["learned"], c["learned_at"] = runs.learned(cell), time.time()
    test = c.get("test")
    return {"positions": positions(L), "markers": markers(L), "slots": c.get("slots", {}), "learned": c["learned"],
            "marker": c.get("marker", ""), "path": c.get("path", ""), "bench": bench.state(L),
            "test": test.state() if test else None}


# ------------------------------------------------------------------------------------------- the checks
def _from_microscope(cell, cyc, forward: bool):
    """Microscope hover <-> stage hover along the microscope's route, as exercise.py travels it."""
    route = cell.layout()["fixtures"][cyc.scope].get("trajectory", "microscope_to_polar")
    if forward:
        exercise.along(cyc, route)
    else:
        exercise.along(cyc, route, reverse=True)
        cyc._arrive(cyc._position(cyc.scope), "back at the microscope", cyc.safety.profile.approach)


def check_slots(cell) -> None:
    """The arm looks into every slot of the taught columns; each verdict is compared with the clicked bench truth."""
    L = cell.layout()
    truth = set(bench.loaded_slots(L))
    cyc = cell.cycle(tag="slots")
    slots = [s for s in bench.slots(L) if s[0] in bench.state(L)["taught_columns"]]
    result = {}
    _from_microscope(cell, cyc, True)
    for slot in slots:
        if cell.stop_event.is_set():
            break
        try:
            v = exercise.probe_slots(cyc, [slot], cyc.tele.log)[slot]
        except SafetyViolation as e:
            cyc.tele.log("exercise", f"probe:{slot}", f"unreachable: {e}")
            result[slot] = "unreachable"
            continue
        result[slot] = "unsure" if v not in ("empty", "occupied") else "agree" if (v == "occupied") == (slot in truth) else "disagree"
        cell.checks["slots"] = dict(result)
    exercise.along(cyc, "polar_to_hotel", reverse=True)
    _from_microscope(cell, cyc, False)
    n = sum(1 for v in result.values() if v == "agree")
    cell.note(f"slot check: {n} of {len(result)} agree, {sum(1 for v in result.values() if v == 'disagree')} disagree")


def check_marker(cell, fixture: str) -> None:
    """Go to the fixture's hover, read the marker, say how far the position has drifted (cli marker-check)."""
    L = cell.layout()
    m = next((x for x in markers(L) if x["fixture"] == fixture), None)
    if not m:
        cell.note(f"{fixture}: no marker taught")
        return
    node = L["fixtures"]
    for part in fixture.split("."):
        node = node[part]
    cyc = cell.cycle(tag="marker")
    cyc.move_to(m["position"])
    off = cell.vision.pose_offset(L["vision"]["down_camera"], node["aruco_reference"], L["vision"]["workspace"],
                                  node.get("aruco_marker_id", 1), None, L["vision"]["led"])
    mag = math.sqrt(sum(v * v for v in off.d[:3]))
    limit = L["vision"]["max_correction_mm"]
    cell.checks["marker"] = (f"{m['label']}: shift {off.d[0]:.1f} {off.d[1]:.1f} {off.d[2]:.1f} mm, {mag:.1f} mm total, "
                             f"limit {limit:.0f} mm: {'ok' if mag <= limit else 'too far, teach it again'}")
    cell.note(cell.checks["marker"])


def show_path(cell, to: str) -> None:
    """The legs the arm would take from where it is to a position's hover, checked, nothing moves."""
    L = cell.layout()
    cyc = cell.cycle(tag="path")
    here = cell.robot.pose()
    try:
        route = plan(L, Safety(L, "supervised"), here, cyc._position(to), cyc._position_here(), to)
    except SafetyViolation as e:
        cell.checks["path"] = f"no path: {e}"
        return
    if route.taught:
        text = f"taught route, {len(route.poses)} points, each checked"
    else:
        legs = []
        for p in route.poses:
            if abs(p[2] - here[2]) > 0.5 and p[:2] == here[:2]:
                legs.append(f"{'up' if p[2] > here[2] else 'down'} to {p[2]:.0f} mm")
            elif p[:2] != here[:2]:
                legs.append("across" if abs(p[2] - here[2]) < 0.5 else f"across and {'up' if p[2] > here[2] else 'down'} to {p[2]:.0f} mm")
            here = p
        text = ", ".join(legs) + f"; {len(route.poses)} moves, all inside the safe box"
    cell.checks["path"] = text


def go(cell, to: str) -> None:
    cyc = cell.cycle(tag="go")
    cyc.safety.profile.travel = cyc.safety.profile.approach = SLOW
    cyc.move_to(to)
    cell.note(f"arrived at {to.replace('.', ' ').replace('_', ' ')}")


# -------------------------------------------------------------------------------- the interleaving test
def test(cell, cycles: int) -> None:
    """exercise.py's flow on this cell: look into every loaded slot, turn the stage with its camera check, trip itself
    on a taken home slot, then interleave the plates through the microscope `cycles` times. Fresh ledgers."""
    L = cell.layout()
    scopes = [s for s in bench.microscopes(L) if L["fixtures"][s].get("pose")] or ["microscope"]
    order = plate_order(L)
    if not order:
        cell.note("no plates clicked on the matrix")
        return
    r = runs.Run(cell, scopes, "supervised", "_test", fresh_ledgers=True)
    cell.checks["test"], cell.run = r, r
    truth = set(bench.loaded_slots(L))
    cyc = next(iter(r.wc.cycles.values()))
    slot_of = {s: order[i % len(order)] for i, s in enumerate(scopes)}
    try:
        r.tele.log("exercise", "start", f"{len(order)} plates: {' '.join(order)}", scopes=scopes)
        _from_microscope(cell, cyc, True)
        verdicts = exercise.probe_slots(cyc, order, r.tele.log)
        r.slot_views = {s: "unsure" if v not in ("empty", "occupied") else "agree" if (v == "occupied") == (s in truth) else "disagree"
                        for s, v in verdicts.items()}
        cell.checks["slots"] = dict(r.slot_views)
        exercise.along(cyc, "polar_to_hotel", reverse=True)
        exercise.probe_stage(cyc, r.tele.log)
        _from_microscope(cell, cyc, False)
        if cell.stop_event.is_set():
            raise StopSafe("STOP pressed")
        exercise.trip_itself(cyc, order[1] if len(order) > 1 else order[0], r.tele.log)
        asyncio.run(r.wc.run(slot_of, cycles, False, r.stop_file, list(order)))
        r.tele.log("exercise", "finished", plates=r.wc.plates(), verdicts=verdicts)
        r.close("finished")
        cell.note("interleaving test finished")
    except (StopSafe, SafetyViolation) as e:
        r.close(f"stopped: {e}")
        cell.note(f"interleaving test stopped: {e}")
    except Exception as e:
        r.close(f"failed: {e!r}")
        raise
    finally:
        cell.run = None
