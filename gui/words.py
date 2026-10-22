"""Plain words for the screen: an event from events.jsonl becomes one short line, a saved frame gets a caption."""
from __future__ import annotations

import os
import re
import time
from typing import Optional

SKIP_KINDS = {"move", "health", "predict", "record"}


def _step_place(step: str) -> str:
    """'pick:hotel:A3 hover' -> 'slot A3', 'place:polar' -> 'the stage', 'pick:microscope' -> 'the microscope'."""
    m = re.match(r"(pick|place|probe):(hotel:)?([A-Za-z_0-9@]+)", step)
    if not m:
        return step
    what = m.group(3)
    if m.group(2):
        return f"slot {what}"
    if what.startswith("polar"):
        return "the stage"
    return "the microscope" if what == "microscope" else what.replace("_", " ")


def plain(e: dict) -> Optional[str]:
    """One line for the log, or None for events the operator does not need to see."""
    kind, step, msg = e.get("kind", ""), e.get("step", ""), e.get("msg", "")
    if kind in SKIP_KINDS:
        return None
    if kind == "run" and step == "arm":
        return msg.replace("microscope_", "microscope ").replace("takes the arm", "takes the arm").replace("returns", "returns")
    if kind == "vision":
        if step.endswith("lower_check"):
            return "looked from above: clear" if e.get("safe") else f"looked from above: not clear ({msg})"
        if "_front" in step and "slot" not in step:
            if "collected" in msg:
                return f"looked from the front: learning this view ({msg.split()[1]} so far)"
            return "looked from the front: clear" if e.get("safe") else f"looked from the front: not clear ({msg})"
        if step.endswith("slot_view"):
            d = e.get("decision", "unknown")
            slot = re.search(r"hotel:(\w+)|probe:(\w+)", step)
            slot = (slot.group(1) or slot.group(2)) if slot else "?"
            return f"slot {slot} looks {'empty' if d == 'empty' else 'full' if d == 'occupied' else 'unclear'}"
        if step.endswith("slot_ref"):
            return None
        if step.endswith("seated_check") or "seated_front" in step:
            if "collected" in msg:
                return None
            return "plate seated" if e.get("safe") else f"plate does not look seated ({msg})"
        if "plate_in_gripper" in step:
            if "collected" in msg:
                return None
            return "plate held" if e.get("safe") else f"plate not held as expected ({msg})"
        if step == "barcode":
            return f"plate code {msg}" if msg != "none" else "no plate code read"
        if " offset" in step:
            return f"marker: shift {e.get('mag', 0):.1f} mm"
        if "stage" in step:
            return None
        return None
    if kind == "jaw" and step == "close":
        return f"fingers closed, {e.get('jaw', 0):.2f} mm"
    if kind == "jaw":
        return None
    if kind == "fault":
        return f"problem at {_step_place(step)}: {msg} {e.get('detail', '')}".strip()
    if kind == "alert":
        return f"attention: {msg}"
    if kind == "stop":
        return f"stopped: {msg}"
    if kind == "cycle":
        return f"plate change done in {e.get('dt', 0):.0f} s"
    if kind == "polar":
        return None if step in ("readback",) else f"stage: {msg}"
    if kind == "correct":
        return f"adjusting: {msg}"
    if kind == "exercise":
        if step.startswith("probe:"):
            return None                     # the slot_view line already said it
        return msg
    if kind == "run":
        if step == "start":
            return "run started"
        if step == "finished":
            return "run finished"
        if step == "slot_occupied":
            return f"slot taken, using another: {msg}"
        if step == "stop_file":
            return "finishing: stop requested"
        return msg or step
    return msg or None


def when(e: dict) -> str:
    return time.strftime("%H:%M:%S", time.localtime(e.get("t", time.time())))


CAPTIONS = [("_lower_", "from above, before lowering"), ("_front", "from the front, before lowering"),
            ("_slot_view", "the slot, from the front"), ("_lifted", "plate held"), ("_released_down", "after release, from above"),
            ("_seated_front", "after release, from the front"), ("_placed", "placed"), ("_aruco", "marker"),
            ("barcode", "plate code"), ("_stage", "stage, before turning")]


def caption(path: str) -> str:
    name = os.path.basename(path)
    for key, text in CAPTIONS:
        if key in name:
            return text
    return name.split("_", 1)[-1].rsplit(".", 1)[0].replace("_", " ")
