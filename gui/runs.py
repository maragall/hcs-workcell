"""
A run of the main loop (run.py Workcell) as the screens see it: its folder, its stop file, the camera recording,
and what the page shows while it runs: the plain-words log, the frames each check judged on, where every plate is.
Used by the interleaving test on the Test screen and by the run program.
"""
from __future__ import annotations

import glob
import json
import os
import re
import tempfile
import time
from typing import Dict, List, Optional

import run as mainloop
from perception.occupancy import Occupancy
from procedures.recorder import Recorder
from procedures.telemetry import Telemetry

from gui import bench, words

ACQUISITION_S = {True: 3.0, False: 5.0}      # how long a pretend microscope images: sim, real
LOG_LINES, FRAMES = 60, 8


class Run:
    def __init__(self, cell, scopes: List[str], mode: str, tag: str, fresh_ledgers: bool, log_run_start=True):
        self.cell, self.scopes, self.mode = cell, scopes, mode
        L = cell.layout()
        if cell.sim:
            for s in scopes:
                if not L["fixtures"][s].get("pose"):
                    mainloop.borrow_fixture(L, s)
        self.run_dir = os.path.join(cell.runs_dir, time.strftime("%Y%m%d_%H%M%S") + tag + ("_sim" if cell.sim else ""))
        self.stop_file = os.path.join(self.run_dir, "STOP")
        self.tele = Telemetry(self.run_dir)
        squid_found = any(d["name"] == "Microscope" and d["ok"] and not cell.sim for d in cell.devices)
        devices = mainloop._scopes(L, scopes, set() if squid_found else set(scopes), ACQUISITION_S[cell.sim], cell.sim, self.tele.log)
        state_dir = tempfile.mkdtemp(prefix="ledgers_") if fresh_ledgers else cell.state_dir
        os.makedirs(state_dir, exist_ok=True)
        self.wc = mainloop.Workcell(L, cell.robot, cell.vision, devices, self.tele, mode, self.run_dir, cell.ref_dir,
                                    state_dir, layout_path=cell.layout_path, frame_source=cell.frames)
        for cyc in self.wc.cycles.values():
            cyc.corr.ask = cell.questions.ask
        self.rec = Recorder(self.run_dir, cell.frames, list(cell.frames.cameras.values()), fps=2.0, screen=False, log=self.tele.log)
        self.rec.start()
        self.started, self.ended, self.outcome = time.time(), None, ""
        self.slot_views: Dict[str, str] = {}      # slot -> agree | disagree | unsure, from this run's slot checks
        if log_run_start:
            self.tele.log("run", "start", mode=mode, scopes=scopes, sim=cell.sim, plates=self.wc.plates())

    # ------------------------------------------------------------------ control
    def stop(self):
        """Now: the loop ends at its next step (Workcell.stop) and between cycles (the stop file)."""
        self.finish()
        self.wc.stop.set()

    def finish(self):
        """After this plate: the stop file ends the loop between cycles."""
        os.makedirs(self.run_dir, exist_ok=True)
        open(self.stop_file, "a").close()

    def close(self, outcome: str):
        self.ended, self.outcome = time.time(), outcome
        self.rec.stop()
        self.wc.close()
        self.tele.log("run", "finished" if outcome == "finished" else "stop", outcome, plates=self.wc.plates())
        self.tele.close()

    @property
    def active(self) -> bool:
        return self.ended is None

    # ------------------------------------------------------------- what to show
    def _events(self) -> List[dict]:
        out = []
        for path in glob.glob(os.path.join(self.run_dir, "**", "events.jsonl"), recursive=True):
            with open(path, "rb") as f:
                f.seek(max(0, os.path.getsize(path) - 65536))
                for line in f.read().decode("utf-8", "replace").splitlines()[1:] if f.tell() > 65536 else f.read().decode("utf-8", "replace").splitlines():
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        pass
        out.sort(key=lambda e: e.get("t", 0))
        return out

    def log(self) -> List[dict]:
        lines = []
        for e in self._events():
            text = words.plain(e)
            if text:
                lines.append({"t": words.when(e), "text": text})
        return lines[-LOG_LINES:]

    def images(self) -> List[dict]:
        paths = glob.glob(os.path.join(self.run_dir, "**", "images", "*", "*.jpg"), recursive=True)
        paths.sort(key=os.path.getmtime)
        return [{"t": time.strftime("%H:%M:%S", time.localtime(os.path.getmtime(p))), "caption": words.caption(p),
                 "path": os.path.relpath(p, self.cell.runs_dir)} for p in paths[-FRAMES:]]

    def rings(self) -> Dict[str, str]:
        """Per slot, the latest camera check of this run against what the loop expected there."""
        rings = dict(self.slot_views)
        for e in self._events():
            if e.get("kind") != "vision" or not e.get("step", "").endswith("slot_view"):
                continue
            m = re.match(r"(pick|place):hotel:(\w+)", e["step"])
            if not m:
                continue
            expect = "occupied" if m.group(1) == "pick" else "empty"
            d = e.get("decision")
            rings[m.group(2)] = "unsure" if d not in ("empty", "occupied") else "agree" if d == expect else "disagree"
        return rings

    def plates(self) -> List[dict]:
        now = {s: "waiting for the arm" for s in self.scopes}
        for e in self._events():
            if e.get("kind") != "run":
                continue
            msg = e.get("msg", "")
            for s in self.scopes:
                if msg.startswith(f"{s} takes the arm"):
                    now[s] = "changing the plate"
                elif msg.startswith(f"{s} returns the arm"):
                    now[s] = "imaging"
                elif msg.startswith(f"{s}: acquisition done"):
                    now[s] = "waiting for the arm"
        rows = []
        for s in self.scopes:
            st = self.wc.cycles[s].store.s
            at = st.plate_at
            plate = f"from {st.home_slot}" if st.home_slot and at in (s, "gripper") else f"in {at.split(':')[1]}" if at.startswith("hotel:") else "its own plate"
            rows.append({"name": bench.label(s), "plate": plate, "now": now[s] if self.active else self.outcome})
        return rows

    def state(self) -> dict:
        return {"active": self.active, "mode": self.mode, "outcome": self.outcome, "since": self.started,
                "plates": self.plates(), "log": self.log(), "images": self.images(), "rings": self.rings(),
                "stopping": os.path.exists(self.stop_file)}


def learned(cell) -> List[str]:
    """Slots the camera has learned (enough photos of both kinds and a clean self-check)."""
    cfg = cell.layout()["vision"].get("slot_check", {})
    occ = Occupancy(cell.ref_dir, cfg.get("min_refs", 5), cfg.get("min_match", 0.75))
    return [s for s in occ.slots() if occ.ready(s)]


def image_path(cell, rel: str) -> Optional[str]:
    """A saved frame under the runs folder, or None for anything else."""
    p = os.path.realpath(os.path.join(cell.runs_dir, rel))
    return p if p.startswith(os.path.realpath(cell.runs_dir)) and p.endswith(".jpg") and os.path.isfile(p) else None
