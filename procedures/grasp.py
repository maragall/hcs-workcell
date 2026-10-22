"""The jaw: open, close, judge the grasp, centre on the plate, remember what was learned. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

import time
from typing import List, Optional

from devices.arm import MotionRefused
from procedures import positions
from procedures.corrector import StopSafe


class Grasp:
    """The jaw: open, close, judge the grasp, centre on the plate, remember what was learned."""

    def _jaw(self, value: float, label: str):
        try:
            self.robot.jaw_to(value)
        except MotionRefused as e:
            self._fault(label, "jaw_refused", str(e))
            raise StopSafe(f"{label}: {e}")

    def _jaw_open(self, grip: str, fixture: Optional[dict] = None) -> float:
        """Opening width: the grip's default, or the fixture's `open_width[grip]` when it has one (the stage opens
        29 for the short edges since 2026-10-20: the plate's position at 0 after a robot place at 90 and the turn is
        not known to better than the old 1.8 mm per side, see docs/INCIDENTS.md; the jaw's end stop is 29.3)."""
        want = (fixture or {}).get("open_width", {}).get(grip) or self.plate.jaw_open[grip]
        self._jaw(want, "jaw open")
        self._wait_still("jaw open")
        time.sleep(0.5)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "open", jaw=round(jaw, 2), grip=grip)
        if jaw < want - 1.0:
            raise StopSafe(f"jaw did not open to {want} for {grip} ({jaw:.2f})")
        return jaw

    def _jaw_close(self, grip: str) -> float:
        """One close command from open to the taught value, as the vendor's own software does (2026-10-20).

        Measured with the controller's torque readout (lua/torque_probe) on the production plate: the holding torque
        is 70 % of rated (the gripper's configured cap) after a one-command close AND after the old 0.5 mm stepping,
        through the SDK or through Lua, whatever the target below the plate. So stepping never weakened the grip;
        one command is kept because it is the vendor path and takes 3 s instead of 10. A stall before the target is
        still not a fault here: the fingers met the plate (or something) at that width, and `grasp_ok` judges it.
        What differs between fixtures is where the pads land (1.62 to 1.67 on a free plate, 2.2 to 2.5 in the stage
        nest and the hotel), which is geometry, not force."""
        closed = self.plate.jaw_closed[grip]
        start = self.robot.jaw()
        stall = None
        try:
            self.robot.jaw_to(closed)
        except MotionRefused as e:
            stall = str(e)
        self._wait_still("jaw close")
        time.sleep(0.5)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "close", "stalled" if stall else "", jaw=round(jaw, 2), start=round(start, 2), grip=grip,
                      stall=stall)
        self._observe(f"jaw:closed:{grip}", jaw)
        return jaw

    def _center_search(self, step: str, fixture_name: str, fixture: dict, target: List[float], grip: str):
        """Tactile centering. The jaw stalled wider than the plate: one pad touched first because the seat pose is a
        few tenths of a millimetre off the plate along the closing axis (a hand-taught seat, a clamped holder).
        Walk the seat sideways in small steps, re-closing each time, until the fingers meet the plate at its true
        width. The offset that works is written back to the taught pose. Returns (jaw, target) on success, None
        otherwise. Lateral moves stay far inside the no-swing allowance and the fingers are open throughout."""
        cfg = fixture.get("grasp_search")
        if not cfg:
            return None
        # Judge against the fixture's learned signature when it has one (microscope 22.50), not the free-plate width:
        # on 2026-10-20 17:07 y+0.2 read 22.54 twice and was rejected against 22.40, then the attempt failed.
        closed = fixture.get("grasp_width", {}).get(grip) or self.plate.jaw_closed[grip]
        tol = self.checks.jaw_tol
        s, m = float(cfg.get("step_mm", 0.2)), float(cfg.get("max_mm", 0.6))
        sd, md = float(cfg.get("step_deg", 0.3)), float(cfg.get("max_deg", 0.6))
        trials = []                                     # (pose index, offset, unit)
        for axis in cfg.get("axes", ["x", "y"]):
            if axis == "yaw":
                k = 1
                while k * sd <= md + 1e-9:
                    for sign in (1, -1):
                        trials.append((5, sign * k * sd, "deg"))
                    k += 1
            elif axis == "z":
                for d in cfg.get("z_offsets_mm", [0.5, 1.0, -0.5]):
                    trials.append((2, float(d), "mm"))
            else:
                i = {"x": 0, "y": 1}[axis]
                k = 1
                while k * s <= m + 1e-9:
                    for sign in (1, -1):
                        trials.append((i, sign * k * s, "mm"))
                    k += 1
        names = {0: "x", 1: "y", 2: "z", 5: "yaw"}
        self._search_widths = []
        for i, d, unit in trials:
            cand = list(target)
            cand[i] += d
            label = f"{step} search {names[i]}{d:+.1f}{unit}"
            self._jaw_open(grip, fixture)
            here = self.robot.pose()
            cap = 0.4                                   # yaw changes inside a no-swing zone are capped per move
            n = int(abs(cand[5] - here[5]) / cap) + 1   # fixed number of parts, targets taken from `cand`, never from readback
            for k in range(1, n):
                mid = list(cand)
                mid[5] = here[5] + (cand[5] - here[5]) * k / n
                self._move("L", mid, label + f" (yaw part {k}/{n})", self.safety.profile.contact, allow_floor=(fixture_name,))
            self._move("L", cand, label, self.safety.profile.contact, allow_floor=(fixture_name,))
            jaw = self._jaw_close(grip)
            self._search_widths.append(jaw)
            self.tele.log("correct", f"{step} search", f"{names[i]}{d:+.1f}{unit} -> jaw {jaw:.2f}", jaw=round(jaw, 2),
                          axis=names[i], offset=d)
            if abs(round(jaw, 2) - closed) <= tol + 1e-6:
                self._persist_pose(fixture_name, fixture, cand,
                                   f"grasp search {names[i]}{d:+.1f}{unit} on {time.strftime('%Y-%m-%d %H:%M')}")
                return jaw, cand
        return None

    def _adopt_grasp_width(self, step: str, fixture_name: str, fixture: dict, grip: str) -> Optional[float]:
        """When the search finds the same width everywhere, the fixture itself presents the plate that wide (a holder
        that clamps it, a plate that bulges under the clips). If that width is consistent and only a little above the
        hand-measured grip, store it as this fixture's grasp signature and accept it. The held-plate camera check
        after the lift remains the guard against a false grasp."""
        cfg = fixture.get("grasp_search", {})
        ws = sorted(self._search_widths)
        closed = self.plate.jaw_closed[grip]
        if len(ws) < 6:
            return None
        med = ws[len(ws) // 2]
        spread = ws[-1] - ws[0]
        over = med - closed
        if spread > float(cfg.get("adopt_max_spread", 0.25)) or not (0 < over <= float(cfg.get("adopt_max_mm", 0.5))):
            self.tele.log("correct", f"{step} grasp width", f"not adopted: median {med:.2f}, spread {spread:.2f}")
            return None
        width = round(med, 2)
        fixture.setdefault("grasp_width", {})[grip] = width
        fixture.setdefault("corrections", []).append({"grasp_width": {grip: width}, "why": f"search widths {ws} on {time.strftime('%Y-%m-%d %H:%M')}"})
        self.tele.log("correct", f"{step} grasp width", f"adopted {width} for {grip} at {fixture_name} (nominal {closed})", widths=ws)
        self._persist_fixture(fixture_name, fixture, ("grasp_width", "corrections"))
        return width

    def _persist_fixture(self, fixture_name: str, fixture: dict, keys):
        """Write learned keys of a fixture back to the positions file (only when layout_path is set)."""
        if not self.layout_path:
            return
        try:
            positions.update_fixture(fixture_name, self.layout_path, **{k: fixture[k] for k in keys})
        except Exception as e:
            self.tele.log("correct", f"{fixture_name}", f"not persisted: {e}")

    def _persist_pose(self, fixture_name: str, fixture: dict, pose: List[float], why: str):
        fixture["pose"] = [round(v, 4) for v in pose]
        fixture.setdefault("corrections", []).append({"pose": fixture["pose"], "why": why})
        self.tele.log("correct", f"{fixture_name} pose", why, pose=fixture["pose"])
        self._persist_fixture(fixture_name, fixture, ("pose", "corrections"))

    # ---------------------------------------------------------- fixture visits
