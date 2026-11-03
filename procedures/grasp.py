"""The jaw: open, close, judge the grasp, centre on the plate, remember what was learned. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

from typing import Optional

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
        try:
            self._jaw(want, "jaw open")
        except StopSafe:
            # The fingers stalled against the plate that sits in the clips of the seat (24.66 at the module, 2026-11-02, a jaw that
            # had opened hundreds of times). The plate is lifted 3 mm off the seat's clips, then the jaw is asked again, once.
            here = self.robot.pose()
            self.tele.log("jaw", "open refused", "lifting 3 mm off the seat and asking again")
            self._move("L", [here[0], here[1], here[2] + 3.0] + list(here[3:]), "lift off the clips before opening", self.safety.profile.contact)
            self._jaw(want, "jaw open")
        self._wait_still("jaw open")
        self.clock.sleep(0.5)
        jaw = self.robot.jaw()
        self.tele.log("jaw", "open", jaw=round(jaw, 2), grip=grip)
        if jaw < want - 1.0:
            raise StopSafe(f"jaw did not open to {want} for {grip} ({jaw:.2f})")
        return jaw

    def _clamp(self, grip: str, step: str):
        """Close under the torque limit (devices/clamp.py) and judge by force: returns (reading, None when a plate is held, else the reason)."""
        g = self.L["gripper"]
        limit, speed = float(g.get("clamp_limit", 0.5)), int(g.get("clamp_speed", 20))
        known = grip in self.checks.width                     # the width this grip reads on the plate, learned at its first holding clamp
        command = max(self.plate.jaw_closed[grip] - (0.0 if known else 2.0), 0.0)      # unknown yet: a command well under any plate
        try:
            r = self.robot.clamp(command, limit, speed, self.checks.width_of(grip))
        except Exception as e:
            self._fault(step, "clamp_error", str(e))
            raise StopSafe(f"{step}: the force clamp failed: {e}")
        why = self.checks.clamp_holds(r, command, limit, grip)
        self.tele.log("jaw", "clamp", why or "holding", grip=grip, width=r["width"], torque=r["torque"], limit=limit)
        self._observe(f"jaw:clamp:{grip}", r["width"])
        if not why and not known:
            self._learn_width(grip, r["width"], step)
        return r, why

    def _learn_width(self, grip: str, width: float, step: str) -> None:
        """The first holding clamp of a grip teaches its width: kept in layout.json (gripper.plate_width), the clamp command 1 mm under it, the
        accept band +-0.5 around it. There is no grips step in the setup: delete gripper.plate_width for a new plate or new fingers."""
        g = self.L["gripper"]
        w = round(width, 2)
        g.setdefault("plate_width", {})[grip] = w
        g["closed"][grip] = round(w - 1.0, 2)
        g.setdefault("accept_band", {})[grip] = [round(w - 0.5, 2), round(w + 0.5, 2)]
        self.checks.width[grip] = w
        self.checks.closed[grip] = self.plate.jaw_closed[grip] = g["closed"][grip]
        self.tele.log("correct", f"{step} grip width", f"{grip}: the pads meet the plate at {w}; command {g['closed'][grip]}, band {g['accept_band'][grip]}")
        positions.save(self.L)

    def _carry_check(self, step: str, grip: str) -> None:
        """The plate is still pressed between the fingers: the torque read back is at the limit and the width has not moved."""
        g = self.L["gripper"]
        limit = float(g.get("clamp_limit", 0.5))
        try:
            r = self.robot.clamp_read()
        except Exception as e:
            self._fault(step, "clamp_error", str(e))
            raise StopSafe(f"{step}: the force read failed: {e}")
        why = self.checks.clamp_holds(r, self.plate.jaw_closed[grip], limit, grip)
        self.tele.log("jaw", "carry", why or "holding", grip=grip, width=r["width"], torque=r["torque"])
        if why:
            self._fault(step, "plate_lost", why)
            raise StopSafe(f"{step}: the plate is not in the claw: {why}")
