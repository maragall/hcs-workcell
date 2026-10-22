"""The plate hotel: slot geometry, the climb and slide in, pick and place in a slot. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

from typing import List, Optional

from procedures.corrector import StopSafe
from procedures.motion import offset
from procedures.safety import SafetyViolation


class Hotel:
    """The plate hotel: slot geometry, the climb and slide in, pick and place in a slot."""

    # ------------------------------------------------------------------ hotel
    def hotel_slot_pose(self, slot: str) -> List[float]:
        h = self.L["fixtures"]["hotel"]
        col, row = slot[:1], slot[1:]
        if col not in h["columns"] or not row.isdigit() or int(row) not in h["odd_rows"]:
            raise SafetyViolation(f"slot {slot} does not exist")
        if slot in h["taught"]:
            return list(h["taught"][slot])
        a, b, n = h["row_vector_from"]
        rv = [(h["taught"][b][i] - h["taught"][a][i]) / n for i in range(3)]
        base = h["taught"][col + "1"]
        return [base[i] + (int(row) - 1) * rv[i] for i in range(3)] + list(base[3:])

    def _hotel_geometry(self, slot: str):
        h = self.L["fixtures"]["hotel"]
        p = self.hotel_slot_pose(slot)
        ax, mm, lift = h["approach"]["axis"], h["approach"]["mm"], h["lift_mm"]
        approach = offset(p, [ax[0] * mm, ax[1] * mm, lift])
        align = offset(p, [ax[0] * mm, ax[1] * mm, 0])
        carry = offset(p, [0, 0, lift])
        return p, approach, align, carry

    def _hotel_junction(self, slot: str) -> List[float]:
        """Where route A ends: the approach point of the column's row-1 slot (taught and validated for A1)."""
        return self._hotel_geometry(slot[0] + "1")[1]

    def _hotel_climb(self, slot: str):
        """For rows above 1 the arm may not climb at the approach x: the plate already overlaps the shelf depth there
        (A3, 2026-10-15). It retreats to `climb_x`, a column in front of the rack the operator cleared, climbs there
        with the column axis only, and slides in at carry height. Returns (foot, top): the retreat point at the
        junction height and the point in front of the slot at carry height; both None for row 1."""
        if slot[1:] == "1":
            return None, None
        h = self.L["fixtures"]["hotel"]
        x = float(h["climb_x"])
        junction = self._hotel_junction(slot)
        _, approach, _, _ = self._hotel_geometry(slot)
        foot = [x, junction[1], junction[2]] + list(junction[3:])
        top = [x, approach[1], approach[2]] + list(approach[3:])
        return foot, top

    def _hotel_go_to_approach(self, slot: str, step: str):
        """Route A end -> junction -> (retreat, climb) -> approach point of the slot, all at carry height."""
        prof = self.safety.profile
        _, approach, _, _ = self._hotel_geometry(slot)
        foot, top = self._hotel_climb(slot)
        junction = self._hotel_junction(slot)
        here = self.robot.pose()
        if max(abs(here[i] - approach[i]) for i in range(3)) <= 5.0:     # resuming at the approach point
            self.resuming = False
            return
        if max(abs(here[i] - junction[i]) for i in range(3)) > 5.0:
            self._waypoints("polar_to_hotel")
            self._move("J", junction, f"{step} junction", prof.approach)
        if foot is not None:
            self._move("L", foot, f"{step} retreat", prof.approach)
            self._move("L", top, f"{step} climb", prof.approach)
            self._move("L", approach, f"{step} approach", prof.approach)

    def _hotel_leave_approach(self, slot: str, step: str):
        """Approach point -> (descend at the climb column) -> junction, ready for route A reversed."""
        prof = self.safety.profile
        foot, top = self._hotel_climb(slot)
        if foot is not None:
            self._move("L", top, f"{step} back to column", prof.approach)
            self._move("L", foot, f"{step} descend", prof.approach)
            self._move("L", self._hotel_junction(slot), f"{step} junction", prof.approach)

    def _hotel_place(self, slot: str):
        prof = self.safety.profile
        p, approach, _, carry = self._hotel_geometry(slot)
        step = f"place:hotel:{slot}"
        if self.store.s.hotel.get(slot) is not None:
            raise StopSafe(f"{step}: state says {slot} is occupied")
        self.store.begin(step)
        self._health()
        self._hotel_go_to_approach(slot, step)
        sv = self._slot_view(step, slot, "empty")
        try:
            ready, _ = self.vision.live_detection(self.front_cam, aruco=False, led=self.led)
        except Exception:
            ready = None
        self.tele.log("vision", f"{step} slot_view", sv.decision if sv else "no frame",
                      score_empty=round(sv.score_empty, 3) if sv else None,
                      score_occupied=round(sv.score_occupied, 3) if sv else None, vendor_live=ready)
        if sv and sv.decision == "occupied":
            self._fault(step, "slot_occupied", f"empty {sv.score_empty:.3f} vs occupied {sv.score_occupied:.3f}")
            if not self.corr.operator_or_stop(step, f"{slot} looks occupied", "Slot is actually empty?"):
                raise StopSafe(f"{slot} occupied")
        if sv is None or sv.decision == "unknown":
            if ready and not self.corr.operator_or_stop(step, f"{slot} may be occupied (vendor live detection)",
                                                        "Slot is actually empty?"):
                raise StopSafe(f"{slot} occupied")
        if self.checks.before_release(self.robot.jaw(), "long_edges"):
            raise StopSafe(f"{step}: not holding a plate on its long edges")
        self._slide(carry, f"{step} slide in")
        self._descend(p, f"{step} lower")
        self._jaw_open("long_edges")
        self._seated_front(step, f"hotel_{slot}")
        self._move("L", carry, f"{step} lift off", prof.approach)
        self._move("L", approach, f"{step} slide out", prof.approach)
        self.store.s.hotel[slot] = self.store.s.plate_id
        self.store.done(plate_at=f"hotel:{slot}", grip="none")
        self._hotel_leave_approach(slot, step)

    def _hotel_pick(self, slot: str):
        prof = self.safety.profile
        p, approach, align, carry = self._hotel_geometry(slot)
        step = f"pick:hotel:{slot}"
        self.store.begin(step)
        self._health()
        junction = self._hotel_junction(slot)
        here = self.robot.pose()
        if max(abs(here[i] - junction[i]) for i in range(3)) > 5.0:
            self._move("J", junction, f"{step} junction", prof.approach)
        foot, top = self._hotel_climb(slot)
        if foot is not None:
            self._move("L", foot, f"{step} retreat", prof.approach)
            self._move("L", top, f"{step} climb", prof.approach)
            self._move("L", approach, f"{step} approach", prof.approach)
        sv = self._slot_view(step, slot, "occupied")
        if sv:
            self.tele.log("vision", f"{step} slot_view", sv.decision, score_empty=round(sv.score_empty, 3),
                          score_occupied=round(sv.score_occupied, 3))
        if sv and sv.decision == "empty":
            self._fault(step, "slot_empty", f"empty {sv.score_empty:.3f} vs occupied {sv.score_occupied:.3f}")
            if self.slot_cfg.get("enforce_pick", False) and not self.commission["no_gates"]:
                if not self.corr.operator_or_stop(step, f"{slot} looks empty", "Plate is in the slot?"):
                    raise StopSafe(f"{slot} looks empty")
        self._jaw_open("long_edges")
        self._slide(carry, f"{step} slide in")                        # same path as the place, reversed
        self._descend(p, f"{step} lower onto plate")
        jaw = self._jaw_close("long_edges")
        err = self.checks.grasp_ok(jaw, "long_edges")
        if err:
            self._fault(step, "grasp", err)
            raise StopSafe(f"{step}: {err}")
        self._move("L", carry, f"{step} lift", prof.approach)
        self._move("L", approach, f"{step} slide out", prof.approach)
        self._check_plate_in_gripper(step, "long_edges")
        expected = self.store.s.hotel.get(slot, "")
        if expected and self.store.s.plate_id and expected != self.store.s.plate_id:
            self.corr.alert(step, f"plate id mismatch: slot had {expected}, scanned {self.store.s.plate_id}")
        self.store.s.hotel.pop(slot, None)
        self.store.done(plate_at="gripper", grip="long_edges")
        self._hotel_leave_approach(slot, step)
        self._waypoints("polar_to_hotel", reverse=True)

    def _choose_slot(self, slot: str) -> str:
        """`random` draws from fixtures.hotel.active_slots, never a slot the state says is occupied."""
        if slot != "random":
            return slot
        h = self.L["fixtures"]["hotel"]
        free = [x for x in h.get("active_slots", ["A1"]) if self.store.s.hotel.get(x) is None]
        if not free:
            raise StopSafe("no free active slot")
        import random
        pick = random.choice(free)
        self.tele.log("run", "slot", f"random draw -> {pick}", free=free)
        return pick

    def _current_hotel_slot(self) -> Optional[str]:
        at = self.store.s.plate_at
        return at.split(":", 1)[1] if at.startswith("hotel:") else None
