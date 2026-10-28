"""The plate hotel: slot geometry, the climb and slide in, pick and place in a slot. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

from typing import List, Optional

from procedures.corrector import StopSafe
from procedures.motion import offset
from procedures.safety import SafetyViolation


class SlotOccupied(Exception):
    """A place found its slot occupied (ledger or camera) and backed out with the plate still held."""

    def __init__(self, slot: str, why: str):
        super().__init__(f"{slot}: {why}")
        self.slot = slot


def interleave(*columns):
    """Round-robin over the columns' slots: A1, C2, A3, C4, ... so consecutive plates come from different
    columns and the arm alternates sides. Shorter columns simply run out first."""
    order = []
    for i in range(max((len(c) for c in columns), default=0)):
        order += [c[i] for c in columns if i < len(c)]
    return order


def plate_order(layout: dict):
    """The slots that hold plates, interleaved across columns: fixtures.hotel.loaded = {"A": [1, 3], "C": [2, 4]}."""
    loaded = layout["fixtures"]["hotel"].get("loaded", {})
    return interleave(*([f"{col}{row}" for row in rows] for col, rows in loaded.items()))


MIN_LIFT_MM = 12.0          # least the plate is raised above its seat to clear the slot's rails


class Hotel:
    """The plate hotel: slot geometry, the climb and slide in, pick and place in a slot."""

    # ------------------------------------------------------------------ hotel
    def hotel_slot_pose(self, slot: str) -> List[float]:
        h = self.L["fixtures"]["hotel"]
        col, row = slot[:1], slot[1:]
        if col not in h["columns"] or not row.isdigit() or int(row) not in h["rows"]:
            raise SafetyViolation(f"slot {slot} does not exist")
        if h.get("pose_source") == "cad":
            return self.cad_poses().hotel_pose(slot)
        if slot in h["taught"]:
            return list(h["taught"][slot])
        a, b, n = h["row_vector_from"]
        rv = [(h["taught"][b][i] - h["taught"][a][i]) / n for i in range(3)]
        base = h["taught"][col + "1"]
        return [base[i] + (int(row) - 1) * rv[i] for i in range(3)] + list(base[3:])

    def cad_poses(self):
        """The poses computed from the CAD (procedures/cad_poses.py); built once per layout."""
        if getattr(self, "_cad", None) is None or self._cad_layout is not self.L:
            from procedures.cad_poses import CadPoses
            self._cad, self._cad_layout = CadPoses(self.L), self.L
        return self._cad

    def _lift_for(self, seat: List[float]) -> float:
        """The carry lift above a seat: fixtures.hotel.lift_mm, less where the workspace ends first (the top row: its seat is
        at 416 and the box ends at 435). A lift under MIN_LIFT_MM does not clear the slot's rails."""
        lift = min(self.L["fixtures"]["hotel"]["lift_mm"], self.safety.box["z"][1] - seat[2] - 1.0)
        if lift < MIN_LIFT_MM:
            raise SafetyViolation(f"the seat at z {seat[2]:.0f} leaves {lift:.0f} mm of lift under the workspace top "
                                  f"({self.safety.box['z'][1]:.0f}); a plate needs {MIN_LIFT_MM:.0f} mm to clear the rails")
        return lift

    def _hotel_geometry(self, slot: str):
        h = self.L["fixtures"]["hotel"]
        p = self.hotel_slot_pose(slot)
        ax, mm = h["approach"]["axis"], h["approach"]["mm"]
        lift = self._lift_for(p)
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

    def _planned(self) -> bool:
        """fixtures.hotel.pose_source = "cad": the slot poses come from the CAD and the way to a slot is planned around the
        hotel's volume with the claw and the plate in it (routes/volumes.py), not replayed from the taught route A."""
        return self.L["fixtures"]["hotel"].get("pose_source") == "cad"

    def _hotel_go_to_approach(self, slot: str, step: str):
        """Route A end -> junction -> (retreat, climb) -> approach point of the slot, all at carry height. Planned: from
        wherever the arm is straight to the approach point, around the hotel."""
        prof = self.safety.profile
        _, approach, _, _ = self._hotel_geometry(slot)
        if self._planned():
            if max(abs(self.robot.pose()[i] - approach[i]) for i in range(3)) <= 5.0:     # resuming at the approach point
                self.resuming = False
                return
            self.go_to(approach, f"hotel approach {slot}")
            return
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
        if self._planned():
            return                                   # the next move plans its own way from the approach point
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
            raise SlotOccupied(slot, "the ledger says it holds a plate")
        self.store.begin(step)
        self._health()
        self._hotel_go_to_approach(slot, step)
        sv = self._slot_view(step, slot, "empty")
        try:
            ready, _ = self.vision.live_detection(self.front_cam, aruco=False, led=self.led)
        except Exception:
            ready = None
        if sv is None and ready is None:                     # nobody looked: not the same as "empty"
            raise StopSafe(f"{step}: no view of {slot}; the front camera and the live detection both failed")
        occupied = sv is not None and sv.decision == "occupied"          # placing onto a plate: always blocks
        if not occupied and (sv is None or sv.decision == "unknown") and ready:
            occupied = not self.corr.operator_or_stop(step, f"{slot} may be occupied (vendor live detection)",
                                                      "Slot is actually empty?")
        if occupied:
            why = f"empty {sv.score_empty:.3f} vs occupied {sv.score_occupied:.3f}" if sv else "vendor live detection"
            self._fault(step, "slot_occupied", why)
            self.store.s.hotel[slot] = self.store.s.hotel.get(slot) or "unknown plate"
            self.store.done()                                 # nothing moved into the slot; the plate is still held
            self._hotel_leave_approach(slot, step)
            raise SlotOccupied(slot, why)
        if self.checks.before_release(self.robot.jaw(), "long_edges"):
            raise StopSafe(f"{step}: not holding a plate on its long edges")
        self._slide(carry, f"{step} slide in")
        self._descend(p, f"{step} lower")
        self._jaw_open("long_edges")
        self._seated_front(step, f"hotel_{slot}")
        self._move("L", carry, f"{step} lift off", prof.approach)
        self._move("L", approach, f"{step} slide out", prof.approach)
        self.store.s.hotel[slot] = self.store.s.plate_id
        self.store.done(plate_at=f"hotel:{slot}", grip="none", home_slot=slot)
        self._hotel_leave_approach(slot, step)

    def _hotel_place_free(self, slot: str):
        """Place in `slot`; if it turns out occupied, back out and use the plate's home slot, then any free slot.
        Stops only when no slot is free."""
        tried = []
        for candidate in [slot, self.store.s.home_slot] + self.L["fixtures"]["hotel"].get("active_slots", []):
            if not candidate or candidate in tried:
                continue
            tried.append(candidate)
            try:
                self._hotel_place(candidate)
                return
            except SlotOccupied as e:
                self.tele.log("run", "slot_occupied", f"{e}; trying another slot", tried=tried)
        raise StopSafe(f"no free slot to place the plate (tried {tried})")

    def _hotel_pick(self, slot: str):
        prof = self.safety.profile
        p, approach, align, carry = self._hotel_geometry(slot)
        step = f"pick:hotel:{slot}"
        self.store.begin(step)
        self._health()
        if self._planned():
            self._hotel_go_to_approach(slot, step)
        else:
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
        if sv and sv.decision == "empty":
            self._fault(step, "slot_empty", f"empty {sv.score_empty:.3f} vs occupied {sv.score_occupied:.3f}")
            if self._enforce(self.slot_cfg, self.slots.ready(slot)):
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
        self.store.done(plate_at="gripper", grip="long_edges", home_slot=slot)
        self._hotel_leave_approach(slot, step)
        if not self._planned():
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
