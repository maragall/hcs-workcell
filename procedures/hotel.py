"""The plate hotel: slot geometry, the climb and slide in, pick and place in a slot. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

import math

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


MOUTH_CLEAR_MM = 5.0         # planned mode: the claw waits this far outside the first part of the hotel before it goes in
MIN_LIFT_MM = 4.0           # least the plate is raised above its seat: the trays are flat, nothing to clear but the surface


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
        if h.get("pose_source") == "grid":              # tools/fit_hotel_grid.py: the 15 hand-taught seats, fitted to one regular grid
            return list(h["taught"][slot])
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
            reg = self.L.get("registration")                  # a layout built from the CAD alone (bringup/scratch.py) carries its frame
            if reg:
                from bringup.scratch import registration_from
                self._cad, self._cad_layout = CadPoses(self.L, registration=registration_from(reg)), self.L
            else:
                self._cad, self._cad_layout = CadPoses(self.L), self.L
        return self._cad

    def _lift_for(self, seat: List[float]) -> float:
        """The carry lift above a seat: fixtures.hotel.lift_mm, less where the workspace ends first, less where the gap above the
        lip ends first. The trays are 8 mm on a 38 mm pitch (STEP) and only the rows in use have one, so the gap above a tray is the pitch times the step between rows, less the tray; the
        slot lip stands lip_mm above the tray, so the plate (with its lid) has to rise above the lip and still go under the next
        tray: lip + clearance <= lift <= gap - plate - clearance."""
        h = self.L["fixtures"]["hotel"]
        rows = sorted(h["rows"])                             # the trays between the rows in use are not there (a plate slid in 25 mm above a seat)
        step = min((b - a for a, b in zip(rows, rows[1:])), default=1)
        gap = h.get("tray_pitch_mm", 38.0) * step - h.get("tray_thickness_mm", 8.0)
        plate = self.L["plate"]
        clear = h.get("tray_clearance_mm", 1.5)
        least = h.get("lip_mm", 3.0) + clear
        fits = gap - (plate.get("height_with_lid") or plate["height"]) - clear
        lift = min(h["lift_mm"], self.safety.box["z"][1] - seat[2] - 1.0, fits)
        if lift < max(least, MIN_LIFT_MM):
            raise SafetyViolation(f"the seat at z {seat[2]:.0f} leaves {lift:.0f} mm of lift (workspace top {self.safety.box['z'][1]:.0f}, "
                                  f"gap between trays {gap:.0f} mm); the plate needs {max(least, MIN_LIFT_MM):.0f} mm to clear the lip")
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
        return self.L["fixtures"]["hotel"].get("pose_source") in ("cad", "grid")

    def stations(self, holding: Optional[str]) -> dict:
        """The stations the cell drives between, for a claw holding `holding` (routes/graph.py): the nest, the hovers of the
        microscopes and the module, and for each hotel slot its approach point and its mouth."""
        st = {}
        nest = self.L.get("routes", {}).get("nest")
        if nest and nest.get("pose"):
            st["nest"] = list(nest["pose"])
        for name in self.L.get("routes", {}).get("positions", []):
            try:
                st[name] = self._position(name)
            except (KeyError, SafetyViolation):
                continue
        h = self.L["fixtures"]["hotel"]
        for slot in h.get("taught", {}):
            if slot[1:].isdigit():
                try:
                    approach = self._hotel_geometry(slot)[1]
                except (KeyError, SafetyViolation):
                    continue
                st[f"approach.{slot}"] = approach
                st[f"mouth.{slot}"] = self._hotel_mouth(approach, holding)
        return st

    def _hotel_mouth(self, approach: List[float], holding: Optional[str] = "now") -> List[float]:
        """Where the claw (the plate in it, or the fingers) is just outside the rack, in line with an approach point: the wrist
        backed along the slot until the front of what the claw carries is clear of the hotel's first part. From the geometry
        (the parts of volumes.obstacles.hotel and the footprint of the claw with what it holds), not a distance chosen by hand.
        Never beyond the approach point itself (an empty claw is outside the rack there already)."""
        from routes.volumes import Volumes
        if holding == "now":
            grip = getattr(getattr(self, "store", None), "s", None) and self.store.s.grip
            holding = grip if grip and grip != "none" else None
        v = Volumes(self.L, self.safety, holding)
        h = v.obstacles["hotel"]
        front = min(q["x"][0] - q.get("margin_mm", 1.0) for q in h["parts"]) if h.get("parts") else h["x"][0]
        ahead = max(r[0][1] for r in v.footprint)
        return [min(approach[0], front - ahead - MOUTH_CLEAR_MM)] + list(approach[1:])

    def _hotel_leg(self, a: List[float], b: List[float], label: str) -> None:
        """One leg between two poses on the slot's line: straight, or by joints where an end is beside the arm's own axis (the B
        column's mouth is), where the controller refuses a straight move. The joints are checked against the hotel's parts and
        the links first; if no path is found nothing moves."""
        from routes.volumes import Volumes
        from routes import plan as _plan
        prof = self.safety.profile
        grip = getattr(getattr(self, "store", None), "s", None) and self.store.s.grip
        near = min(math.hypot(q[0] - _plan.fk.MODEL["x0"], q[1] - _plan.fk.MODEL["y0"]) for q in (a, b)) < _plan.SINGULAR_MM
        if not near:
            self._move("L", b, label, prof.approach)
            return
        v = Volumes(self.L, self.safety, grip if grip and grip != "none" else None)
        jb = v.check_joint_leg(a, b, label, start_actual=True)
        self._move_joints_to(b, label, prof.approach, joints=jb)

    def _hotel_row_change(self, slot: str, step: str, approach: List[float]) -> None:
        """Another row of the same column, from the approach point. There the plate's front is already between two trays, so a z
        move goes through one (2026-11-01: B3 -> B1 descended 76 mm that way). The claw backs out to the mouth, changes height
        there, and goes in again."""
        here = self.robot.pose()
        mouth = self._hotel_mouth(here)
        up = [mouth[0], here[1], approach[2]] + list(here[3:])
        self._hotel_leg(here, mouth, f"{step} back out of the rack")
        self._move("L", up, f"{step} change row outside the rack", self.safety.profile.approach)
        self._hotel_leg(up, approach, f"{step} in to the approach point")
        self.resuming = False

    def _hotel_go_to_approach(self, slot: str, step: str):
        """Route A end -> junction -> (retreat, climb) -> approach point of the slot, all at carry height. Planned: from
        wherever the arm is straight to the approach point, around the hotel."""
        prof = self.safety.profile
        _, approach, _, _ = self._hotel_geometry(slot)
        if self._planned():
            if max(abs(self.robot.pose()[i] - approach[i]) for i in range(3)) <= 5.0:     # resuming at the approach point
                self.resuming = False
                return
            here = self.robot.pose()
            mouth = self._hotel_mouth(approach)
            if mouth[0] < approach[0] - 1.0 and math.hypot(here[0] - approach[0], here[1] - approach[1]) <= 8.0 and abs(here[2] - approach[2]) > 5.0:
                self._hotel_row_change(slot, step, approach)
                return
            if mouth[0] >= approach[0] - 1.0:
                self.go_to(approach, f"hotel approach {slot}")
                return
            # The claw turns, and the plate in it swings, outside the rack, at the height of the slot's approach: one gap between two
            # trays holds one plate, so a turn at any other height with the plate's front in the rack hits a tray. From the
            # mouth the way in is straight along the slot.
            self.go_to(mouth, f"hotel mouth {slot}")
            self._hotel_leg(mouth, approach, f"{step} approach")
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
        """Approach point -> out to the mouth of the rack (planned), or the junction for route A reversed (taught)."""
        prof = self.safety.profile
        if self._planned():
            here = self.robot.pose()
            mouth = self._hotel_mouth(here)
            if mouth[0] < here[0] - 1.0:                     # the plate is partly inside the rack: out of it before any other move
                self._hotel_leg(here, mouth, f"{step} back out of the rack")
            return
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
        # the ledger says the slot is free (checked above); no camera looks at it (the slot occupancy check was removed 2026-11-01)
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
