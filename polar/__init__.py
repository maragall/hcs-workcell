"""The driver language: three verbs over places with names.

    from polar import Polar

    arm = Polar()
    arm.move("A1", "microscope")
    arm.move("microscope", "A1")

A place is a hotel slot ("A1" .. "C9"), "module" (the rotation module), or the name of an instrument that was taught
(`python cli.py teach NAME`). `move` picks at the source and places at the destination; `pick` and `place` do one half each.
Each call returns when the arm is done and raises PolarError with the reason when it cannot do it.

Underneath: the loop's own pick and place (procedures/), the stored routes (preset/routes.json), the force close.
The plate is carried by its long edges into and out of the hotel and by its short edges into and out of an instrument; the
rotation module is where the grip changes. `move` does that regrip by itself.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Optional

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

__all__ = ["Polar", "PolarError"]


class PolarError(Exception):
    """The arm did not do what was asked. The message says why; the arm is left where it stopped."""


class Polar:
    def __init__(self, check_occupancy: bool = False, speed: str = "stress", plate_at: Optional[str] = None):
        """check_occupancy: look at a seat with the camera before placing on it (optional, off).
        speed: a profile of layout.json speeds.
        plate_at: where the plate is now, when this is the first run or it was moved by hand: a place name, or "none"."""
        import cli as R
        from devices import nova
        from procedures import motion
        from procedures.corrector import StopSafe
        from procedures.cycle import Cycle
        from procedures.safety import SpeedProfile
        from procedures.state import StateStore
        from procedures.telemetry import Telemetry
        from routes.plan import Route

        self._StopSafe = StopSafe
        self.layout = R.load_layout()
        self.layout["vision"]["check_before_lower"] = bool(check_occupancy)
        runs = os.path.join(R.RUNS, "polar")
        self._tele = Telemetry(runs)
        try:
            self._robot, st = R.connect_robot_readonly(self.layout, self._tele.log)
        except Exception as e:
            raise PolarError(f"the arm is not reachable ({e}): close BioyondWorks and any other program that holds it") from e
        if st.drag:
            self._robot.drag(False)
        if not self._robot.enable():
            raise PolarError("the arm could not be enabled")
        self._store = StateStore(os.path.join(runs, "state.json"))
        self._cyc = Cycle(self.layout, self._robot, nova.vision_box(self.layout, self._tele.log), self._tele, self._store, "supervised", runs, R.REFS)
        self._cyc.safety.profile = SpeedProfile(**self.layout["speeds"][speed])
        self._cyc.corr.ask = lambda q, t=0: "y"
        self._cyc._carry_check = self._carry_by_width
        self._routes = json.load(open(os.path.join(HERE, "preset", "routes.json"), encoding="utf-8"))["routes"]
        self._Route = Route
        motion.plan = self._stored_or_planned(motion.plan)
        self.instruments = [n for n in self.layout["routes"].get("positions", []) if "." not in n and self.layout["fixtures"].get(n, {}).get("pose")]
        if plate_at is not None:
            self._set_plate(plate_at)

    # ------------------------------------------------------------------ the three verbs
    def move(self, source: str, destination: str) -> None:
        """Pick the plate at the source and place it at the destination."""
        self._kind(destination)
        self.pick(source)
        self.place(destination)

    def pick(self, place: str) -> None:
        """Take the plate from a place. A slot is gripped by the long edges, an instrument by the short edges, the module
        by whichever grip its angle is set for (90: long edges, 0: short edges)."""
        kind, s = self._kind(place), self._store.s
        if s.grip != "none":
            raise PolarError(f"the claw already holds a plate ({s.grip}): place it first")
        if self._where() != place:
            raise PolarError(f"no plate at {place}: the plate is at {self._where()}")
        F = self.layout["fixtures"]
        if kind == "slot":
            self._do(lambda: self._cyc._hotel_pick(place))
        elif kind == "module":
            grip = "long_edges" if s.polar_angle == 90 else "short_edges"
            self._pick_module(grip)
        else:
            self._do(lambda: self._cyc._pick(place, F[place]["pose"], F[place], "short_edges", None))

    def place(self, place: str) -> None:
        """Set the plate down at a place. When the grip in hand is not the one the place needs, the plate is first set
        down on the module, the module turned, and the plate taken again by the other edges."""
        kind, s = self._kind(place), self._store.s
        if s.grip == "none":
            raise PolarError("the claw holds no plate: pick one first")
        F = self.layout["fixtures"]
        if kind == "slot":
            if self._store.s.hotel.get(place) is not None:
                raise PolarError(f"{place} already holds a plate")
            self._regrip("long_edges")
            self._do(lambda: self._cyc._hotel_place(place))
        elif kind == "module":
            self._place_module()
        else:
            self._regrip("short_edges")
            self._do(lambda: self._cyc._place(place, F[place]["pose"], F[place], place, None))

    def where(self) -> str:
        """Where the plate is: a place name, "gripper", or "none"."""
        return self._where()

    def close(self) -> None:
        try:
            self._robot.disconnect()
        finally:
            self._tele.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------ underneath
    def _kind(self, place: str) -> str:
        h = self.layout["fixtures"]["hotel"]
        if place in h.get("taught", {}):
            return "slot"
        if place == "module":
            return "module"
        if place in self.instruments:
            return "instrument"
        raise PolarError(f"unknown place {place!r}: a slot ({', '.join(sorted(h.get('taught', {}))[:3])}, ...), 'module', or one of {self.instruments}")

    def _where(self) -> str:
        at = self._store.s.plate_at
        return at[6:] if at.startswith("hotel:") else ("module" if at == "polar" else at)

    def _set_plate(self, place: str) -> None:
        s = self._store.s
        s.step_done, s.grip, s.plate_id, s.hotel = True, "none", "plate", {}
        if place == "none":
            s.plate_at = "none"
        else:
            kind = self._kind(place)
            s.plate_at = f"hotel:{place}" if kind == "slot" else ("polar" if kind == "module" else place)
            if kind == "slot":
                s.hotel = {place: "plate"}
        self._store.save()

    def _do(self, fn) -> None:
        from devices.arm import MotionRefused
        from procedures.hotel import SlotOccupied
        from procedures.safety import SafetyViolation
        try:
            fn()
        except (self._StopSafe, SlotOccupied, SafetyViolation, MotionRefused) as e:
            raise PolarError(str(e)) from e

    def _seat(self, grip: str):
        P = self.layout["fixtures"]["polar"]
        return (P["pose_at_90"]["pose"], "polar@90", 90) if grip == "long_edges" else (P["pose_at_0"]["pose"], "polar@0", 0)

    def _pick_module(self, grip: str) -> None:
        pose, gate, _ = self._seat(grip)
        self._do(lambda: self._cyc._pick("polar", pose, self.layout["fixtures"]["polar"], grip, None, gate=gate))

    def _place_module(self) -> None:
        pose, gate, _ = self._seat(self._store.s.grip)
        self._do(lambda: self._cyc._place("polar", pose, self.layout["fixtures"]["polar"], "polar", None, gate=gate, check=self.layout["vision"]["check_before_lower"]))

    def _regrip(self, grip: str) -> None:
        if self._store.s.grip == grip:
            return
        self._place_module()
        pose, _, deg = self._seat(grip)
        other = self._seat("short_edges" if grip == "long_edges" else "long_edges")[0]
        self._do(lambda: self._cyc._polar_rotate(deg, other))
        self._pick_module(grip)

    def _carry_by_width(self, step: str, grip: str) -> None:
        w, (lo, hi) = self._robot.jaw(), self.layout["gripper"]["accept_band"][grip]
        if not lo <= w <= hi:
            raise self._StopSafe(f"{step}: the jaw reads {w:.2f}, outside {lo}..{hi}: the plate is not in the claw as it was")

    def _stored_or_planned(self, real_plan):
        """A route between two known places is the stored one, replayed; anything else (a few millimetres at a seat) is planned."""
        def near(a, b, tol=1.0):
            return max(abs(x - y) for x, y in zip(a[:3], b[:3])) <= tol

        def plan(layout, safety, here, target, from_name=None, to_name=None, holding=None, heading=None):
            h, f, t = holding or "empty", from_name or "", to_name or ""
            scope = next((n for n in self.instruments if n in t or n == f), None)
            key = None
            if f.startswith("polar.pose_at_") and "hotel" in t:
                key = f"module->{t.split()[-1]}|{h}"
            elif f.startswith("polar.pose_at_") and scope and scope in t:
                key = f"module->{scope}|{h}"
            elif scope and f == scope and "polar" in t:
                key = f"{scope}->module|{h}"
            elif "polar" in t:
                key = next((k for k, v in self._routes.items() if k.endswith(f"->module|{h}") and "poses" in v and "entry" in v and near(v["entry"], here)), None)
            want = self._routes.get(key or "", {})
            if "poses" in want and near(want["poses"][-1], target, 0.6):
                return self._Route("stored", [list(q) for q in want["poses"]], joint_targets={int(i): list(j) for i, j in want["joint_targets"].items()})
            return real_plan(layout, safety, here, target, from_name, to_name, holding=holding, heading=heading)
        return plan
