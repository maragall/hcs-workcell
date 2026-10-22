"""What is checked before and after a move: health, the camera gates, the barcode. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

import math
import shutil
import time
from typing import List, Optional

from devices.vision_box import VisionError
from procedures.corrector import StopSafe


class Gates:
    """What is checked before and after a move: health, the camera gates, the barcode."""

    def _health(self, force: bool = False):
        if not force and time.time() - self._last_health < self.lim["health_check_every_s"]:
            return
        self._last_health = time.time()
        st = self.robot.state()
        vs = None
        deadline = time.time() + self.lim.get("vision_retry_s", 300)
        while vs is None:
            try:
                vs = self.vision.status()
            except Exception as e:
                if time.time() > deadline:
                    self.corr.alert("health", f"vision box unreachable for {self.lim.get('vision_retry_s', 300)} s: {e}")
                    raise StopSafe("vision box unreachable")
                self.tele.log("health", "vision", f"unreachable, retrying: {e}")
                time.sleep(10)
        ctrl_err = self._controller_errors()
        if ctrl_err:
            self.corr.alert("health", f"controller error: {ctrl_err}")
            raise StopSafe(f"controller error {ctrl_err}")
        free_gb = shutil.disk_usage(self.run_dir).free / 1e9
        self.tele.log("health", "check", robot_enabled=st.enabled, drag=st.drag, vision=vs.state, clients=vs.clients,
                      disk_free_gb=round(free_gb, 1), sdk_stops=getattr(self.robot, "sdk_stops", 0))
        if st.drag:
            raise StopSafe("drag mode turned on during the run")
        if not st.enabled:
            # between steps the arm is stationary; one re-enable per health check, counted by the corrector
            if not self.corr.on_disabled("health") or not self.robot.state().enabled:
                raise StopSafe("controller lost enable between steps and could not be re-enabled")
        if free_gb < self.lim["min_disk_free_gb"]:
            raise StopSafe(f"disk nearly full ({free_gb:.1f} GB free)")

    # ------------------------------------------------------------ vision gates
    def _enforce(self, cfg: dict, trained: bool) -> bool:
        """Whether a gate may block a move: its config says true or false, or "auto": once it is trained."""
        if self.commission["no_gates"]:
            return False
        mode = cfg.get("enforce", "auto")
        return trained if mode == "auto" else bool(mode)

    def _frame(self, cam: str, led: Optional[int] = None) -> Optional[bytes]:
        # 1280x720: the camera's 1080p mode is a different sensor crop, and all gate references are 720p.
        # The front camera is lit by its own LED so day and night frames match (vision.front_led).
        if led is None:
            led = self.front_led if cam == self.front_cam else self.led
        try:
            return self.vision.frame(cam, 1280, 720, 85, None, led)
        except Exception as e:
            self.tele.log("vision", "frame", f"{cam}: {e}")
            return None

    def _vision_correction(self, fixture: dict, label: str) -> List[float]:
        ref = fixture.get("aruco_reference")
        if not ref or self.commission["no_vision_correction"]:
            if ref:
                self.tele.log("vision", f"{label} offset", "COMMISSIONING: vision correction disabled")
            return [0.0] * 6
        try:
            off = self.vision.pose_offset(self.down_cam, ref, self.L["vision"]["workspace"],
                                          fixture.get("aruco_marker_id", 1), None, self.led)
        except (VisionError, OSError) as e:
            self.tele.image(f"{label}_vision_error", self._frame(self.down_cam))
            if self.corr.on_vision_error(label, e):
                return [0.0] * 6
            raise StopSafe(f"{label}: no vision correction available")
        self.tele.image(f"{label}_aruco", off.image)
        mag = math.sqrt(off.d[0] ** 2 + off.d[1] ** 2 + off.d[2] ** 2)
        self.tele.log("vision", f"{label} offset", d=[round(v, 2) for v in off.d], mag=round(mag, 2), frame=off.frame)
        self._observe(f"vision_mag:{label}", mag)
        if mag > self.safety.vision_jump_mm:
            self.corr.on_vision_too_far(label, mag)
        return [off.d[0], off.d[1], 0.0, 0.0, 0.0, 0.0]

    def _gate_lower(self, gate: str, state: str, label: str):
        if self.commission["no_gates"]:
            img = self._frame(self.down_cam)
            self.tele.image(f"{label}_lower_nogate", img)
            self.tele.log("vision", f"{label} lower_check", "COMMISSIONING: gate disabled, image recorded only", gate=gate)
            return
        attempts = self.L["vision"]["lower_check"]["retries"] + 1
        for attempt in range(attempts):
            img = self._frame(self.down_cam)
            if img is None:
                raise StopSafe(f"{label}: no frame from {self.down_cam} for the lower gate")
            v = self.lower.verdict(gate, state, img)
            self.tele.image(f"{label}_lower_{attempt}", img)
            self.tele.log("vision", f"{label} lower_check", v.reason, score=round(v.best, 3), thr=round(v.threshold, 3),
                          refs=v.n_refs, safe=v.safe, gate=gate)
            self._observe(f"lower:{gate}:{state}", v.best)
            if v.safe:
                return
            self.corr.on_lower_blocked(gate, attempt, attempts, v)
        raise StopSafe(f"{label}: lower gate exhausted {attempts} attempts without a safe verdict")

    def _seated_front(self, step: str, gate: str):
        """Seated check from the front camera at the release moment, fingers open, before lifting.
        Uses the bottom band of the frame, where the plate's top edge and the holder sit. The first
        `seated_front.auto_refs` releases at each gate become its references; until then the verdict is only
        logged. Blocks once trained (seated_front.enforce "auto"), or as the config says."""
        img = self._frame(self.front_cam)
        self.tele.image(f"{step}_released_front", img)
        self.tele.image(f"{step}_released_down", self._frame(self.down_cam))
        if img is None:
            return
        cfg = self.L["vision"].get("seated_front", {})
        roi = tuple(cfg.get("roi", [0.0, 0.55, 1.0, 1.0]))
        name = "seated_front"
        n = self.lower.count(name, gate)
        if n < cfg.get("auto_refs", 12):
            self.lower.add_reference(name, gate, img)
            self.tele.log("vision", f"{step} seated_front", f"reference {n + 1} collected", gate=gate)
            return
        v = self.lower.verdict(name, gate, img, roi=roi)
        self._observe(f"seated_front:{gate}", v.best)
        enforce = self._enforce(cfg, trained=True)        # trained: the references above are complete
        self.tele.log("vision", f"{step} seated_front", v.reason, score=round(v.best, 3), thr=round(v.threshold, 3),
                      safe=v.safe, gate=gate, enforce=enforce)
        if enforce and not v.safe:
            self._fault(step, "not_seated_front", v.reason)
            if not self.corr.operator_or_stop(step, "front camera: plate does not look seated", "Plate is seated correctly?"):
                raise StopSafe("plate not seated (front camera)")

    def _slot_view(self, step: str, slot: str, expect: str):
        """Front-camera view of the slot from the approach point, with the verdict of the two-class check.
        The cell labels its own references: before a place the slot is empty, before a pick it is occupied.
        The first `min_refs` of each class are taken on consecutive visits; later ones every `spacing_s`, so the
        set spans the day's lighting. A frame that confidently contradicts its label is an alarm, never a lesson."""
        img = self._frame(self.front_cam)
        self.tele.image(f"{step}_slot_view", img)
        if img is None:
            return None
        sv = self.slots.verdict(slot, img)
        other = "occupied" if expect == "empty" else "empty"
        self.tele.log("vision", f"{step} slot_view", sv.reason, decision=sv.decision, confidence=round(sv.confidence, 3),
                      score_empty=round(sv.score_empty, 3), score_occupied=round(sv.score_occupied, 3),
                      ready=self.slots.ready(slot))
        n, cap = self.slots.count(slot, expect), self.slot_cfg.get("auto_refs", 24)
        spaced = n < self.slots.min_refs or self.slots.newest_age_s(slot, expect) >= self.slot_cfg.get("spacing_s", 1800)
        if n < cap and spaced and sv.decision != other:
            self.slots.add_reference(slot, expect, img)
            self.tele.log("vision", f"{step} slot_ref", f"{expect} reference {n + 1} collected", slot=slot)
        return sv

    def _scan_plate(self):
        try:
            codes, img = self.vision.scan(self.front_cam, 3.0, "QR;DATAMATRIX;CODE128", self.led, True)
        except (VisionError, OSError):
            codes, img = [], None
        self.tele.image("plate_barcode", img)
        self.tele.log("vision", "barcode", ",".join(codes) or "none")
        self.store.s.plate_id = codes[0] if codes else ""

    def _check_plate_in_gripper(self, step: str, grip: str):
        img = self._frame(self.front_cam)
        self.tele.image(f"{step}_lifted", img)
        if img is None:
            if self.supervised or self.commission["no_gates"]:
                return
            raise StopSafe(f"{step}: front camera frame failed after lift")
        n = self.lower.count("gripper", grip)
        if n < int(self.L["vision"].get("held_plate_auto_refs", 12)):   # first lifts become the held-plate references
            self.lower.add_reference("gripper", grip, img)
            self.tele.log("vision", f"{step} plate_in_gripper", f"reference {n + 1} collected", grip=grip)
            return
        if self.commission["no_gates"]:
            v = self.lower.verdict("gripper", grip, img)
            self.tele.log("vision", f"{step} plate_in_gripper", v.reason, score=round(v.best, 3), safe=v.safe,
                          refs=v.n_refs, shadow=True)
            self._observe(f"gripper:{grip}", v.best)
            return
        v = self.lower.verdict("gripper", grip, img)
        self.tele.log("vision", f"{step} plate_in_gripper", v.reason, score=round(v.best, 3), safe=v.safe, refs=v.n_refs)
        if v.n_refs < self.lower.min_refs:
            return
        self._observe(f"gripper:{grip}", v.best)
        if not v.safe:
            self._fault(step, "not_held", v.reason)
            if not self.corr.operator_or_stop(step, "plate in gripper does not match references",
                                              "Plate is held correctly?"):
                raise StopSafe("plate not held correctly after lift")
