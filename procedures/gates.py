"""What is checked before and after a move: health, the camera gates, the barcode. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

import shutil
from typing import List, Optional

from procedures.corrector import OperatorConfirmed, StopSafe


class Gates:
    """What is checked before and after a move: health, the camera gates, the barcode."""

    def _health(self, force: bool = False):
        if not force and self.clock.time() - self._last_health < self.lim["health_check_every_s"]:
            return
        self._last_health = self.clock.time()
        st = self.robot.state()
        ctrl_err = self.robot.errors()
        if ctrl_err:
            self.corr.alert("health", f"controller error: {ctrl_err}")
            raise StopSafe(f"controller error {ctrl_err}")
        free_gb = shutil.disk_usage(self.run_dir).free / 1e9
        self.tele.log("health", "check", robot_enabled=st.enabled, drag=st.drag,
                      disk_free_gb=round(free_gb, 1), sdk_stops=getattr(self.robot, "sdk_stops", 0))
        if st.drag:
            raise StopSafe("drag mode turned on during the run")
        if not st.enabled:
            # between steps the arm is stationary; one re-enable per health check, counted by the corrector
            if not self.corr.on_disabled("health") or not self.robot.state().enabled:
                raise StopSafe("controller lost enable between steps and could not be re-enabled")
        if free_gb < self.lim["min_disk_free_gb"]:
            raise StopSafe(f"disk nearly full ({free_gb:.1f} GB free)")

    # ------------------------------------------------------------ pictures (setup only; no camera gate stops a move)
    def _frame(self, cam: str, led: Optional[int] = None) -> Optional[bytes]:
        # 1280x720: the camera's 1080p mode is a different sensor crop, and all gate references are 720p.
        # The front camera is lit by its own LED so day and night frames match (vision.front_led).
        if led is None:
            led = self.front_led if cam == self.front_cam else self.led
        try:
            return self.frames.frame(cam, 1280, 720, 85, None, led)
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

    def _vision_correction(self, fixture: dict, label: str) -> List[float]:
        """Vision offset correction is off: every move goes to the taught pose (the box answers 1300 Calibration Not Found)."""
        return [0.0] * 6
