"""The check between steps: the controller reports no error, the arm is enabled and not loose, the disk has room. Part of Cycle, see run/cycle.py."""
from __future__ import annotations

import shutil

from run.corrector import StopSafe


class Health:
    """What is checked between steps, before the arm moves again."""

    def _health(self, force: bool = False) -> None:
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
