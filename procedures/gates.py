"""What is checked before and after a move: health, the camera gates, the barcode. Part of Cycle, see procedures/cycle.py."""
from __future__ import annotations

import json
import os
import shutil
import time
from typing import List, Optional

from procedures.corrector import OperatorConfirmed, StopSafe

REFS_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "reference_images")


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
        # 1280x960: the full field at 4:3 (the 16:9 modes are a crop of the sensor), the same size bringup/frames.py and gui.py ask for.
        # The front camera is lit by its own LED so day and night frames match (vision.front_led).
        if led is None:
            led = self.front_led if cam == self.front_cam else self.led
        try:
            return self.frames.frame(cam, 1280, 960, 85, None, led)
        except Exception as e:
            self.tele.log("vision", "frame", f"{cam}: {e}")
            return None

    # ------------------------------------------------- collection: a picture and a verdict in the log, never a stop (2026-11-04)
    def _collect(self, kind: str, station: str, state: str, camera: str, judge: bool = False) -> None:
        """Record only. `kind` occupancy or seat: the camera's freshest frame goes under reference_images/<kind>/<station>/<state>_<stamp>_
        <camera>.jpg with the pose, labelled by what the ledger says (`state`). With `judge`, the occupancy detector's verdict on that
        frame is logged beside the truth, so the morning tells how often they agree. Off with vision.collect = false. Nothing here raises:
        the loop's moves are not gated by any of this; a failure is one log line."""
        if not self.L["vision"].get("collect", True):
            return
        try:
            from devices.vision_box import locating
            cam = self.down_cam if camera == "down" else self.front_cam
            with locating(self.L["vision"]["host"], wait_s=5.0):             # one fresh frame from the box itself (the buffer was stale at the module view, 01:03)
                jpg = self.vision.frame(cam, 1280, 960, 90, None, 0)
            d = os.path.join(REFS_ROOT, kind, station)
            os.makedirs(d, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            with open(os.path.join(d, f"{state}_{stamp}_{camera}.jpg"), "wb") as f:
                f.write(jpg)
            pose = [round(v, 3) for v in self.robot.pose()]
            meta = {"station": station, "state": state, "pose": pose, "view_pose": pose, "size": [1280, 960], "cameras": {camera: cam},
                    "source": "loop", "at": time.strftime("%Y-%m-%d %H:%M:%S")}
            with open(os.path.join(d, f"{state}_{stamp}.json"), "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=1)
            if judge:
                import cv2
                import numpy as np
                from perception.occupancy import Station
                cache = self.__dict__.setdefault("_occ_stations", {})
                st = cache.get(station) or Station.load(station)
                cache[station] = st
                if st.empties:
                    v = st.verdict(cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR))
                    self.tele.log("occupancy", station, v.state, truth=state, agree=(v.state == state), score=round(v.score, 2), noise=round(v.noise, 2))
                    return f"{v.state} ({v.score:.2f} against noise {v.noise:.2f})"
        except Exception as e:
            self.tele.log("collect", station, f"{kind} {state}: not recorded: {str(e)[:80]}")
        return None

    def _vision_correction(self, fixture: dict, label: str) -> List[float]:
        """Vision offset correction is off: every move goes to the taught pose (the box answers 1300 Calibration Not Found)."""
        return [0.0] * 6
