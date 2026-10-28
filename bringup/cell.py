"""
The cell the setup program works on: the two devices, the positions file, the live readout of the arm, and the notes
of what the program did or refused.

Nothing is cached from the positions file: every read is positions.load, every change positions.save, so the
procedures (which write learned poses themselves) and the setup never disagree.
"""
from __future__ import annotations

import collections
import os
import threading
import time
from typing import Callable, Optional

from procedures import positions
from procedures.telemetry import Telemetry

from bringup import discover
from bringup.frames import FrameSource

READOUT_S = 0.5


class Cell:
    def __init__(self, layout_path: str, ref_dir: str, state_dir: str, runs_dir: str, robot, vision, sim: bool):
        self.layout_path, self.ref_dir, self.state_dir, self.runs_dir = layout_path, ref_dir, state_dir, runs_dir
        self.robot, self.vision, self.sim = robot, vision, sim
        self.tele = Telemetry(os.path.join(runs_dir, time.strftime("setup_%Y%m%d_%H%M%S")))
        self.log = self.tele.log
        self.stop_event = threading.Event()
        self.notes = collections.deque(maxlen=30)     # (t, text): what the program last did or refused
        self.arm: dict = {"ok": False, "error": "not connected"}
        self.arm_error: Optional[str] = None
        self.vision_error: Optional[str] = None
        self.devices: list = []                       # what discover() found: arm, vision box, rotation module, microscope
        L = self.layout()
        v = L["vision"]
        self.frames = FrameSource(vision, {"down": v["down_camera"], "front": v["front_camera"]},
                                  {v["down_camera"]: v["led"], v["front_camera"]: int(v.get("front_led", v["led"]))},
                                  log=self.log)

    # ------------------------------------------------------------ positions file
    def layout(self) -> dict:
        return positions.load(self.layout_path)

    def save(self, layout: dict) -> None:
        positions.save(layout, self.layout_path)

    def edit(self, fn: Callable[[dict], None]) -> dict:
        L = self.layout()
        fn(L)
        self.save(L)
        return L

    # ------------------------------------------------------------------ devices
    def connect(self) -> None:
        try:
            self.robot.connect()
            self.arm_error = None
        except Exception as e:
            self.arm_error = str(e)
            self.log("setup", "arm", f"not connected: {e}")
        for attempt in range(3):                      # the box can be slow to accept; discover() then looks wider
            try:
                self.vision.status()
                self.vision_error = None
                break
            except Exception as e:
                self.vision_error = str(e)
                self.log("setup", "vision", f"not reachable (try {attempt + 1} of 3): {e}")
        self.frames.start()
        threading.Thread(target=self._readout, daemon=True, name="readout").start()

    def stop_file_seen(self) -> bool:
        """runs/STOP, the file run_forever.py and cli.py run end on: while the setup runs it is a STOP."""
        if os.path.exists(os.path.join(self.runs_dir, "STOP")) and not self.stop_event.is_set():
            self.stop()
            self.note(f"{os.path.join(self.runs_dir, 'STOP')} is present; delete it to run again")
            return True
        return False

    def stop(self) -> None:
        """STOP: the setup ends at its next step."""
        self.stop_event.set()
        self.note("STOP")

    def _readout(self) -> None:
        while True:
            self.stop_file_seen()
            if self.arm_error is None:
                try:
                    st, p = self.robot.state(), self.robot.pose()
                    self.arm = {"ok": True, "pose": [round(v, 1) for v in p], "jaw": round(self.robot.jaw(), 2),
                                "enabled": st.enabled, "moving": st.moving, "drag": st.drag}
                except Exception as e:
                    self.arm = {"ok": False, "error": str(e)}
            else:
                self.arm = {"ok": False, "error": self.arm_error}
            time.sleep(READOUT_S)

    def discover(self) -> None:
        """Find the devices; found addresses go into the positions file."""
        L = self.layout()
        if self.sim:
            st = self.vision.status()
            self.devices = [
                {"name": "Arm", "found": "simulator", "state": "enabled, holding still", "ok": True},
                {"name": "Vision box", "found": "simulator", "state": f"{st.cameras} cameras, video above", "ok": True},
                {"name": "Rotation module", "found": "through the arm", "state": self._stage_words(L), "ok": True},
                {"name": "Microscope", "found": "simulator", "state": "pretends to image", "ok": True}]
            return
        arm, box, squid = discover.arm(L, connected=self.arm_error is None), discover.vision_box(L), discover.squid(L)
        if "ip" in arm and (arm["ip"] != L["robot"]["ip"] or arm["port"] != L["robot"]["port"]):
            self.edit(lambda L: L["robot"].update(ip=arm["ip"], port=arm["port"]))
        if "cameras" in box and (box["host"], box["port"]) != (L["vision"]["host"], L["vision"]["port"]):   # only one that answered as a vision box
            self.edit(lambda L: L["vision"].update(host=box["host"], port=box["port"]))
        if "ip" in arm and self.arm_error:
            self.robot.ip, self.robot.port = arm["ip"], arm["port"]
            self.connect_arm_again()
        if "host" in box and self.vision_error:
            self.vision.host, self.vision.port = box["host"], box["port"]
            try:
                self.vision.status()
                self.vision_error = None
            except Exception as e:
                self.vision_error = str(e)
        a = self.arm
        self.devices = [
            {"name": "Arm", "found": arm.get("ip", "not found"), "ok": "ip" in arm and a.get("ok", False),
             "state": arm.get("error") or (self.arm_error or self._arm_words(a))},
            {"name": "Vision box", "found": f"{box['host']}:{box['port']}" if "host" in box else "not found", "ok": "cameras" in box,
             "state": box.get("error") or f"{box['cameras']} cameras, video above"},
            {"name": "Rotation module", "found": "through the arm", "ok": a.get("ok", False),
             "state": self._stage_words(L) if a.get("ok") else "needs the arm"},
            {"name": "Microscope", "found": squid.get("folder", "not found"), "ok": "module" in squid, "browse": bool(squid.get("browse")),
             "state": squid.get("error", "software present") if "error" in squid else
             "software present; the arm pretends to image until Squid is wired in"}]

    def connect_arm_again(self) -> None:
        try:
            self.robot.connect()
            self.arm_error = None
        except Exception as e:
            self.arm_error = str(e)

    @staticmethod
    def _arm_words(a: dict) -> str:
        if not a.get("ok"):
            return a.get("error", "not connected")
        if a.get("drag"):
            return "loose, hand-guided"
        return ("enabled" if a.get("enabled") else "not enabled") + (", moving" if a.get("moving") else ", holding still")

    def _stage_words(self, L: dict) -> str:
        try:
            a = L["polar"]["ext_axis"]
            deg = (self.robot.ext_axis(int(a["index"])) - (a.get("zero_reading") or 0.0)) * a["direction"]
            return f"reads {deg:.1f} deg"
        except Exception as e:
            return f"no reading: {e}"

    def note(self, text: str) -> None:
        self.notes.append((time.time(), text))
        self.log("setup", "note", text)

    def ready_to_move(self) -> Optional[str]:
        """Why the arm may not move now, or None. A loose arm is locked and enabled first."""
        if not self.arm.get("ok"):
            return f"arm not connected: {self.arm.get('error')}"
        if self.arm.get("drag"):
            from bringup import hand
            hand.drag(self, False)
        if not self.arm.get("enabled") and not self.robot.enable():
            return "the arm could not be enabled"
        return None
