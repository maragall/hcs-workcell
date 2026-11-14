"""
The cell the setup program works on: the arm, the positions file, the live readout of the arm, and the notes
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

from run import positions
from run.telemetry import Telemetry

from setup import discover

READOUT_S = 0.5
CONNECT_TRIES, CONNECT_PAUSE_S = 3, 3.0      # the controller refuses the SDK for some seconds after it was switched on or another program let go of it


class Cell:
    def __init__(self, layout_path: str, state_dir: str, runs_dir: str, robot, sim: bool):
        self.layout_path, self.state_dir, self.runs_dir = layout_path, state_dir, runs_dir
        self.robot, self.sim = robot, sim
        self.tele = Telemetry(os.path.join(runs_dir, time.strftime("setup_%Y%m%d_%H%M%S")))
        self.log = self.tele.log
        self.stop_event = threading.Event()
        self.notes = collections.deque(maxlen=30)     # (t, text): what the program last did or refused
        self.arm: dict = {"ok": False, "error": "not connected"}
        self.arm_error: Optional[str] = None
        self.devices: list = []                       # what discover() found: arm, rotation module, each Squid
        self._reading = False

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
        """Connect the arm (a few tries) and start the live readout. Never raises: `arm_error` says why it is not connected."""
        self.connect_arm_again()
        self._read_arm()                              # a section that moves the arm may be the first thing the run does
        if not self._reading:
            self._reading = True
            threading.Thread(target=self._readout, daemon=True, name="readout").start()

    between_plates_only = False      # the rotation sets this: runs/STOP then ends the run between two plates (sections.py stopped()), not mid-carry

    def stop_file_seen(self) -> bool:
        """runs/STOP, the file run_forever.py and cli.py run end on: while the setup runs it is a STOP at the next step, except during the
        rotation, which reads the file itself between plates (2026-11-03 22:43 it stopped a carry with the plate in the claw)."""
        if os.path.exists(os.path.join(self.runs_dir, "STOP")) and not self.stop_event.is_set():
            if self.between_plates_only:
                return True
            self.stop()
            self.note(f"{os.path.join(self.runs_dir, 'STOP')} is present; delete it to run again")
            return True
        return False

    def stop(self) -> None:
        """STOP: the setup ends at its next step."""
        self.stop_event.set()
        self.note("STOP")

    def _read_arm(self) -> None:
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

    def _readout(self) -> None:
        while True:
            self._read_arm()
            time.sleep(READOUT_S)

    def discover(self) -> None:
        """Find the devices; a found arm address goes into the positions file. Fills `devices`: one row per device with
        name, found, ok, state and, when not ok, fix."""
        L = self.layout()
        if self.sim:
            self.devices = [
                {"name": "Arm", "found": "simulator", "state": "enabled, holding still", "ok": True},
                {"name": "Rotation module", "found": "through the arm", "state": self._stage_words(L), "ok": True},
                {"name": "Microscope", "found": "simulator", "state": "pretends to image", "ok": True}]
            return
        arm = discover.arm(L, connected=self.arm_error is None)
        if "ip" in arm and (arm["ip"] != L["robot"]["ip"] or arm["port"] != L["robot"]["port"]):
            self.edit(lambda L: L["robot"].update(ip=arm["ip"], port=arm["port"]))
        if "ip" in arm and self.arm_error:
            self.robot.ip, self.robot.port = arm["ip"], arm["port"]
            self.connect_arm_again()
            self._read_arm()
        a = self.arm
        ok = "ip" in arm and a.get("ok", False)
        self.devices = [
            {"name": "Arm", "found": arm.get("ip", "not found"), "ok": ok,
             "state": arm.get("error") or (self.arm_error or self._arm_words(a)), "fix": "" if ok else self.arm_fix(arm)},
            {"name": "Rotation module", "found": "through the arm", "ok": a.get("ok", False),
             "state": self._stage_words(L) if a.get("ok") else "needs the arm"}]
        for name, got in discover.squid(L).items():
            exec_words = {True: "Python Exec on", False: "Python Exec off (teach and the plate moves switch it on: Squid asks, click Yes)", None: ""}[got.get("python_exec")]
            self.devices.append({"name": f"Squid {name}", "found": got["at"], "ok": "error" not in got, "squid": name,
                                 "state": got.get("error") or ", ".join(w for w in ("answers", "acquiring" if got.get("acquiring") else "idle", exec_words) if w),
                                 "fix": "" if "error" not in got else "start Squid on this PC, then in the Squid window: Settings > Enable MCP Control Server"})

    def arm_fix(self, found: dict) -> str:
        """What to do about an arm that is not connected, from where it failed: the SDK does not load on this PC, no controller
        answers on the network, or the controller answers and refuses this program."""
        why = self.arm_error or ""
        if "not importable" in why:
            return ("the vendor's arm package does not load on this PC: run setup\\install.bat with the vendor .whl file, then "
                    "`python cli.py doctor` names what is missing (the .whl, pythonnet, or the .NET Desktop Runtime 8)")
        if "ip" not in found:
            return found.get("fix", "power the arm and check the network cable")
        return ("the controller answers but does not let this program in: it takes one program at a time. Close BioyondWorks, RobStudio "
                "and any other window of this program, wait ten seconds, and try again")

    def connect_arm_again(self) -> None:
        """Up to CONNECT_TRIES tries. A package that does not load is not tried again: waiting does not install it."""
        for attempt in range(CONNECT_TRIES):
            try:
                self.robot.connect()
                self.arm_error = None
                return
            except Exception as e:
                self.arm_error = str(e)
                self.log("setup", "arm", f"not connected (try {attempt + 1} of {CONNECT_TRIES}): {e}")
                if "not importable" in self.arm_error or attempt == CONNECT_TRIES - 1:
                    return
                try:
                    self.robot.disconnect()
                except Exception:
                    pass
                time.sleep(CONNECT_PAUSE_S)

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
            from setup import hand
            hand.drag(self, False)
        if not self.arm.get("enabled") and not self.robot.enable():
            return "the arm could not be enabled"
        return None
