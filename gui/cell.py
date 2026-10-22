"""
The cell as the screens see it: the two devices, the positions file, one job at a time on the arm, the live
readout at the top of every page, STOP, and the operator questions a procedure may ask.

Nothing is cached from the positions file: every read is positions.load, every change positions.save, so the
procedures (which write learned poses themselves) and the screen never disagree.
"""
from __future__ import annotations

import collections
import os
import threading
import time
import traceback
from typing import Callable, Optional

from procedures import positions
from procedures.cycle import Cycle
from procedures.state import StateStore
from procedures.telemetry import Telemetry

from gui import discover
from gui.frames import FrameSource

READOUT_S = 0.5


def plain_error(e: Exception) -> str:
    """One line for the page. A missing or null value in the positions file would otherwise show as "'pose'" or
    "'NoneType' object is not subscriptable"."""
    if isinstance(e, (KeyError, TypeError, IndexError)):
        return f"a value is missing in layout.json ({type(e).__name__} {e}); `python cli.py positions` and `preflight` list what is not taught"
    return str(e) or type(e).__name__


class Questions:
    """A yes/no the procedure thread waits on while the page shows it. No answer in time means no."""

    def __init__(self):
        self.pending: Optional[dict] = None
        self._answer, self._event, self._lock = "n", threading.Event(), threading.Lock()

    def ask(self, question: str, timeout_s: float) -> str:
        with self._lock:
            self.pending = {"text": question, "since": time.time()}
            self._event.clear()
        answered = self._event.wait(timeout_s)
        with self._lock:
            self.pending = None
            return self._answer if answered else "n"

    def answer(self, yes: bool) -> None:
        with self._lock:
            self._answer = "y" if yes else "n"
            self._event.set()


class Cell:
    def __init__(self, layout_path: str, ref_dir: str, state_dir: str, runs_dir: str, robot, vision, sim: bool):
        self.layout_path, self.ref_dir, self.state_dir, self.runs_dir = layout_path, ref_dir, state_dir, runs_dir
        self.robot, self.vision, self.sim = robot, vision, sim
        self.tele = Telemetry(os.path.join(runs_dir, time.strftime("gui_%Y%m%d_%H%M%S")))
        self.log = self.tele.log
        self.stop_event = threading.Event()
        self.questions = Questions()
        self.notes = collections.deque(maxlen=30)     # (t, text): what the program last did or refused
        self.arm: dict = {"ok": False, "error": "not connected"}
        self.arm_error: Optional[str] = None
        self.vision_error: Optional[str] = None
        self.devices: list = []                       # rows of the Setup table, from discover()
        self.busy: Optional[str] = None               # the running job's label
        self._job: Optional[threading.Thread] = None
        self.run = None                               # gui.runs.Run while plates move
        self._relax = False                           # the arm was loose before the job now starting: loosen it after
        self.checks: dict = {}                        # latest results of the Test screen
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
            self.log("gui", "arm", f"not connected: {e}")
        try:
            self.vision.status()
            self.vision_error = None
        except Exception as e:
            self.vision_error = str(e)
            self.log("gui", "vision", f"not reachable: {e}")
        self.frames.start()
        threading.Thread(target=self._readout, daemon=True, name="readout").start()

    def stop_file_seen(self) -> bool:
        """runs/STOP, the file run_forever.py and cli.py run end on: while a job runs it is a STOP press."""
        if self.busy and os.path.exists(os.path.join(self.runs_dir, "STOP")) and not self.stop_event.is_set():
            self.stop()
            self.note(f"{os.path.join(self.runs_dir, 'STOP')} is present; delete it to run again")
            return True
        return False

    def start_up(self) -> None:
        """Connect both devices (reads only, nothing moves), then fill the Setup table."""
        self.connect()
        self.discover()
        self.loosen_arm()

    def loosen_arm(self) -> None:
        """Default state of the arm: loose (hand-guided). Skipped while the arm is not connected or already loose."""
        from gui import teach
        if self.arm_error is not None or self.arm.get("drag"):
            return
        try:
            teach.loosen(self)
            self.arm = dict(self.arm, drag=True)
        except Exception as e:
            self.note(f"could not leave the arm loose: {plain_error(e)}")

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
        """Fill the Setup table; found addresses go into the positions file."""
        L = self.layout()
        if self.sim:
            st = self.vision.status()
            self.devices = [
                {"name": "Arm", "found": "simulator", "state": "enabled, holding still", "ok": True},
                {"name": "Vision box", "found": "simulator", "state": f"{st.cameras} cameras, video above", "ok": True},
                {"name": "Rotation module", "found": "through the arm", "state": self._stage_words(L), "ok": True},
                {"name": "Microscope", "found": "simulator", "state": "pretends to image", "ok": True}]
            return
        arm, box, squid = discover.arm(L, connected=self.arm_error is None), discover.vision_box(L), discover.squid()
        if "ip" in arm and (arm["ip"] != L["robot"]["ip"] or arm["port"] != L["robot"]["port"]):
            self.edit(lambda L: L["robot"].update(ip=arm["ip"], port=arm["port"]))
        if "cameras" in box and box["host"] != L["vision"]["host"]:       # only a host that answered as a vision box
            self.edit(lambda L: L["vision"].update(host=box["host"]))
        if "ip" in arm and self.arm_error:
            self.robot.ip, self.robot.port = arm["ip"], arm["port"]
            self.connect_arm_again()
        if "host" in box and self.vision_error:
            self.vision.host = box["host"]
            try:
                self.vision.status()
                self.vision_error = None
            except Exception as e:
                self.vision_error = str(e)
        a = self.arm
        self.devices = [
            {"name": "Arm", "found": arm.get("ip", "not found"), "ok": "ip" in arm and a.get("ok", False),
             "state": arm.get("error") or (self.arm_error or self._arm_words(a))},
            {"name": "Vision box", "found": box.get("host", "not found"), "ok": "cameras" in box,
             "state": box.get("error") or f"{box['cameras']} cameras, video above"},
            {"name": "Rotation module", "found": "through the arm", "ok": a.get("ok", False),
             "state": self._stage_words(L) if a.get("ok") else "needs the arm"},
            {"name": "Microscope", "found": squid.get("module", "not found"), "ok": "module" in squid,
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

    # --------------------------------------------------------------------- jobs
    def start(self, label: str, fn: Callable[[], None]) -> bool:
        """One job at a time: the arm is one resource. False when something else is running."""
        if self._job and self._job.is_alive():
            self.note(f"busy: {self.busy}")
            return False
        self.stop_event.clear()
        self.busy = label
        relax, self._relax = self._relax, False        # ready_to_move locked a loose arm for this job

        def work():
            try:
                fn()
            except Exception as e:
                self.log("gui", label, f"failed: {e!r}", trace=traceback.format_exc())
                self.note(f"{label}: {plain_error(e)}")
            finally:
                if relax:                               # the arm's resting state is loose: back to it, fault or not
                    try:
                        self.arm = dict(self.arm, drag=False)
                        self.loosen_arm()
                    except Exception as e:
                        self.note(f"could not leave the arm loose: {plain_error(e)}")
                self.busy = None

        self._job = threading.Thread(target=work, daemon=True, name=label)
        self._job.start()
        return True

    def note(self, text: str) -> None:
        self.notes.append((time.time(), text))
        self.log("gui", "note", text)

    def stop(self) -> None:
        """STOP: every procedure ends at its next step, a pending question is answered no, a run is told to stop."""
        self.stop_event.set()
        self.questions.answer(False)
        if self.run:
            self.run.stop()
        self.note("STOP pressed")

    # ------------------------------------------------------------ a procedure
    def cycle(self, scope: str = "microscope", mode: str = "supervised", tag: str = "gui") -> Cycle:
        """A Cycle for teaching and tests, on a throwaway ledger, taking its frames from the buffer and answering
        its questions on the page."""
        run_dir = os.path.join(self.runs_dir, f"{tag}_{time.strftime('%Y%m%d_%H%M%S')}")
        store = StateStore(os.path.join(run_dir, "state.json"))
        cyc = Cycle(self.layout(), self.robot, self.vision, Telemetry(run_dir), store, mode, run_dir, self.ref_dir,
                    scope=scope, layout_path=self.layout_path, stop_event=self.stop_event, frame_source=self.frames)
        cyc.corr.ask = self.questions.ask
        return cyc

    def ready_to_move(self) -> Optional[str]:
        """Why the arm may not move now, or None."""
        if not self.arm.get("ok"):
            return f"arm not connected: {self.arm.get('error')}"
        if self.arm.get("drag"):                      # the arm starts loose: a step that moves it locks it first
            from gui import teach
            teach.drag(self, False)
            self.arm = dict(self.arm, drag=False)
            self._relax = True
        if not self.arm.get("enabled") and not self.robot.enable():
            return "the arm could not be enabled"
        return None

    def state(self) -> dict:
        last = self.notes[-1] if self.notes else None
        return {"sim": self.sim, "busy": self.busy, "arm": self.arm, "devices": self.devices,
                "message": {"t": time.strftime("%H:%M:%S", time.localtime(last[0])), "text": last[1]} if last else None,
                "question": self.questions.pending,
                "frames": {k: self.frames.latest(c) is not None for k, c in self.frames.cameras.items()}}
