"""
One frame source for the whole program: a thread per camera asks the vision box for a frame at a fixed rate,
the same size and quality the checks use (1280x720 JPEG 85), into a short ring buffer. The page shows the
newest frame as video, the recorder writes it during runs, and the checks take it instead of asking the box
(Cycle(frame_source=...)), so the video never freezes while a check runs.
"""
from __future__ import annotations

import collections
import threading
import time
from typing import Dict, Optional, Tuple

SIZE, QUALITY = (1280, 720), 85


class FrameSource:
    def __init__(self, vision, cameras: Dict[str, str], leds: Dict[str, int], hz: float = 2.0, keep: int = 20,
                 max_age_s: float = 5.0, log=None):
        """cameras: label -> camera name on the box ({"down": ..., "front": ...}); leds: camera name -> LED level."""
        self.vision, self.cameras, self.leds = vision, cameras, leds
        self.period, self.max_age_s = 1.0 / hz, max_age_s
        self._log = log or (lambda *a: None)
        self._buf: Dict[str, collections.deque] = {c: collections.deque(maxlen=keep) for c in cameras.values()}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []
        self.errors: Dict[str, str] = {}

    def start(self):
        for cam in self.cameras.values():
            t = threading.Thread(target=self._loop, args=(cam,), daemon=True, name=f"frames:{cam}")
            t.start()
            self._threads.append(t)

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)

    def _loop(self, cam: str):
        while not self._stop.is_set():
            t0 = time.time()
            try:
                data = self.vision.frame(cam, SIZE[0], SIZE[1], QUALITY, None, self.leds.get(cam, 0))
                with self._lock:
                    self._buf[cam].append((t0, data))
                    self.errors.pop(cam, None)
            except Exception as e:
                with self._lock:
                    self.errors[cam] = str(e)
            self._stop.wait(max(0.0, self.period - (time.time() - t0)))

    def latest(self, cam: str) -> Optional[Tuple[float, bytes]]:
        with self._lock:
            buf = self._buf.get(cam)
            return buf[-1] if buf else None

    def frame(self, cam: str, *args, **kwargs) -> bytes:
        """vision_box.frame's signature, answered from the buffer: the freshest frame, never older than max_age_s."""
        got = self.latest(cam)
        if got is None or time.time() - got[0] > self.max_age_s:
            raise RuntimeError(f"no fresh frame from {cam}: {self.errors.get(cam, 'not polled yet')}")
        return got[1]

    def by_label(self, label: str) -> Optional[Tuple[float, bytes]]:
        cam = self.cameras.get(label)
        return self.latest(cam) if cam else None
