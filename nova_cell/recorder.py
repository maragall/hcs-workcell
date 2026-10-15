"""
Continuous recording of both cameras and the operator screen during a run.

Frames go to <run_dir>/video/<source>/NNNNNN.jpg with a timestamp index file, so a run can be
replayed next to events.jsonl. `to_mp4()` assembles them with ffmpeg when it is installed.
"""
from __future__ import annotations

import io
import json
import os
import shutil
import subprocess
import threading
import time
from typing import Dict, List, Optional


class Recorder:
    def __init__(self, run_dir: str, vision, cameras: List[str], fps: float = 2.0, screen: bool = True,
                 size=(1280, 720), quality: int = 75, log=None):
        self.dir = os.path.join(run_dir, "video")
        self.vision, self.cameras, self.fps, self.screen = vision, cameras, fps, screen
        self.size, self.quality = size, quality
        self._log = log or (lambda *a: None)
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._index: Dict[str, list] = {}

    def start(self):
        for cam in self.cameras:
            self._spawn(cam, lambda c=cam: self.vision.frame(c, self.size[0], self.size[1], self.quality))
        if self.screen:
            self._spawn("screen", self._grab_screen)
        self._log("record", "start", f"{', '.join(self._index)} at {self.fps} fps -> {self.dir}")

    def _spawn(self, name: str, grab):
        d = os.path.join(self.dir, name)
        os.makedirs(d, exist_ok=True)
        self._index[name] = []
        t = threading.Thread(target=self._loop, args=(name, d, grab), daemon=True)
        t.start()
        self._threads.append(t)

    def _loop(self, name: str, d: str, grab):
        period = 1.0 / self.fps
        n = 0
        failures = 0
        while not self._stop.is_set():
            t0 = time.time()
            try:
                data = grab()
                if data:
                    with open(os.path.join(d, f"{n:06d}.jpg"), "wb") as f:
                        f.write(data)
                    self._index[name].append((n, round(t0, 3)))
                    n += 1
                    failures = 0
            except Exception as e:
                failures += 1
                if failures in (1, 10, 100):
                    self._log("record", name, f"grab failed x{failures}: {e}")
                time.sleep(min(5.0, 0.5 * failures))
            self._stop.wait(max(0.0, period - (time.time() - t0)))

    def _grab_screen(self) -> Optional[bytes]:
        from PIL import ImageGrab
        im = ImageGrab.grab()
        im.thumbnail(self.size)
        b = io.BytesIO()
        im.convert("RGB").save(b, "JPEG", quality=self.quality)
        return b.getvalue()

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=5)
        for name, idx in self._index.items():
            with open(os.path.join(self.dir, name, "index.json"), "w", encoding="utf-8") as f:
                json.dump(idx, f)
        self._log("record", "stop", ", ".join(f"{k}:{len(v)}" for k, v in self._index.items()))

    @staticmethod
    def to_mp4(video_dir: str, fps: float = 2.0) -> List[str]:
        ff = shutil.which("ffmpeg")
        out = []
        for name in sorted(os.listdir(video_dir)):
            d = os.path.join(video_dir, name)
            if not os.path.isdir(d):
                continue
            target = os.path.join(video_dir, f"{name}.mp4")
            if ff:
                subprocess.run([ff, "-y", "-loglevel", "error", "-framerate", str(fps), "-i",
                                os.path.join(d, "%06d.jpg"), "-c:v", "libx264", "-pix_fmt", "yuv420p", target],
                               check=False)
                out.append(target)
        return out
