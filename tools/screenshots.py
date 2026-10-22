"""
Screenshots and the walkthrough recording of the setup program on the simulator, for the manual and the next
session on the cell PC.

    python tools/screenshots.py            -> docs/gui/<step>.png and docs/gui/setup-walkthrough.gif

Starts `python gui.py --sim`, clicks through it over its own JSON endpoints (Setup, Cameras, Teach: a Record,
Test: Test everything), and photographs the page with headless Chrome once per step and once a second while the
test runs. The GIF is assembled with Pillow; no ffmpeg.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "gui")
PORT = 8799
URL = f"http://127.0.0.1:{PORT}/"
CHROME = shutil.which("google-chrome") or shutil.which("chromium") or "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
SIZE = "1400,1250"
GIF_WIDTH = 900


def api(path: str, body=None):
    req = urllib.request.Request(URL + "api/" + path, data=json.dumps(body).encode() if body is not None else None,
                                 headers={"Content-Type": "application/json"}, method="POST" if body is not None else "GET")
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


def wait_idle(timeout_s: float = 300, every=None) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if api("state")["busy"] is None:
            return
        if every:
            every()
        time.sleep(1.0)
    raise TimeoutError("the program stayed busy")


def shot(tab: str, path: str, profile: str) -> str:
    cmd = [CHROME, "--headless=new", "--disable-gpu", "--no-first-run", "--hide-scrollbars", f"--user-data-dir={profile}",
           f"--window-size={SIZE}", "--virtual-time-budget=2500", f"--screenshot={path}", f"{URL}#{tab}"]
    subprocess.run(cmd, capture_output=True, timeout=60)
    return path


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    profile = tempfile.mkdtemp(prefix="chrome_")
    frames_dir = tempfile.mkdtemp(prefix="walkthrough_")
    frames = []
    n = [0]

    def frame(tab: str, label: str = "") -> None:
        p = shot(tab, os.path.join(frames_dir, f"{n[0]:04d}.png"), profile)
        n[0] += 1
        frames.append(p)
        if label:
            shutil.copy(p, os.path.join(OUT, f"{label}.png"))

    srv = subprocess.Popen([sys.executable, os.path.join(ROOT, "gui.py"), "--sim", "--no-browser", "--port", str(PORT)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(50):
            try:
                api("state")
                break
            except Exception:
                time.sleep(0.2)
        wait_idle(30)                                    # discovery
        frame("setup", "setup")
        frame("cameras", "cameras")
        frame("teach", "teach")
        api("record", {"row": "slot:C10"})               # one Record: the row turns from red to ok (C10 alone changes no geometry)
        wait_idle(30)
        frame("teach", "teach-recorded")
        frame("test", "test")
        api("test", {})
        time.sleep(1.0)
        wait_idle(600, every=lambda: frame("test"))
        frame("test", "test-done")
    finally:
        srv.terminate()
        srv.wait(timeout=10)
    imgs = [Image.open(p).convert("RGB") for p in frames]
    w = GIF_WIDTH
    small = [im.resize((w, int(im.height * w / im.width))).quantize(colors=128) for im in imgs]
    gif = os.path.join(OUT, "setup-walkthrough.gif")
    small[0].save(gif, save_all=True, append_images=small[1:], duration=[1500] * 4 + [900] * (len(small) - 5) + [3000], loop=0)
    print(f"{len(frames)} frames -> {gif}")
    for name in ("setup", "cameras", "teach", "teach-recorded", "test", "test-done"):
        print(os.path.join(OUT, f"{name}.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
