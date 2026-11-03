"""The two cameras of the cell, live, at their own aspect ratio (4:3: the IMX219 sensors are 3280x2464; the 16:9 modes are a crop).
Nothing to click. Closes with the window; the setup runs in its own console and never waits for this.

    pythonw gui.py            (the desktop shortcut starts it next to the setup)
    python gui.py --selftest  one frame from each camera and one drawn window, then exits
"""
import base64
import json
import os
import sys
import threading
import time
import tkinter as tk

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from devices.vision_box import VisionClient  # noqa: E402

ASPECT = 4 / 3                    # width / height of both cameras at the full field of view
REQUEST = (1280, 960)             # the full-field 4:3 mode; 1280x720 is a crop of the sensor
PERIOD_S = 0.6
BG, FG, MUTED, BAD = "#0e0f11", "#e6e8eb", "#6b7280", "#d4574a"


class Feed(threading.Thread):
    """One camera: its own connection to the box, newest frame kept, its age known."""

    def __init__(self, host, port, camera, led):
        super().__init__(daemon=True)
        self.camera, self.led = camera, led
        self.client = VisionClient(host, port, log=None)
        self.frame, self.at, self.error = None, 0.0, ""

    def run(self):
        while True:
            t0 = time.time()
            try:
                jpg = self.client.frame(self.camera, REQUEST[0], REQUEST[1], 80, None, self.led)
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    self.frame, self.at, self.error = img, time.time(), ""
            except Exception as e:
                self.error = str(e)[:60] or "no answer"
                time.sleep(2.0)
            time.sleep(max(0.0, PERIOD_S - (time.time() - t0)))


class App:
    def __init__(self, feeds, labels):
        self.feeds, self.root = feeds, tk.Tk()
        self.root.title("Nova Cell")
        self.root.configure(bg=BG)
        self.root.geometry("1040x470")
        self.root.minsize(560, 280)
        self.panels = []
        for i, (feed, label) in enumerate(zip(feeds, labels)):
            frame = tk.Frame(self.root, bg=BG)
            frame.grid(row=0, column=i, sticky="nsew", padx=(14, 7) if i == 0 else (7, 14), pady=(14, 0))
            canvas = tk.Label(frame, bg="#000000", bd=0, highlightthickness=0)
            canvas.pack()
            name = tk.Label(frame, text=label, bg=BG, fg=FG, font=("Segoe UI", 11), anchor="w")
            name.pack(fill="x", pady=(8, 0))
            state = tk.Label(frame, text="", bg=BG, fg=MUTED, font=("Segoe UI", 9), anchor="w")
            state.pack(fill="x")
            self.panels.append((canvas, state))
        self.root.grid_columnconfigure(0, weight=1, uniform="c")
        self.root.grid_columnconfigure(1, weight=1, uniform="c")
        self.root.grid_rowconfigure(0, weight=1)
        self.photos = [None, None]
        self.tick()

    def size(self):
        w = max(self.root.winfo_width(), 560)
        h = max(self.root.winfo_height(), 280)
        pw = (w - 14 * 2 - 14) // 2
        ph = h - 14 - 62                         # the labels under the picture
        pw = int(min(pw, ph * ASPECT))
        return pw, int(pw / ASPECT)

    def tick(self):
        pw, ph = self.size()
        for i, feed in enumerate(self.feeds):
            canvas, state = self.panels[i]
            if feed.frame is not None:
                img = cv2.resize(feed.frame, (pw, ph), interpolation=cv2.INTER_AREA)
                ok, png = cv2.imencode(".png", img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                self.photos[i] = tk.PhotoImage(data=base64.b64encode(png.tobytes()))
                canvas.configure(image=self.photos[i], width=pw, height=ph)
            else:
                canvas.configure(image="", width=pw, height=ph)
            age = time.time() - feed.at if feed.at else None
            if feed.error and (age is None or age > 4):
                state.configure(text=f"no signal  {feed.error}", fg=BAD)
            elif age is None:
                state.configure(text="connecting", fg=MUTED)
            else:
                state.configure(text="live" if age < 3 else f"last picture {age:.0f} s ago", fg=MUTED)
        self.root.after(250, self.tick)


def main():
    v = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))["vision"]
    feeds = [Feed(v["host"], v["port"], v["down_camera"], v.get("led", 0)),
             Feed(v["host"], v["port"], v["front_camera"], int(v.get("front_led", v.get("led", 0))))]
    if "--selftest" in sys.argv:
        for f in feeds:
            jpg = f.client.frame(f.camera, REQUEST[0], REQUEST[1], 80, None, f.led)
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            print(f.camera, img.shape[1], "x", img.shape[0], "aspect", round(img.shape[1] / img.shape[0], 3))
            f.frame, f.at = img, time.time()
        app = App(feeds, ("Gripper camera", "Front camera"))
        app.root.withdraw()
        app.root.update()
        print("window drawn, panel size", app.size())
        app.root.destroy()
        return
    for f in feeds:
        f.start()
    App(feeds, ("Gripper camera", "Front camera")).root.mainloop()


if __name__ == "__main__":
    main()
