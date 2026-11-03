"""The setup, in one window: the two cameras live at their own aspect ratio (4:3, the IMX219 sensors are 3280x2464; the 16:9 modes are a crop),
the step in hand, the log, and the question the setup is asking with a button (Enter does the same). The setup runs as `cli.py setup` in a
child process; this window only shows it and answers it. Stop kills the child: the arm finishes the move it was sent and holds where it is.

    pythonw gui.py [setup arguments]      e.g.  pythonw gui.py --from "grips.toml"      (the desktop shortcut runs this)
    python gui.py --view                  the cameras only, no setup
    python gui.py --selftest              one frame from each camera, one drawn window, one question answered; then exits
"""
import base64
import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
import tkinter as tk

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from devices.vision_box import VisionClient  # noqa: E402

PYTHON = os.path.join(os.path.dirname(sys.executable), "python.exe") if sys.executable.lower().endswith("pythonw.exe") else sys.executable
ASPECT = 4 / 3                    # width / height of both cameras at the full field of view
REQUEST = (640, 480)              # the full 4:3 field at the size it is shown: one picture pixel per screen pixel, nothing resampled here
CAM = REQUEST                     # (the 16:9 modes are a crop of the sensor; 1280x960 and 3280x2464 show the same field as this)
PERIOD_S = 0.6
BG, PANEL, FG, MUTED, BAD, GOOD, ACCENT = "#0e0f11", "#16181b", "#e6e8eb", "#6b7280", "#d4574a", "#5aa874", "#4c8dff"
ANSI = re.compile(r"\x1b\[[0-9;]*m")
STEP = re.compile(r"^\s{2,}(user|computer|polar|nova|squid) : (.*)$")
RESUME = re.compile(r'--from "([^"]+)"(?: --step (\d+))?')
NO_WINDOW = 0x08000000


def letterbox(img, pw, ph):
    """The picture scaled to fit pw x ph at its own aspect, black bars where it does not fill: a frame is never stretched."""
    h, w = img.shape[:2]
    k = min(pw / w, ph / h)
    sw, sh = max(1, int(w * k)), max(1, int(h * k))
    out = np.zeros((ph, pw, 3), np.uint8)
    x, y = (pw - sw) // 2, (ph - sh) // 2
    out[y:y + sh, x:x + sw] = cv2.resize(img, (sw, sh), interpolation=cv2.INTER_AREA)
    return out


class Feed(threading.Thread):
    """One camera: its own connection to the box, newest frame kept, its age known."""

    def __init__(self, host, port, camera, led):
        super().__init__(daemon=True)
        self.camera, self.led = camera, led
        self.client = VisionClient(host, port, log=None)
        self.frame, self.at, self.error = None, 0.0, ""

    def grab(self):
        jpg = self.client.frame(self.camera, REQUEST[0], REQUEST[1], 80, None, self.led)
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.frame, self.at, self.error = img, time.time(), ""
        return img

    def run(self):
        while True:
            t0 = time.time()
            try:
                self.grab()
            except Exception as e:
                self.error = str(e)[:60] or "no answer"
                time.sleep(2.0)
            time.sleep(max(0.0, PERIOD_S - (time.time() - t0)))


class App:
    def __init__(self, feeds, labels, argv, selftest_child=None, autostart=True):
        self.feeds, self.argv, self.child_cmd = feeds, argv, selftest_child
        self.proc, self.lines, self.prompt, self.exited = None, queue.Queue(), None, None
        self.tail = []
        self.root = tk.Tk()
        r = self.root
        r.title("hcs-workcell")
        r.configure(bg=BG)
        self.cam = CAM
        w = CAM[0] * 2 + 16 * 3
        r.geometry(f"{w}x{CAM[1] + 420}")
        r.minsize(w, CAM[1] + 420)                 # never smaller than the two full frames
        r.grid_columnconfigure(0, weight=1, uniform="c")
        r.grid_columnconfigure(1, weight=1, uniform="c")
        r.grid_rowconfigure(1, minsize=58)            # the step in hand: two lines, always
        r.grid_rowconfigure(2, minsize=70)            # the prompt bar: always there, empty when nothing is asked
        r.grid_rowconfigure(3, weight=1)
        self.photos = [None, None]
        self.drawn = [(0.0, None), (0.0, None)]
        self.panels = []
        for i, label in enumerate(labels):
            f = tk.Frame(r, bg=BG)
            f.grid(row=0, column=i, sticky="n", padx=(16, 8) if i == 0 else (8, 16), pady=(16, 0))
            img = tk.Canvas(f, width=self.cam[0], height=self.cam[1], bg="#000000", bd=0, highlightthickness=0)
            img.pack()
            item = img.create_image(0, 0, anchor="nw")
            row = tk.Frame(f, bg=BG)
            row.pack(fill="x", pady=(8, 0))
            tk.Label(row, text=label, bg=BG, fg=FG, font=("Segoe UI", 11)).pack(side="left")
            state = tk.Label(row, text="", bg=BG, fg=MUTED, font=("Segoe UI", 9))
            state.pack(side="right")
            self.panels.append((img, state, item))
        # the step in hand
        self.now = tk.Label(r, text="", bg=BG, fg=FG, font=("Segoe UI", 12), anchor="nw", justify="left", wraplength=1000, height=2)
        self.now.grid(row=1, column=0, columnspan=2, sticky="ew", padx=16, pady=(16, 0))
        # the question, with its buttons (shown only while the setup waits for an answer)
        self.ask = tk.Frame(r, bg=PANEL)
        self.ask.grid(row=2, column=0, columnspan=2, sticky="ew", padx=16, pady=(10, 0))
        self.ask_text = tk.Label(self.ask, text="", bg=PANEL, fg=FG, font=("Segoe UI", 11), anchor="w", justify="left", wraplength=760)
        self.ask_text.pack(side="left", padx=14, pady=12, fill="x", expand=True)
        self.buttons = tk.Frame(self.ask, bg=PANEL)
        self.buttons.pack(side="right", padx=10)
        # the log
        box = tk.Frame(r, bg=BG)
        box.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=16, pady=(10, 0))
        self.log = tk.Text(box, bg=PANEL, fg=MUTED, font=("Consolas", 9), bd=0, highlightthickness=0, wrap="word", height=8,
                           padx=10, pady=8, state="disabled", cursor="arrow")
        self.log.pack(fill="both", expand=True)
        self.log.tag_configure("bad", foreground=BAD)
        self.log.tag_configure("step", foreground=FG)
        self.log.tag_configure("good", foreground=GOOD)
        self.follow = True                            # the newest line stays in view; off while the reader has scrolled up
        for ev in ("<MouseWheel>", "<ButtonRelease-1>", "<Key>"):
            self.log.bind(ev, lambda e: self.root.after(60, self.check_follow))
        # the bar
        bar = tk.Frame(r, bg=BG)
        bar.grid(row=4, column=0, columnspan=2, sticky="ew", padx=16, pady=12)
        self.status = tk.Label(bar, text="", bg=BG, fg=MUTED, font=("Segoe UI", 9))
        self.status.pack(side="left")
        self.stop_btn = self.button(bar, "Stop", self.stop, BAD)
        self.stop_btn.pack(side="right")
        r.bind("<Return>", lambda e: self.primary())
        r.bind("<Configure>", lambda e: self.now.configure(wraplength=max(e.width - 40, 400)) if e.widget is self.root else None)
        r.protocol("WM_DELETE_WINDOW", self.close)
        if autostart:
            self.start()
        else:
            self.idle()
        self.tick()

    # ------------------------------------------------------------------ widgets
    @staticmethod
    def button(parent, text, command, colour=ACCENT):
        return tk.Button(parent, text=text, command=command, bg=colour, fg="#ffffff", activebackground=colour, activeforeground="#ffffff",
                         relief="flat", bd=0, padx=18, pady=6, font=("Segoe UI", 10), cursor="hand2")

    def show_log(self, text, tag=None):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n", tag)
        self.log.configure(state="disabled")
        if self.follow:
            self.log.see("end")

    def check_follow(self):
        self.follow = self.log.yview()[1] >= 0.999

    # ------------------------------------------------------------------ the child
    def start(self, extra=None):
        args = (extra if extra is not None else self.argv)
        cmd = self.child_cmd or [PYTHON, "-X", "utf8", "-u", "cli.py", "setup"] + list(args)
        env = dict(os.environ, NOVA_GUI="1", PYTHONIOENCODING="utf-8")
        self.exited, self.tail = None, []
        try:
            os.remove(os.path.join(HERE, "runs", "STOP"))        # left by this window's own Stop or an earlier run: a new run starts clean
        except OSError:
            pass
        self.set_prompt(None)
        self.stop_btn.configure(state="normal")
        self.status.configure(text="setup running", fg=MUTED)
        self.proc = subprocess.Popen(cmd, cwd=HERE, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                                     creationflags=NO_WINDOW if os.name == "nt" else 0)
        threading.Thread(target=self.read, args=(self.proc,), daemon=True).start()

    def idle(self):
        """Cameras only: nothing runs until the button is pressed; it starts the real setup (the same as Start over)."""
        self.stop_btn.configure(state="disabled")
        self.status.configure(text="cameras only", fg=MUTED)
        for w in self.buttons.winfo_children():
            w.destroy()
        self.ask_text.configure(text="cameras only: the setup has not been started")
        self.button(self.buttons, "Start setup", lambda: self.start(list(self.argv))).pack(side="right")

    def read(self, proc):
        for raw in iter(proc.stdout.readline, b""):
            self.lines.put(raw.decode("utf-8", "replace").rstrip("\r\n"))
        self.lines.put(("exit", proc.wait()))

    def send(self, text):
        try:
            self.proc.stdin.write((text + "\n").encode("utf-8"))
            self.proc.stdin.flush()
        except Exception:
            pass

    def stop(self):
        """Kill the child and what it started. The arm finishes the move it was sent, then holds."""
        if self.proc and self.proc.poll() is None:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(self.proc.pid)], capture_output=True, creationflags=NO_WINDOW)
            else:
                self.proc.kill()
            try:
                open(os.path.join(HERE, "runs", "STOP"), "w").close()
            except OSError:
                pass
            self.show_log("stopped by you: the arm finishes the move it was sent and holds where it is", "bad")

    def close(self):
        if self.proc and self.proc.poll() is None:
            self.stop()
        self.root.destroy()

    # ------------------------------------------------------------------ the question
    def set_prompt(self, text):
        self.prompt = text
        for w in self.buttons.winfo_children():
            w.destroy()
        if text is None:
            self.ask_text.configure(text="")
            return
        yes_no = "[y/N]" in text
        self.ask_text.configure(text=re.sub(r"^>+\s*", "", text.replace("[y/N]:", "").strip()).rstrip(":"))
        if yes_no:
            self.button(self.buttons, "No", lambda: self.answer("n"), "#3a3f47").pack(side="right", padx=(8, 0))
            self.button(self.buttons, "Yes", lambda: self.answer("y")).pack(side="right")
        else:
            self.button(self.buttons, "Done", lambda: self.answer("")).pack(side="right")

    def primary(self):
        if self.prompt is not None and "[y/N]" not in self.prompt:
            self.answer("")

    def answer(self, text):
        if self.prompt is None:
            return
        self.send(text)
        self.set_prompt(None)

    # ------------------------------------------------------------------ one tick: lines, pictures
    def handle(self, line):
        if line.startswith("@@ASK "):
            self.set_prompt(line[6:])
            return
        clean = ANSI.sub("", line)
        if not clean.strip():
            return
        self.tail = (self.tail + [clean])[-30:]
        m = STEP.match(clean)
        if clean.startswith("== "):
            self.show_log(clean, "step")
            self.now.configure(text=clean[3:])
        elif m:
            self.show_log(clean.strip(), "step")
            self.now.configure(text=f"{m.group(2)}")
        elif "REFUSED" in clean or "FAILED" in clean or "Traceback" in clean or "STOP" in clean:
            self.show_log(clean.strip(), "bad")
        elif clean.strip().startswith("->"):
            self.show_log(clean.strip(), "good")
        else:
            self.show_log(clean.strip())

    def finished(self, code):
        self.exited = code
        self.set_prompt(None)
        self.stop_btn.configure(state="disabled")
        text = {0: "setup done", 2: "setup stopped: see the log", 3: "setup waiting"}.get(code, f"setup ended (exit {code})")
        self.status.configure(text=text, fg=GOOD if code == 0 else BAD)
        hint = next((RESUME.search(t) for t in reversed(self.tail) if RESUME.search(t)), None)
        self.set_prompt_buttons_after_exit(hint)

    def set_prompt_buttons_after_exit(self, hint):
        for w in self.buttons.winfo_children():
            w.destroy()
        label = self.status.cget("text")
        self.ask_text.configure(text=label)
        if hint:
            name, step = hint.group(1), hint.group(2)
            extra = ["--from", name] + (["--step", step] if step else [])
            self.button(self.buttons, f"Resume at {name}", lambda: self.start(extra)).pack(side="right")
        self.button(self.buttons, "Start over", lambda: self.start(list(self.argv)), "#3a3f47").pack(side="right", padx=(0, 8))

    def tick(self):
        try:
            while True:
                item = self.lines.get_nowait()
                if isinstance(item, tuple):
                    self.finished(item[1])
                else:
                    self.handle(item)
        except queue.Empty:
            pass
        for i, feed in enumerate(self.feeds):
            img, state, item = self.panels[i]
            if feed.frame is not None and self.drawn[i] != (feed.at, self.cam):          # a new picture or a new size: draw, otherwise leave it
                pw, ph = self.cam
                h, w = feed.frame.shape[:2]
                frame = feed.frame if (w, h) == (pw, ph) else letterbox(feed.frame, pw, ph)      # as served; fitted only if the box sent another size
                ok, png = cv2.imencode(".png", frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                self.photos[i] = tk.PhotoImage(data=base64.b64encode(png.tobytes()))
                img.itemconfigure(item, image=self.photos[i])
                self.drawn[i] = (feed.at, self.cam)
            age = time.time() - feed.at if feed.at else None
            if feed.error and (age is None or age > 4):
                state.configure(text=f"no signal  {feed.error}", fg=BAD)
            elif age is None:
                state.configure(text="connecting", fg=MUTED)
            else:
                served = "" if (feed.frame.shape[1], feed.frame.shape[0]) == REQUEST else f"   box sent {feed.frame.shape[1]}x{feed.frame.shape[0]}"
                state.configure(text=("live" if age < 3 else f"last picture {age:.0f} s ago") + served, fg=MUTED)
        if self.follow and self.log.yview()[1] < 0.999:       # a resize pushed the end out of view: back to it
            self.log.see("end")
        self.root.after(200, self.tick)


def main():
    v = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))["vision"]
    feeds = [Feed(v["host"], v["port"], v["down_camera"], v.get("led", 0)),
             Feed(v["host"], v["port"], v["front_camera"], int(v.get("front_led", v.get("led", 0))))]
    labels = ("Gripper camera", "Front camera")
    argv = [a for a in sys.argv[1:] if a not in ("--view", "--selftest")]
    if "--selftest" in sys.argv:
        for f in feeds:
            img = f.grab()
            print(f.camera, img.shape[1], "x", img.shape[0], "aspect", round(img.shape[1] / img.shape[0], 3))
        child = [sys.executable, "-u", "-c",
                 "import sys; print('== demo'); print('   user : put the plate on the module'); print('@@ASK      done? Enter: ', flush=True); "
                 "a = sys.stdin.readline(); print('got', repr(a)); print('@@ASK   >>> proceed? [y/N]: ', flush=True); b = sys.stdin.readline(); "
                 "print('got', repr(b)); print('      -> ok'); sys.exit(0 if (a, b) == ('\\n', 'y\\n') else 5)"]
        app = App(feeds, labels, [], selftest_child=child)
        app.root.withdraw()
        deadline, stage = time.time() + 20, 0
        while time.time() < deadline and app.exited is None:
            app.root.update()
            if app.prompt is not None and stage == 0 and "done?" in app.prompt:
                print("question 1:", repr(app.prompt)); app.primary(); stage = 1
            elif app.prompt is not None and stage == 1 and "[y/N]" in app.prompt:
                print("question 2:", repr(app.prompt)); app.answer("y"); stage = 2
            time.sleep(0.05)
        print("child exit", app.exited, "window drawn, panel size", app.cam)
        app.root.destroy()
        return
    for f in feeds:
        f.start()
    App(feeds, labels, argv, autostart="--view" not in sys.argv).root.mainloop()


if __name__ == "__main__":
    main()
