"""The setup, in one window: the two cameras live at their own aspect ratio (4:3, the IMX219 sensors are 3280x2464; the 16:9 modes are a crop),
the step in hand, the log, and the question the setup is asking with a button (Enter does the same). The setup runs as `cli.py setup` in a
child process; this window only shows it and answers it. Stop kills the child: the arm finishes the move it was sent and holds where it is.

    pythonw gui.py [setup arguments]      e.g.  pythonw gui.py --from "grips.toml"      (the desktop shortcut runs this)
    python gui.py --view                  the cameras only, no setup
    python gui.py --selftest              one frame from each camera, one drawn window, one question answered; then exits
"""
import base64
import math
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
from devices.controller import Brcapi  # noqa: E402
from devices.vision_box import VisionClient  # noqa: E402
from tools.hotel_model_html import scene_boxes  # noqa: E402

PYTHON = os.path.join(os.path.dirname(sys.executable), "python.exe") if sys.executable.lower().endswith("pythonw.exe") else sys.executable
ASPECT = 4 / 3                    # width / height of both cameras at the full field of view
REQUEST = (640, 480)              # the full 4:3 field at the size it is shown: one picture pixel per screen pixel, nothing resampled here
MODEL_W = 560                     # the 3D view between the cameras: two 640 frames and this fit a 1920 screen
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


class PosePoll(threading.Thread):
    """The arm's pose, jaw and module angle from the controller's own channel (BRCAPI, port 56788), 5 times a second. It does not touch the
    vendor SDK, so it runs beside the setup; a drag by hand shows the same way as a commanded move."""

    def __init__(self, ip):
        super().__init__(daemon=True)
        self.ip = ip
        self.pose, self.jaw, self.module, self.at, self.error = None, None, None, 0.0, ""

    def run(self):
        """One short connection per reading (connect, two reads, close, about 60 ms), under this PC's lock on the port (devices/portlock.py):
        the port takes one client at a time, and the setup's own readings, error checks and force clamps take the same lock in between.
        While the setup holds it (a clamp, a few seconds) this waits 0.3 s, gives the turn up, and the view says how old it is."""
        while True:
            t0 = time.time()
            b = Brcapi(self.ip, 56788, timeout=2.0, log=None, wait_s=0.3)
            try:
                b.connect()
                p = b.pose_mm_rad()
                j = b.joints()
                ext = list(j[1]) if isinstance(j, (tuple, list)) and len(j) == 2 and isinstance(j[0], (list, tuple)) else list(j)[4:]
                self.pose = [p[0], p[1], p[2], math.degrees(p[5])]
                self.jaw, self.module = (ext[0] if ext else None), (ext[1] if len(ext) > 1 else None)
                self.at, self.error = time.time(), ""
            except ConnectionRefusedError:
                self.error = "the setup has the controller's port"
            except Exception as e:
                self.error = str(e)[:60] or "busy"
            finally:
                try:
                    b.close()
                except Exception:
                    pass
            time.sleep(max(0.05, 0.4 - (time.time() - t0)))

class Model:
    """One fixed view of the cell: the hotel's parts, the pillar, the microscope box, the module; the claw where the arm is, the plate in it
    when the jaw reads a plate width. Orthographic, drawn on a tk canvas: static lines once, the claw on every tick."""
    AZ, EL = math.radians(-38.0), math.radians(27.0)
    COL = {"tray": "#4aa3ff", "post": "#ffb347", "hotel": "#7aa0c4", "other": "#2a3038", "obstacle": "#ff6b6b"}

    def __init__(self, parent, layout, w, h):
        self.w, self.h = w, h
        self.canvas = tk.Canvas(parent, width=w, height=h, bg="#000000", bd=0, highlightthickness=0)
        self.layout = layout
        self.boxes = scene_boxes(layout)
        v = layout["volumes"]
        self.claw = v["claw"]
        self.held = v["held"]
        bands = layout["gripper"].get("accept_band", {})
        self.bands = {k: bands[k] for k in ("short_edges", "long_edges") if k in bands}
        pts = [(x, y, z) for b in self.boxes if b["k"] != "other" for x in b["x"] for y in b["y"] for z in b["z"]]
        pts += [(126, 0, 0), (126, 0, 500), (-80, -350, 0), (300, 650, 0)]
        us = [self._rot(*q) for q in pts]
        u0, u1 = min(q[0] for q in us), max(q[0] for q in us)
        v0, v1 = min(q[1] for q in us), max(q[1] for q in us)
        self.scale = 0.92 * min(w / (u1 - u0), h / (v1 - v0))
        self.ox, self.oy = w / 2 - self.scale * (u0 + u1) / 2, h / 2 + self.scale * (v0 + v1) / 2
        self.route, self.planning = None, None
        self.draw_static()

    def set_route(self, r):
        self.route, self.planning = r, None

    def _rot(self, x, y, z):
        y = -y                                                              # mirrored across the xz plane: the view matches the cell as the operator stands at it (2026-11-03)
        ca, sa, ce, se = math.cos(self.AZ), math.sin(self.AZ), math.cos(self.EL), math.sin(self.EL)
        xr, yr = x * ca - y * sa, x * sa + y * ca
        return xr, z * ce - yr * se, yr * ce + z * se          # screen right, screen up, depth

    def P(self, x, y, z):
        u, v, d = self._rot(x, y, z)
        return self.ox + u * self.scale, self.oy - v * self.scale, d

    @staticmethod
    def _corners(b):
        return [(x, y, z) for z in b["z"] for y in b["y"] for x in b["x"]]

    EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
    FACES = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]

    def wire(self, corners, colour, tag):
        pts = [self.P(*c) for c in corners]
        for a, b in self.EDGES:
            self.canvas.create_line(pts[a][0], pts[a][1], pts[b][0], pts[b][1], fill=colour, tags=tag)

    def solid(self, corners, fill, outline, tag):
        pts = [self.P(*c) for c in corners]
        faces = sorted(self.FACES, key=lambda f: sum(pts[i][2] for i in f))          # far faces first
        for f in faces:
            self.canvas.create_polygon(*[v for i in f for v in pts[i][:2]], fill=fill, outline=outline, tags=tag)

    def draw_static(self):
        self.canvas.delete("static")
        for b in self.boxes:
            if b["k"] == "other" and (b["z"][1] - b["z"][0]) < 3:
                continue
            self.wire(self._corners(b), self.COL[b["k"]], "static")
        F = self.layout["fixtures"]
        try:                                                                # the module: a square where the plate sits at 0 deg
            P0 = F["polar"]["pose_at_0"]["pose"]
            hx = self.held["short_edges"]["x"]
            a = (hx[0] + hx[1]) / 2
            yaw = math.radians(P0[5])
            cx, cy = P0[0] + a * math.cos(yaw), P0[1] + a * math.sin(yaw)
            self.wire(self._corners({"x": [cx - 70, cx + 70], "y": [cy - 70, cy + 70], "z": [P0[2] - 40, P0[2] - 12]}), "#8fd18f", "static")
        except KeyError:
            pass
        x0, y0, _ = self.P(126, 0, 0)
        self.canvas.create_oval(x0 - 4, y0 - 4, x0 + 4, y0 + 4, outline="#9aa7b5", tags="static")
        self.canvas.create_text(8, self.h - 8, anchor="sw", fill="#6b7280", font=("Segoe UI", 9), text="cell model: hotel, pillar, microscope box, module; the claw as the arm reports it", tags="static")

    def _tool_box(self, pose, rect, z):
        """A tool-frame rectangle (x along the fingers, y across) at the wrist pose: world corners."""
        x, y, zw, yaw = pose
        c, s_ = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        out = []
        for dz in z:
            for v in rect["y"]:
                for u in rect["x"]:
                    out.append((x + u * c - v * s_, y + u * s_ + v * c, zw + dz))
        return out

    def draw(self, poll):
        self.canvas.delete("dyn")
        if poll is None or poll.pose is None:
            self.canvas.create_text(self.w / 2, 18, fill="#6b7280", font=("Segoe UI", 10), text=("arm: " + poll.error) if poll and poll.error else "arm: connecting", tags="dyn")
            return
        pose, jaw = poll.pose, poll.jaw
        body, fingers, z = self.claw["body"], self.claw["fingers"], self.claw["z"]
        self.solid(self._tool_box(pose, body, z), "#3a4656", "#9fb3c8", "dyn")
        if jaw is not None:
            half = max(2.0, min(48.0, jaw / 2.0))
            for side in (-1, 1):                                            # two finger blocks, the gap between them is the jaw reading
                rect = {"x": fingers["x"], "y": sorted([side * half, side * 48.0])}
                self.solid(self._tool_box(pose, rect, z), "#4c5a6d", "#9fb3c8", "dyn")
        else:
            self.solid(self._tool_box(pose, fingers, z), "#4c5a6d", "#9fb3c8", "dyn")
        grip = next((k for k, (lo, hi) in self.bands.items() if jaw is not None and lo - 0.6 <= jaw <= hi + 0.6), None)
        if grip and grip in self.held:
            h = self.held[grip]
            self.solid(self._tool_box(pose, {"x": h["x"], "y": h["y"]}, h.get("z", [-10.7, 3.7])), "#d8c76a", "#fff2a8", "dyn")
        if self.route and self.route.get("poses"):
            pts = [self.P(q[0], q[1], q[2]) for q in self.route["poses"]]
            start = self.P(pose[0], pose[1], pose[2])
            prev = start
            for k, q in enumerate(pts):
                col = "#ffb347" if k in self.route.get("joint_legs", []) else "#4c8dff"
                self.canvas.create_line(prev[0], prev[1], q[0], q[1], fill=col, width=2, tags="dyn")
                self.canvas.create_oval(q[0] - 3, q[1] - 3, q[0] + 3, q[1] + 3, outline=col, fill="#000000", tags="dyn")
                prev = q
            self.canvas.create_oval(prev[0] - 6, prev[1] - 6, prev[0] + 6, prev[1] + 6, outline="#ffffff", tags="dyn")
            r = self.route
            how = "taught route" if r.get("taught") else ("stored leg" if r.get("planned_s", 9) < 1.0 else f"planned in {r.get('planned_s')} s")
            self.canvas.create_text(self.w - 8, 8, anchor="ne", fill="#9fb3c8", font=("Consolas", 9), tags="dyn",
                                    text=f"route to {r.get('to')}: {len(r['poses'])} legs, {how}   (orange = joint leg)")
        elif self.planning:
            self.canvas.create_text(self.w - 8, 8, anchor="ne", fill="#ffb347", font=("Consolas", 9), tags="dyn", text=f"planning a route to {self.planning} ...")
        age = time.time() - poll.at
        txt = f"x {pose[0]:.1f}  y {pose[1]:.1f}  z {pose[2]:.1f}  yaw {pose[3]:.1f}   jaw {jaw:.2f}" if jaw is not None else f"x {pose[0]:.1f}  y {pose[1]:.1f}  z {pose[2]:.1f}"
        if poll.module is not None:
            txt += f"   module {poll.module:.1f}"
        self.canvas.create_text(8, 8, anchor="nw", fill="#e6e8eb" if age < 2 else "#d4574a", font=("Consolas", 9), text=txt + ("" if age < 2 else f"   ({age:.0f} s old)"), tags="dyn")


class App:
    def __init__(self, feeds, labels, argv, selftest_child=None, autostart=True, layout=None, poll=None):
        self.feeds, self.argv, self.child_cmd, self.poll = feeds, argv, selftest_child, poll
        self.proc, self.lines, self.prompt, self.exited = None, queue.Queue(), None, None
        self.tail = []
        self.root = tk.Tk()
        r = self.root
        r.title("hcs-workcell")
        r.configure(bg=BG)
        self.cam = CAM
        w = CAM[0] * 2 + MODEL_W + 16 * 4
        r.geometry(f"{w}x{CAM[1] + 420}")
        r.minsize(w, CAM[1] + 420)                 # never smaller than the two full frames
        for col in (0, 1, 2):
            r.grid_columnconfigure(col, weight=1)
        r.grid_rowconfigure(1, minsize=58)            # the step in hand: two lines, always
        r.grid_rowconfigure(2, minsize=70)            # the prompt bar: always there, empty when nothing is asked
        r.grid_rowconfigure(3, weight=1)
        self.photos = [None, None]
        self.drawn = [(0.0, None), (0.0, None)]
        self.panels = []
        for i, label in enumerate(labels):
            f = tk.Frame(r, bg=BG)
            f.grid(row=0, column=i * 2, sticky="n", padx=(16, 8) if i == 0 else (8, 16), pady=(16, 0))      # cameras left and right, the model between
            img = tk.Canvas(f, width=self.cam[0], height=self.cam[1], bg="#000000", bd=0, highlightthickness=0)
            img.pack()
            item = img.create_image(0, 0, anchor="nw")
            row = tk.Frame(f, bg=BG)
            row.pack(fill="x", pady=(8, 0))
            tk.Label(row, text=label, bg=BG, fg=FG, font=("Segoe UI", 11)).pack(side="left")
            state = tk.Label(row, text="", bg=BG, fg=MUTED, font=("Segoe UI", 9))
            state.pack(side="right")
            self.panels.append((img, state, item))
        mf = tk.Frame(r, bg=BG)
        mf.grid(row=0, column=1, sticky="n", padx=8, pady=(16, 0))
        self.model = Model(mf, layout or json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8")), MODEL_W, CAM[1])
        self.model.canvas.pack()
        mrow = tk.Frame(mf, bg=BG)
        mrow.pack(fill="x", pady=(8, 0))
        tk.Label(mrow, text="Cell", bg=BG, fg=FG, font=("Segoe UI", 11)).pack(side="left")
        # the step in hand
        self.now = tk.Label(r, text="", bg=BG, fg=FG, font=("Segoe UI", 12), anchor="nw", justify="left", wraplength=1000, height=2)
        self.now.grid(row=1, column=0, columnspan=3, sticky="ew", padx=16, pady=(16, 0))
        # the question, with its buttons (shown only while the setup waits for an answer)
        self.ask = tk.Frame(r, bg=PANEL)
        self.ask.grid(row=2, column=0, columnspan=3, sticky="ew", padx=16, pady=(10, 0))
        self.ask_text = tk.Label(self.ask, text="", bg=PANEL, fg=FG, font=("Segoe UI", 11), anchor="w", justify="left", wraplength=760)
        self.ask_text.pack(side="left", padx=14, pady=12, fill="x", expand=True)
        self.buttons = tk.Frame(self.ask, bg=PANEL)
        self.buttons.pack(side="right", padx=10)
        # the log
        box = tk.Frame(r, bg=BG)
        box.grid(row=3, column=0, columnspan=3, sticky="nsew", padx=16, pady=(10, 0))
        self.log = tk.Text(box, bg=PANEL, fg=MUTED, font=("Consolas", 9), bd=0, highlightthickness=0, wrap="word", height=8,
                           padx=10, pady=8, state="disabled", cursor="arrow")
        self.log.pack(fill="both", expand=True)
        self.log.tag_configure("bad", foreground=BAD)
        self.log.tag_configure("step", foreground=FG)
        self.log.tag_configure("good", foreground=GOOD)
        self.follow = True                            # the newest line stays in view; off while the reader has scrolled up
        self.root.bind_all("<MouseWheel>", self.on_wheel, add="+")          # Windows sends the wheel to the focused widget, not the one under the pointer
        for ev in ("<ButtonRelease-1>", "<Key>"):
            self.log.bind(ev, lambda e: self.root.after(60, self.check_follow))
        # the bar
        bar = tk.Frame(r, bg=BG)
        bar.grid(row=4, column=0, columnspan=3, sticky="ew", padx=16, pady=12)
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
        self.follow = self.log.yview()[1] >= 0.999          # back at the bottom: following again

    def on_wheel(self, e):
        """A wheel turn anywhere over the log scrolls it and stops the following until the reader is back at the bottom."""
        x, y = self.log.winfo_rootx(), self.log.winfo_rooty()
        if x <= e.x_root <= x + self.log.winfo_width() and y <= e.y_root <= y + self.log.winfo_height():
            self.log.yview_scroll(-1 if e.delta > 0 else 1, "units")
            self.follow = self.log.yview()[1] >= 0.999
            return "break"

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
        if line.startswith("@@ROUTE "):
            try:
                self.model.set_route(json.loads(line[8:]))
            except ValueError:
                pass
            return
        clean = ANSI.sub("", line)
        if not clean.strip():
            return
        self.tail = (self.tail + [clean])[-30:]
        m = STEP.match(clean)
        pm = re.search(r"move_to (.+?)\s+planning a new route", clean)
        if pm:
            self.model.planning = pm.group(1).strip()
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
        self.model.draw(self.poll)
        if self.follow and self.log.yview()[1] < 0.999:       # a resize pushed the end out of view: back to it
            self.log.see("end")
        self.root.after(200, self.tick)


def main():
    lay = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))
    v = lay["vision"]
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
        app = App(feeds, labels, [], selftest_child=child, layout=lay)
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
    poll = PosePoll(lay["robot"]["ip"])
    poll.start()
    App(feeds, labels, argv, autostart="--view" not in sys.argv, layout=lay, poll=poll).root.mainloop()


if __name__ == "__main__":
    main()
