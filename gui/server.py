"""One page and a handful of JSON endpoints on the standard library's http.server. Every action is a POST that
starts a job on the cell; the page polls /api/state and /frame/<camera>."""
from __future__ import annotations

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from gui import bench, cameras, checks, runs, teach

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
TYPES = {".html": "text/html; charset=utf-8", ".css": "text/css", ".js": "application/javascript", ".png": "image/png"}


def full_state(cell) -> dict:
    s = cell.state()
    s["cameras"] = cameras.state(cell)
    s["teach"] = teach.rows(cell)
    s["test"] = checks.state(cell)
    return s


def act(cell, name: str, body: dict) -> dict:
    """Run one action; returns what the page shows. Jobs that move the arm go through cell.start, one at a time."""
    if name == "stop":
        cell.stop()
        return {"ok": True}
    if name == "answer":
        cell.questions.answer(bool(body.get("yes")))
        return {"ok": True}
    if name == "discover":
        return {"ok": cell.start("searching the network", cell.discover)}
    if name == "hotel":
        cell.edit(lambda L: bench.set_hotel_size(L, int(body.get("size", 30))))
        return {"ok": True}
    if name == "loaded":
        cell.edit(lambda L: bench.set_loaded(L, list(body.get("slots", []))))
        return {"ok": True}
    if name == "squid_folder":                     # typed or chosen: remember where Squid's software is
        say = cell.set_squid_folder(str(body.get("folder", "")))
        cell.note(say)
        return {"ok": say.startswith("Squid software found"), "message": say}
    if name == "browse_squid":                     # a folder picker on this PC; the program runs where Squid does
        def pick():
            try:
                import tkinter
                from tkinter import filedialog
                root = tkinter.Tk()
                root.withdraw()
                root.attributes("-topmost", True)
                folder = filedialog.askdirectory(title="Squid software folder", parent=root)
                root.destroy()
            except Exception as e:
                cell.note(f"no folder picker here ({e}); type the folder instead")
                return
            if folder:
                cell.note(cell.set_squid_folder(os.path.normpath(folder)))
        threading.Thread(target=pick, daemon=True, name="browse").start()
        return {"ok": True}
    if not cell.arm.get("ok"):
        return {"ok": False, "error": f"arm not connected: {cell.arm.get('error')}"}
    if name == "read_marker":
        return {"ok": cell.start("reading the marker", lambda: cameras.read_marker(cell, body.get("camera")))}
    if name == "teach_free":
        return {"ok": cell.start("teach-free point", lambda: cameras.teach_free(cell, str(body["step"])))}
    if name == "upload":
        return {"ok": cell.start("uploading to the box", lambda: cameras.upload(cell))}
    if name == "check":
        return {"ok": cell.start("checking the cameras", lambda: cameras.check(cell))}
    if name == "drag":
        return {"ok": cell.start("hand-guiding", lambda: teach.drag(cell, bool(body.get("on"))))}
    if name == "record":
        return {"ok": cell.start("recording", lambda: teach.record(cell, str(body["row"])))}
    if name == "measure":
        return {"ok": cell.start("measuring the grip", lambda: teach.measure(cell, str(body["grip"])))}
    why = cell.ready_to_move()
    if why:
        return {"ok": False, "error": why}
    if name == "capture":
        return {"ok": cell.start("capturing the marker", lambda: cameras.capture(cell, bool(body.get("hand")), body.get("camera")))}
    if name == "replay":
        return {"ok": cell.start("replaying the routes", lambda: teach.replay(cell))}
    if name == "test":
        rounds = body.get("rounds")                   # absent: round after round until STOP
        return {"ok": cell.start("testing everything", lambda: checks.test(cell, int(rounds) if rounds else None))}
    return {"ok": False, "error": f"unknown action {name}"}


_VIEW = {}                                                   # label -> (time, bytes)
_VIEW_LOCK = threading.Lock()
VIEW_SIZE, VIEW_EVERY_S = (1280, 960), 0.4


def view(cell, label: str):
    """The page's picture of a camera. The box's 1280x720 frame (what the checks and the recorder use) cuts the top
    and bottom off the sensor's 4:3 field; this asks for 1280x960, the whole field. One request per camera per
    VIEW_EVERY_S however many pages are open; the buffer's frame stands in when the box does not answer."""
    cam = cell.frames.cameras.get(label)
    if not cam:
        return None
    with _VIEW_LOCK:
        t, data = _VIEW.get(label, (0.0, None))
        if data and time.time() - t < VIEW_EVERY_S:
            return data
        try:
            data = cell.vision.frame(cam, VIEW_SIZE[0], VIEW_SIZE[1], 80, None, cell.frames.leds.get(cam, 0))
        except Exception:
            got = cell.frames.latest(cam)
            data = got[1] if got else data
        if data:
            _VIEW[label] = (time.time(), data)
        return data


class Handler(BaseHTTPRequestHandler):
    server_version = "hcs-workcell-gui"

    @property
    def cell(self):
        return self.server.cell

    def log_message(self, *a):          # the terminal shows the cell's events, not every poll
        pass

    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200):
        self._send(code, json.dumps(obj, default=str).encode("utf-8"), "application/json")

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/":
            return self._file("index.html")
        if u.path.startswith("/static/"):
            return self._file(os.path.basename(u.path))
        if u.path == "/api/state":
            return self._json(full_state(self.cell))
        if u.path.startswith("/view/"):                 # the page's video: the whole sensor field, 4:3
            data = view(self.cell, u.path.split("/", 2)[2])
            return self._send(200, data, "image/jpeg") if data else self._send(503, b"no frame yet", "text/plain")
        if u.path.startswith("/frame/"):
            got = self.cell.frames.by_label(u.path.split("/", 2)[2])
            return self._send(200, got[1], "image/jpeg") if got else self._send(503, b"no frame yet", "text/plain")
        if u.path == "/api/image":
            p = runs.image_path(self.cell, q.get("p", [""])[0])
            if p:
                with open(p, "rb") as f:
                    return self._send(200, f.read(), "image/jpeg")
            return self._send(404, b"no such frame", "text/plain")
        self._send(404, b"not found", "text/plain")

    def _file(self, name: str):
        p = os.path.join(STATIC, name)
        if not os.path.isfile(p):
            return self._send(404, b"not found", "text/plain")
        with open(p, "rb") as f:
            self._send(200, f.read(), TYPES.get(os.path.splitext(name)[1], "application/octet-stream"))

    def do_POST(self):
        u = urlparse(self.path)
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._json({"ok": False, "error": "bad request"}, 400)
        if not u.path.startswith("/api/"):
            return self._send(404, b"not found", "text/plain")
        try:
            out = act(self.cell, u.path[5:], body)
        except Exception as e:
            out = {"ok": False, "error": str(e)}
        if not out.get("ok") and "error" not in out:
            out["error"] = f"busy: {self.cell.busy}"
        self._json(out, 200 if out.get("ok") else 409)


def serve(cell, port: int) -> ThreadingHTTPServer:
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    srv.daemon_threads = True
    srv.cell = cell
    threading.Thread(target=srv.serve_forever, daemon=True, name="http").start()
    return srv
