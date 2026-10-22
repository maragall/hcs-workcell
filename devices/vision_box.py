"""
Pure-Python client for the VIBVision box (Raspberry Pi, TCP text protocol, port 9000).

No vendor code. Every command used by the cell is here, parsed into plain Python values.

Protocol:  <command>,<request_id>,<p1>,...\r
Reply:     success,<request_id>,<payload...>\r
           error,<request_id>,<code>,<message>[,<ng_error_id>]\r

Formats below were confirmed against the live box on 2026-10-14 (service build 20250929).
"""
from __future__ import annotations

import base64
import socket
import threading
import time
import uuid
from dataclasses import dataclass
from typing import List, Optional


class VisionError(Exception):
    def __init__(self, code: str, message: str, ng_id: str = "", command: str = ""):
        super().__init__(f"{command} -> {code}: {message}" + (f" (ng {ng_id})" if ng_id else ""))
        self.code, self.message, self.ng_id, self.command = code, message, ng_id, command


@dataclass
class ArucoPose:
    pixel: List[float]
    camera: List[float]          # x y z rx ry rz in camera frame
    target: List[float]          # x y z rx ry rz (robot frame when calibration is present)
    frame: str
    image: Optional[bytes] = None


@dataclass
class PoseOffset:
    d: List[float]               # dx dy dz drx dry drz
    frame: str
    pos_mag: float
    rot_mag: float
    image: Optional[bytes] = None


@dataclass
class Status:
    state: str
    uptime_ms: int
    cameras: int
    clients: int
    streaming: int
    active_ops: int
    last_error: str = ""


@dataclass
class NgEvent:
    error_id: str
    ts_ms: int
    command: str
    has_image: bool
    camera: str
    message: str


class VisionClient:
    def __init__(self, host: str, port: int = 9000, timeout: float = 30.0, log=None):
        self.host, self.port, self.timeout = host, port, timeout
        self._lock = threading.Lock()
        self._log = log or (lambda *a: None)

    # ------------------------------------------------------------------ core
    def _raw(self, line: bytes) -> bytes:
        with self._lock:
            deadline = time.time() + self.timeout
            s = socket.create_connection((self.host, self.port), timeout=self.timeout)
            try:
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.sendall(line)
                buf = bytearray()
                while True:
                    remaining = deadline - time.time()
                    if remaining <= 0:
                        raise VisionError("timeout", f"no complete reply within {self.timeout}s")
                    s.settimeout(remaining)
                    chunk = s.recv(1 << 20)
                    if not chunk:
                        raise ConnectionError("vision box closed the connection")
                    buf += chunk
                    if buf.endswith(b"\r"):
                        return bytes(buf[:-1])
            finally:
                s.close()

    def request(self, command: str, *params, max_split: int = -1) -> List[bytes]:
        rid = str(uuid.uuid4())
        fields = [command, rid] + ["" if p is None else str(p) for p in params]
        t0 = time.time()
        resp = self._raw((",".join(fields) + "\r").encode("utf-8"))
        dt = time.time() - t0
        head = resp.split(b",", 4)
        status = head[0].decode("ascii", "replace")
        if status == "error":
            code = head[2].decode() if len(head) > 2 else "?"
            msg = head[3].decode("utf-8", "replace") if len(head) > 3 else ""
            ng = head[4].decode("ascii", "replace") if len(head) > 4 else ""
            self._log("vision", command, f"error {code} {msg} {ng} ({dt:.2f}s)")
            raise VisionError(code, msg, ng, command)
        if status != "success":
            raise VisionError("?", resp[:120].decode("utf-8", "replace"), "", command)
        if len(head) > 1 and head[1].decode("ascii", "replace") != rid:
            raise VisionError("id", "request id mismatch", "", command)
        self._log("vision", command, f"ok ({dt:.2f}s, {len(resp)} B)")
        payload = resp.split(b",", 2)[2] if resp.count(b",") >= 2 else b""
        return payload.split(b",", max_split) if payload else []

    @staticmethod
    def _f(parts, i, default=0.0):
        try:
            return float(parts[i])
        except (IndexError, ValueError):
            return default

    @staticmethod
    def _img(b: bytes) -> Optional[bytes]:
        if len(b) < 64:
            return None
        try:
            return base64.b64decode(b)
        except Exception:
            return None

    # -------------------------------------------------------------- commands
    def version(self) -> str:
        return b",".join(self.request("get_version")).decode("utf-8", "replace")

    def status(self) -> Status:
        p = self.request("get_status")
        if not p:
            raise VisionError("empty", "get_status returned no payload", "", "get_status")
        return Status(p[0].decode(), int(self._f(p, 1)), int(self._f(p, 2)), int(self._f(p, 3)),
                      int(self._f(p, 4)), int(self._f(p, 5)), p[6].decode() if len(p) > 6 else "")

    def cameras(self) -> List[str]:
        return [x.decode().split("|")[0] for x in self.request("get_available_cameras") if x]

    def frame(self, camera: str, width=1920, height=1080, quality=85, exposure_us=None, led=0) -> bytes:
        p = self.request("get_frame", width, height, quality, exposure_us or "", camera, led, max_split=0)
        return base64.b64decode(p[0])

    def frame_full(self, camera: str, led=0) -> bytes:
        return self.frame(camera, 3280, 2464, 92, None, led)

    def aruco_pose(self, camera: str, marker_id=1, marker_size_m=None, led=0) -> ArucoPose:
        p = self.request("get_aruco_pose", camera, marker_id, marker_size_m or "", led, max_split=17)
        f = [self._f(p, i) for i in range(16)]
        return ArucoPose(f[0:4], f[4:10], f[10:16], p[16].decode() if len(p) > 16 else "",
                         self._img(p[17]) if len(p) > 17 else None)

    def set_reference(self, camera: str, reference: str, cur_pose, marker_id=1, marker_size_m=None, led=0):
        """Store the current ArUco view as the reference for `reference`. cur_pose = robot pose (6)."""
        return self.request("set_reference_aruco_pose", camera, marker_id, marker_size_m or "", reference,
                            *[f"{v:.4f}" for v in cur_pose], led, max_split=0)

    def pose_offset(self, camera: str, reference: str, workspace="autoteach", marker_id=1,
                    marker_size_m=None, led=0) -> PoseOffset:
        p = self.request("get_pose_offset", camera, marker_id, marker_size_m or "", reference, workspace, led,
                         max_split=9)
        d = [self._f(p, i) for i in range(6)]
        return PoseOffset(d, p[6].decode() if len(p) > 6 else "", self._f(p, 7), self._f(p, 8),
                          self._img(p[9]) if len(p) > 9 else None)

    def live_detection(self, camera: str, aruco=True, led=0):
        """Returns (ready: bool, image bytes). ready==1 means the marker/plate was found."""
        p = self.request("run_live_detection", 1 if aruco else 0, camera, led, max_split=3)
        return (p[1] == b"1") if len(p) > 1 else False, self._img(p[3]) if len(p) > 3 else None

    def scan(self, camera: str, timeout_s=3.0, types="QR;DATAMATRIX;CODE128", led=0, image=False):
        p = self.request("scan", timeout_s, types, camera, "true" if image else "false", led, max_split=3)
        codes = p[1].decode("utf-8", "replace") if len(p) > 1 else ""
        return [c for c in codes.split("|") if c and c != "no_barcode_detected"], (
            self._img(p[3]) if len(p) > 3 else None)

    def led(self, camera: str, brightness: int):
        if brightness <= 0:
            return self.request("turn_off_led", camera)
        return self.request("turn_on_led", camera, brightness)

    def led_status(self):
        p = [x.decode() for x in self.request("get_led_status")]
        out = {}
        for i in range(0, len(p) - 3, 4):
            out[p[i]] = {"brightness": int(float(p[i + 1] or 0)), "auto": p[i + 2] == "true",
                         "manual": p[i + 3] == "true"}
        return out

    def ng_events(self, limit=20, command: str = "", since_ms: Optional[int] = None, camera: str = ""):
        p = [x for x in self.request("get_ng_events", command, limit, since_ms or "", camera)]
        n = int(self._f(p, 0, 0))
        out = []
        for k in range(n):
            b = 1 + k * 8
            if b + 7 >= len(p) + 1:
                break
            try:
                msg = base64.b64decode(p[b + 7] + b"==").decode("utf-8", "replace")
            except Exception:
                msg = ""
            out.append(NgEvent(p[b].decode(), int(self._f(p, b + 1)), p[b + 2].decode(), p[b + 3] == b"1",
                               p[b + 4].decode(), msg))
        return out

    def ng_image(self, error_id: str) -> Optional[bytes]:
        p = self.request("get_ng_image", error_id, max_split=1)
        return self._img(p[0]) if p else None

    # -------------------------------------------------- no-teach workspace API
    def positions(self, workspace="autoteach", ptype="all"):
        import re
        raw = b",".join(self.request("get_position_list", workspace, ptype)).decode("utf-8", "replace")
        out = []
        for m in re.finditer(r"(teach|trigger)\|([^|]*)\|([^|]*)\|([^|]*)\|(\w+)", raw):
            out.append({"type": m.group(1), "name": m.group(2), "id": m.group(3),
                        "pose": [float(v) for v in m.group(4).split(",")], "pick": m.group(5)})
        return out

    def add_teach(self, name, pose, pick="wide", workspace="autoteach"):
        return self.request("add_teach_position", name, *[f"{v:.4f}" for v in pose], pick, workspace, max_split=8)

    def add_trigger(self, name, pose, camera, marker="aruco", marker_mm=150, pick="wide",
                    workspace="autoteach", led=0):
        return self.request("add_trigger_position", name, *[f"{v:.4f}" for v in pose], marker, marker_mm, pick,
                            workspace, camera, led, max_split=8)

    def add_relation(self, trigger_name, teach_name, relation_name, pick="wide", workspace="autoteach"):
        return self.request("add_relation", trigger_name, teach_name, relation_name, pick, workspace)

    def delete_position(self, name, ptype, workspace="autoteach"):
        return self.request("delete_position", workspace, name, ptype)

    def start_vision_teach(self, cur_pose, camera, is_gripper=False, pick="wide", aruco=True, led=0):
        p = self.request("start_vision_teach", *[f"{v:.4f}" for v in cur_pose], 1 if is_gripper else 0, pick,
                         camera, 1 if aruco else 0, led, max_split=14)
        return self._teach_result(p)

    def fine_tune_vision_teach(self, cur_pose, camera, is_gripper=False, aruco=True):
        p = self.request("fine_tune_vision_teach", *[f"{v:.4f}" for v in cur_pose], 1 if is_gripper else 0,
                         camera, 1 if aruco else 0, max_split=14)
        return self._teach_result(p)

    def _teach_result(self, p):
        return {
            "job": p[0].decode() if p else "",
            "pose": [self._f(p, i) for i in range(1, 7)],
            "has_error": p[7] == b"true" if len(p) > 7 else True,
            "ready": p[8] == b"true" if len(p) > 8 else False,
            "finished": p[9] == b"true" if len(p) > 9 else False,
            "progress": int(self._f(p, 10)),
            "image": self._img(p[12]) if len(p) > 12 else None,
            "error": p[13].decode("utf-8", "replace") if len(p) > 13 else "",
        }
