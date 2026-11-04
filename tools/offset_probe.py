"""The vision box's offset correction, end to end, with the box's own log for each step. Read-only unless --set-reference.

  python tools/offset_probe.py                         ArUco pose from the down camera, then get_pose_offset against --reference
  python tools/offset_probe.py --set-reference NAME    first store the current view as reference NAME (writes the box), then the offset
  python tools/offset_probe.py --camera front

What it settles (2026-11-03): the box's locating calls force the camera to 3280x2464 and hang while the window keeps asking for 640x480
frames; this runs under the vision lock (devices/vision_box.locating), which pauses the window's pictures. With the lock the calls answer,
and the box's log (fetched here over the protocol, no ssh) says why an offset is refused: 1300 "Calibration Not Found" names the
camera's calibration files, 1002 "Operation Failed" is no marker in view, a missing reference is reported by name. The marker is the
40 mm tag of the practice plate (layout vision.marker_mm); the robot pose for a reference comes from the controller's JSON port, not the SDK,
so this runs beside a setup.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from devices.controller import Brcapi  # noqa: E402
from devices.vision_box import VisionClient, VisionError, locating  # noqa: E402


def robot_pose(ip: str):
    b = Brcapi(ip, log=None, wait_s=6.0)
    b.connect()
    try:
        p = b.pose_mm_rad()
        return [p[0], p[1], p[2], math.degrees(p[3]), math.degrees(p[4]), math.degrees(p[5])]
    finally:
        b.close()


def box_log(v: VisionClient, since_s: float, lines: int = 400) -> str:
    """The box's newest log, the lines about locating since this run started (the box's clock is not this PC's: matched by content)."""
    try:
        name = v.logs(1)[0]["name"]
        text = v.log_tail(name, lines)
    except Exception as e:
        return f"(no log from the box: {e})"
    keep = [l for l in text.splitlines() if re.search(r"pose_offset|aruco|ArUco|reference|参考|标定|calib|ERROR|WARN|error|1300|1002", l)]
    return "\n".join("   " + l[:200] for l in keep[-40:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="down", help="down (the gripper camera) or front")
    ap.add_argument("--reference", default="aruco_reference")
    ap.add_argument("--workspace", default=None, help="default: layout vision.workspace")
    ap.add_argument("--set-reference", metavar="NAME", help="store the current view as this reference (writes the box), then measure the offset to it")
    ap.add_argument("--size-mm", type=float, default=None, help="marker size, default layout vision.marker_mm")
    a = ap.parse_args()
    L = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))
    vis = L["vision"]
    cam = vis["down_camera"] if a.camera == "down" else vis["front_camera"]
    ws = a.workspace or vis.get("workspace", "autoteach")
    size_m = (a.size_mm or float(vis.get("marker_mm", 40.0))) / 1000.0
    v = VisionClient(vis["host"], vis["port"], timeout=150)
    t_run = time.time()
    st = v.status()
    print(f"box: {st.state}, {st.clients} clients, {st.cameras} cameras; camera {cam}, workspace {ws}, marker {size_m * 1000:.0f} mm")
    with locating(vis["host"], wait_s=20.0):
        t = time.time()
        try:
            r = v.aruco_pose(cam, marker_size_m=size_m, led=int(vis.get("led", 0)))
            print(f"get_aruco_pose: target {[round(x, 2) for x in r.target]} frame {r.frame} pixel {[round(x, 1) for x in r.pixel[:2]]} ({time.time() - t:.1f} s)")
        except VisionError as e:
            print(f"get_aruco_pose: error {e.code} {e.message} ({time.time() - t:.1f} s)")
        ref = a.reference
        if a.set_reference:
            pose = robot_pose(L["robot"]["ip"])
            print("robot pose", [round(x, 2) for x in pose])
            t = time.time()
            try:
                out = v.set_reference(cam, a.set_reference, pose, marker_size_m=size_m, led=int(vis.get("led", 0)))
                print(f"set_reference_aruco_pose {a.set_reference}: {b','.join(out)[:160]!r} ({time.time() - t:.1f} s)")
                ref = a.set_reference
            except VisionError as e:
                print(f"set_reference_aruco_pose: error {e.code} {e.message} ({time.time() - t:.1f} s)")
        t = time.time()
        try:
            o = v.pose_offset(cam, ref, workspace=ws, marker_size_m=size_m, led=int(vis.get("led", 0)))
            print(f"get_pose_offset vs {ref}: d {[round(x, 2) for x in o.d]} frame {o.frame} |pos| {o.pos_mag:.2f} |rot| {o.rot_mag:.2f} ({time.time() - t:.1f} s)")
        except VisionError as e:
            print(f"get_pose_offset vs {ref}: error {e.code} {e.message} ({time.time() - t:.1f} s)")
    print("the box's log, locating lines:")
    print(box_log(v, t_run))


if __name__ == "__main__":
    main()
