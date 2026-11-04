"""Torque balancing, step 1: the bench curve. The plate sits in the module's short-edge seat (module at 0, held by the clips); the arm
clamps it from the taught pose and from known sideways shifts (across the fingers, tool y) and records the clamped width each time.

One motor drives both fingers, so once both touch, their forces are equal by construction. Off-centre, one finger meets the clip-held
plate first and the gripper stalls with the other finger short of it: the clamped width grows by the offset. Expected: a V, width =
free width + |shift|. This run gives the slope, the noise floor and the resolution a centring routine can rely on (step 2).

  python tools/balance_bench.py                    shifts 0, +0.5, -0.5, +1, -1, +2, -2, 0 mm
  python tools/balance_bench.py --shifts 0,1,-1    your own list (mm, + is the claw's left looking along the fingers)

Each shift: hover, jaw open, lower to the seat, force clamp (the loop's own limit and speed), read width and torque, open, lift. About
15 s a point. Writes runs/balance_bench/<time>.json and prints the table. Start with the claw empty near the module; the arm goes to the
module's hover by the planner first.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
os.chdir(HERE)

import cli as R  # noqa: E402
from devices import nova  # noqa: E402
from procedures.cycle import Cycle  # noqa: E402
from procedures.motion import offset  # noqa: E402
from procedures.safety import SpeedProfile  # noqa: E402
from procedures.state import StateStore  # noqa: E402
from procedures.telemetry import Telemetry  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shifts", default="0,0.5,-0.5,1,-1,2,-2,0")
    ap.add_argument("--speed", default="supervised")
    a = ap.parse_args()
    shifts = [float(s) for s in a.shifts.split(",")]
    layout = R.load_layout()
    runs = os.path.join(R.RUNS, "balance_bench")
    tele = Telemetry(runs)
    robot, st = R.connect_robot_readonly(layout, tele.log)
    if st.drag:
        robot.drag(False)
    if not robot.enable():
        sys.exit("robot could not be enabled")
    store = StateStore(os.path.join(R.HERE, "state_microscope.json"))
    cyc = Cycle(layout, robot, nova.vision_box(layout, tele.log), tele, store, "supervised", runs, R.REFS)
    cyc.safety.profile = SpeedProfile(**layout["speeds"][a.speed])
    cyc.corr.ask = lambda q, t=0: "y"
    s = store.s
    if s.plate_at != "polar" or s.grip != "none":
        sys.exit(f"the ledger says plate_at={s.plate_at} grip={s.grip}: this needs the plate on the module and the claw empty")
    F = layout["fixtures"]["polar"]
    g = layout["gripper"]
    limit, speed = float(g.get("clamp_limit", 0.5)), int(g.get("clamp_speed", 20))
    closed = float(g["closed"]["short_edges"])
    free = (g.get("plate_width") or {}).get("short_edges")
    P0 = F["pose_at_0"]["pose"]
    seat = cyc._seat(P0, [0.0] * 6)
    hover = offset(seat, [0, 0, F["hover_mm"]])
    yaw = math.radians(P0[5])
    ty = (-math.sin(yaw), math.cos(yaw))                                   # tool y: across the fingers, + to the claw's left
    print(f"seat {[round(v, 1) for v in seat[:3]]} yaw {P0[5]:.1f}; clamp command {closed}, limit {limit}, speed {speed}; free width on record: {free}", flush=True)
    rows = []
    try:
        cyc.move_to("polar.pose_at_0")                                       # the planner takes the claw to the module's hover
        cyc._polar_ensure("polar", "polar@0", P0, hover)
        for k, d in enumerate(shifts):
            h = offset(hover, [d * ty[0], d * ty[1], 0.0])
            t = offset(seat, [d * ty[0], d * ty[1], 0.0])
            cyc._move("L", h, f"bench {k + 1}/{len(shifts)}: hover at {d:+.1f} mm", cyc.safety.profile.approach)
            cyc._jaw_open("short_edges", F)
            cyc._descend(t, f"bench {k + 1}: lower at {d:+.1f} mm", allow_floor=("polar",))
            t0 = time.time()
            r = robot.clamp(closed, limit, speed, free)
            dt = time.time() - t0
            rows.append({"shift_mm": d, "width": r["width"], "torque": r["torque"], "torque_min": r.get("torque_min"), "torque_max": r.get("torque_max"), "code": r.get("code"), "clamp_s": round(dt, 1)})
            print(f"  shift {d:+5.1f} mm -> width {r['width']:.2f}  torque {r['torque']:+.3f}  ({dt:.1f} s)", flush=True)
            cyc._jaw_open("short_edges", F)
            cyc._move("L", h, f"bench {k + 1}: lift", cyc.safety.profile.approach)
        cyc._move("L", hover, "back to the hover", cyc.safety.profile.approach)
    except BaseException as e:
        print("STOPPED:", type(e).__name__, e, flush=True)
        raise
    finally:
        robot.disconnect()
        tele.close()
        if rows:
            os.makedirs(runs, exist_ok=True)
            out = os.path.join(runs, time.strftime("%Y%m%d_%H%M%S") + ".json")
            json.dump({"free_width_on_record": free, "closed": closed, "limit": limit, "rows": rows}, open(out, "w", encoding="utf-8"), indent=1)
            print("wrote", out)
            w0 = [r["width"] for r in rows if r["shift_mm"] == 0]
            if w0 and len(rows) > 2:
                base = sum(w0) / len(w0)
                print(f"centre width {base:.2f} (n={len(w0)}, spread {max(w0) - min(w0):.2f});", "per point:", ", ".join(f"{r['shift_mm']:+.1f}: {r['width'] - base:+.2f}" for r in rows if r["shift_mm"] != 0))


if __name__ == "__main__":
    main()
