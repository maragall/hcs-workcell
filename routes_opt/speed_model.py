"""
Duration model for waypoint (AJ) moves, fitted from runs/*/events.jsonl. Offline only.

How a waypoint is executed (nova_cell/cycle.py _waypoints -> robot.move_abs_joints_stepped, max_step 8):
  the joint delta d (max over j1 mm, j2..j4 deg) is split into nsub = int(d/8 + 0.999) equal sub-moves, each a
  separate MoveAbsJointAsync call verified by joint readback (0.1 s polling); then _wait_still, one pose read.
The log carries per waypoint: dt (whole waypoint), dev. Joints are not logged, so the deltas come from the
layout trajectory (unchanged since 2026-10-15 02:04) and the direction from the preceding lift step.
Travel speed per run is inferred from the linear-move speeds (approach 0.1 -> 20 % travel, 0.25 -> 50 %).

Model per sub-move:  t_sub = t0 + max(dz_sub / v_z, d23_sub / v_23, d4_sub / v_4)
  t0 fixed call + verification overhead; v_* effective axis rates (column mm/s, shoulder/elbow deg/s, wrist deg/s).
  Fitted on per-waypoint medians (robust to the SDK MotionStop re-issues), per travel profile. Zero-motion AJ
  calls (route B waypoint 1 forward, reached from the hover) give t0 directly: 0.90-0.92 s at both profiles.
Writes speed_model.json.
"""
from __future__ import annotations

import collections
import glob
import json
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SUBSTEP = 8.0


def nsub_of(d: float) -> int:
    return max(1, int(d / SUBSTEP + 0.999))


def extract_rows(layout):
    TR = {n: np.asarray(t["joints"])[:, :4] for n, t in layout["trajectories"].items()}
    fwd_prev = {"microscope_to_polar": "pick:microscope", "polar_to_hotel": "pick:polar"}
    rev_prev = {"microscope_to_polar": "pick:polar", "polar_to_hotel": "pick:hotel"}
    rows = []
    for run in sorted(glob.glob(os.path.join(ROOT, "runs", "2026*"))):
        f = os.path.join(run, "events.jsonl")
        if not os.path.exists(f) or os.path.getsize(f) == 0:
            continue
        speeds = set()
        with open(f, encoding="utf-8") as fh:
            for line in fh:
                if '"msg": "L"' in line:
                    m = re.search(r'"speed": ([0-9.]+)', line)
                    if m:
                        speeds.add(float(m.group(1)))
        travel = 0.5 if 0.25 in speeds else (0.2 if (0.1 in speeds and 0.05 in speeds) else None)
        if travel is None:
            continue
        prev = ""; direction = None; last = None
        with open(f, encoding="utf-8") as fh:
            for line in fh:
                if '"kind": "move"' not in line:
                    continue
                try:
                    e = json.loads(line)
                except Exception:
                    continue
                step = e.get("step", "")
                if e.get("msg") != "AJ" or "[" not in step:
                    prev = step
                    continue
                name, idx = step.split("["); i, n = (int(v) for v in idx.rstrip("]").split("/"))
                if name not in TR or len(TR[name]) != n:
                    continue
                if i == 1 or last != name:
                    direction = 1 if prev.startswith(fwd_prev[name]) else (-1 if prev.startswith(rev_prev[name]) else None)
                last = name
                if direction is None:
                    continue
                seq = TR[name] if direction > 0 else TR[name][::-1]
                if i == 1:
                    if direction > 0 and name == "microscope_to_polar":
                        rows.append(dict(run=os.path.basename(run), travel=travel, route=name, i=1, dir=1, dt=e["dt"],
                                         dz=0.0, d23=0.0, d4=0.0, dmax=0.0, nsub=1))   # from the hover: zero motion
                    continue
                d = np.abs(seq[i - 1] - seq[i - 2])
                rows.append(dict(run=os.path.basename(run), travel=travel, route=name, i=i, dir=direction, dt=float(e["dt"]),
                                 dz=float(d[0]), d23=float(max(d[1], d[2])), d4=float(d[3]), dmax=float(d.max()), nsub=nsub_of(float(d.max()))))
    return rows


def predict_segment(d: np.ndarray, params: dict) -> float:
    """d = |joint delta| (4,) for one waypoint-to-waypoint move; returns seconds."""
    dmax = float(np.max(d)); n = nsub_of(dmax)
    motion = max(d[0] / n / params["v_z"], max(d[1], d[2]) / n / params["v_23"], d[3] / n / params["v_4"])
    return n * (params["t0"] + motion)


def fit_profile(meds, t0_fixed=None):
    """meds: list of dicts with dz, d23, d4, nsub, dt (per-waypoint medians). Grid search, then local refinement."""
    dz = np.array([m["dz"] for m in meds]); d23 = np.array([m["d23"] for m in meds]); d4 = np.array([m["d4"] for m in meds])
    ns = np.array([m["nsub"] for m in meds]); y = np.array([m["dt"] for m in meds])

    def pred(t0, vz, v23, v4):
        return ns * (t0 + np.maximum.reduce([dz / ns / vz, d23 / ns / v23, d4 / ns / v4]))

    best = None
    grid_v = np.exp(np.linspace(np.log(2), np.log(400), 28))
    t0s = [t0_fixed] if t0_fixed is not None else np.linspace(0.5, 1.1, 13)
    for t0 in t0s:
        for vz in grid_v:
            for v23 in grid_v:
                for v4 in grid_v:
                    r = pred(t0, vz, v23, v4) - y
                    s = float((r * r).mean())
                    if best is None or s < best[0]:
                        best = (s, t0, vz, v23, v4)
    s, t0, vz, v23, v4 = best
    # local refinement
    for _ in range(3):
        for k in range(4):
            vals = [t0, vz, v23, v4]
            base = vals[k]
            cands = base * np.exp(np.linspace(-0.15, 0.15, 31)) if k else base + np.linspace(-0.05, 0.05, 21)
            for cnd in cands:
                v = list(vals); v[k] = cnd
                if t0_fixed is not None and k == 0:
                    continue
                r = pred(*v) - y; sc = float((r * r).mean())
                if sc < s:
                    s, (t0, vz, v23, v4) = sc, v
    return dict(t0=float(t0), v_z=float(vz), v_23=float(v23), v_4=float(v4), rms_s=float(np.sqrt(s)), n_points=int(len(y)))


def main():
    layout = json.load(open(os.path.join(ROOT, "layout.json"), encoding="utf-8"))
    rows = extract_rows(layout)
    print("AJ records paired with waypoint deltas:", len(rows))
    out = {"substep_units": SUBSTEP, "profiles": {}, "_source": "runs/2026*/events.jsonl AJ records; deltas from layout trajectories"}
    by = collections.defaultdict(list)
    for r in rows:
        by[(r["travel"], r["route"], r["dir"], r["i"])].append(r)
    for travel in (0.2, 0.5):
        meds = []
        for k, v in by.items():
            if k[0] != travel:
                continue
            dts = np.array([r["dt"] for r in v])
            meds.append(dict(dz=v[0]["dz"], d23=v[0]["d23"], d4=v[0]["d4"], nsub=v[0]["nsub"], dt=float(np.median(dts)), n=len(v),
                             route=k[1], dir=k[2], i=k[3]))
        zero = [m for m in meds if m["nsub"] == 1 and m["dz"] == 0]
        t0 = float(np.mean([m["dt"] for m in zero])) if zero else None
        p = fit_profile([m for m in meds if m["nsub"] > 1], t0_fixed=None)
        p_fixed = fit_profile([m for m in meds if m["nsub"] > 1], t0_fixed=t0) if t0 else None
        print("\ntravel %.0f %%: %d waypoint medians (%d records); zero-motion call %.2f s" % (travel * 100, len(meds), sum(m["n"] for m in meds), t0 or -1))
        print("  free fit : t0 %.3f  v_z %.1f mm/s  v_23 %.1f deg/s  v_4 %.1f deg/s  rms %.3f s" % (p["t0"], p["v_z"], p["v_23"], p["v_4"], p["rms_s"]))
        if p_fixed:
            print("  t0 fixed : t0 %.3f  v_z %.1f mm/s  v_23 %.1f deg/s  v_4 %.1f deg/s  rms %.3f s" % (p_fixed["t0"], p_fixed["v_z"], p_fixed["v_23"], p_fixed["v_4"], p_fixed["rms_s"]))
        chosen = p_fixed if p_fixed and p_fixed["rms_s"] < p["rms_s"] * 1.15 else p
        # observed one-way route times (sum of per-waypoint medians) for validation
        obs = {}
        for route in ("microscope_to_polar", "polar_to_hotel"):
            for d in (1, -1):
                ms = [m for m in meds if m["route"] == route and m["dir"] == d and m["nsub"] > 1]
                if ms:
                    pred = sum(predict_segment(np.array([m["dz"], m["d23"], m["d23"], m["d4"]]), chosen) for m in ms)
                    obs[f"{route}:{'fwd' if d > 0 else 'rev'}"] = dict(observed_s=float(sum(m["dt"] for m in ms)), modelled_s=float(pred), n_segments=len(ms))
                    print("  %-28s %2d segments  observed %6.1f s  modelled %6.1f s" % (f"{route} {'fwd' if d > 0 else 'rev'}", len(ms), sum(m["dt"] for m in ms), pred))
        chosen["zero_motion_call_s"] = t0
        chosen["route_times_observed_vs_modelled"] = obs
        chosen["per_waypoint_medians"] = sorted(meds, key=lambda m: (m["route"], m["dir"], m["i"]))
        out["profiles"]["%.0f" % (travel * 100)] = chosen
    with open(os.path.join(HERE, "speed_model.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    print("\nwrote", os.path.join(HERE, "speed_model.json"))


if __name__ == "__main__":
    main()
