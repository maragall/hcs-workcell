"""
Hand-guided trajectory capture and normalisation.

Record: sample joints and Cartesian pose while the operator drags the arm (read-only).
Normalise: several demonstrations of the same route are resampled by arc length, averaged,
smoothed, and thinned to a waypoint list.  Each waypoint carries joints and pose, so it can be
validated against the exclusion zones before it is ever replayed.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class Trace:
    name: str
    joints: List[List[float]]
    poses: List[List[float]]
    t: List[float]

    def save(self, path: str):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.__dict__, f)

    @staticmethod
    def load(path: str) -> "Trace":
        with open(path, "r", encoding="utf-8") as f:
            return Trace(**json.load(f))


def record(robot, name: str, stop_file: str, interval_s: float = 0.1, max_s: float = 900, log=print) -> Trace:
    joints, poses, ts = [], [], []
    t0 = time.time()
    log(f"recording '{name}'. Stops when {stop_file} exists or after {max_s:.0f} s.")
    while not os.path.exists(stop_file) and time.time() - t0 < max_s:
        joints.append(robot.joints())
        poses.append(robot.pose())
        ts.append(time.time() - t0)
        if len(ts) % 50 == 0:
            log(f"  {len(ts)} samples  pose {[round(v, 1) for v in poses[-1][:3]]}")
        time.sleep(interval_s)
    if os.path.exists(stop_file):
        os.remove(stop_file)
    return Trace(name, joints, poses, ts)


def _trim_static(tr: Trace, eps_deg: float = 0.3) -> Trace:
    j = np.asarray(tr.joints)
    moving = np.r_[False, np.abs(np.diff(j, axis=0)).max(axis=1) > eps_deg]
    if not moving.any():
        raise ValueError("trace never moves")
    a, b = int(np.argmax(moving)) - 1, len(moving) - int(np.argmax(moving[::-1]))
    a = max(a, 0)
    return Trace(tr.name, tr.joints[a:b], tr.poses[a:b], tr.t[a:b])


def _arc(arr: np.ndarray) -> np.ndarray:
    seg = np.linalg.norm(np.diff(arr, axis=0), axis=1)
    return np.r_[0, np.cumsum(seg)]


def _resample(arr: np.ndarray, n: int, s: np.ndarray = None) -> np.ndarray:
    """Resample `arr` at n points evenly spaced along the arc length `s` (defaults to arr's own arc length).
    Passing the joints' arc length for the poses keeps joint and pose samples aligned in time."""
    s = _arc(arr) if s is None else s
    if s[-1] == 0:
        return np.repeat(arr[:1], n, axis=0)
    target = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(target, s, arr[:, k]) for k in range(arr.shape[1])])


def _smooth(arr: np.ndarray, win: int = 7) -> np.ndarray:
    if len(arr) < win:
        return arr
    k = np.ones(win) / win
    out = arr.copy()
    for c in range(arr.shape[1]):
        out[:, c] = np.convolve(np.pad(arr[:, c], (win // 2, win // 2), mode="edge"), k, mode="valid")
    out[0], out[-1] = arr[0], arr[-1]
    return out


def normalise(traces: List[Trace], n_samples: int = 200, max_points: int = 15, max_joint_step: float = 25.0) -> dict:
    """Average several demonstrations and thin them to waypoints.

    Returns {"joints": [...], "poses": [...], "spread_deg": per-sample max deviation between demos}.
    """
    trimmed = [_trim_static(t) for t in traces]
    arcs = [_arc(np.asarray(t.joints)[:, :4]) for t in trimmed]
    J = np.stack([_resample(np.asarray(t.joints)[:, :4], n_samples, a) for t, a in zip(trimmed, arcs)])
    E = np.stack([_resample(np.asarray(t.joints)[:, 4:], n_samples, a) for t, a in zip(trimmed, arcs)]).mean(axis=0)
    P = np.stack([_resample(np.asarray(t.poses), n_samples, a) for t, a in zip(trimmed, arcs)])
    spread = float(np.abs(J - J.mean(axis=0)).max()) if len(traces) > 1 else 0.0
    j_mean = _smooth(J.mean(axis=0))
    p_mean = _smooth(P.mean(axis=0))

    idx = [0]
    for i in range(1, n_samples):
        if np.abs(j_mean[i] - j_mean[idx[-1]]).max() >= max_joint_step * 0.6:
            idx.append(i)
    if idx[-1] != n_samples - 1:
        idx.append(n_samples - 1)
    # thin to the point budget, but never let a joint step exceed the limit: grow the budget if needed
    budget = max_points
    while True:
        cand = list(idx)
        while len(cand) > budget:
            gaps = np.diff(cand)
            k = int(np.argmin(gaps[:-1] + gaps[1:])) + 1
            cand.pop(k)
        steps = [float(np.abs(j_mean[cand[i + 1]] - j_mean[cand[i]]).max()) for i in range(len(cand) - 1)]
        if not steps or max(steps) <= max_joint_step or budget >= len(idx):
            idx = cand
            break
        budget += 5
    joints = [list(map(float, j_mean[i])) + list(map(float, E[i])) for i in idx]   # ext axes recorded, never replayed
    poses = [list(map(float, p_mean[i])) for i in idx]
    steps = [float(np.abs(np.asarray(joints[i + 1][:4]) - np.asarray(joints[i][:4])).max()) for i in range(len(idx) - 1)]
    if steps and max(steps) > max_joint_step:
        raise ValueError(f"waypoint joint step {max(steps):.1f} deg exceeds {max_joint_step}; record more points or slower")
    return {"joints": joints, "poses": poses, "spread_deg": spread, "max_step_deg": max(steps) if steps else 0.0,
            "demos": len(traces)}
