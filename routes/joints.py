"""
The arm's joints as the planner sees them: limits, the elbow branch, and whether a straight leg can be driven on a branch.

A straight Cartesian leg is only a leg the controller can drive if the joints it needs stay inside their limits and do not
jump (near the arm's own axis, or where the other elbow branch begins, the joints race or run out). On 2026-10-28 the
controller refused a straight leg with 'singularity or unreachable path' from an arm that had been dragged onto the MIRROR
elbow branch, and earlier refused j2 = 99.4 with 'joint reached soft limit'. The numbers below are what the cell showed:

    j2  +-95.0   the arm stopped at 95.1 when sent to 99.4; it held 93.9 on both branches
    j3  +-180.0  the standard branch (j3 negative) refused -187.2 four mm short; the mirror branch held +174.8
    j4  +-180.0  the routes held -181.8 and 183.6 (their turn point) and a hand drag went further. The planned range is ONE turn
                 (less the margin) on purpose: a pose then has exactly one j4, so a leg checked alone is the leg driven

They are conservative guesses at soft limits that nobody has read from the controller; `layout.limits.joint_limits_deg`
overrides them ({"j2": [lo, hi], ...}). Reading them from the controller is the open item (docs: routes_opt/REPORT.md).
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from routes_opt import fk

JOINT_LIMITS: Dict[str, Tuple[float, float]] = {"j2": (-95.0, 95.0), "j3": (-180.0, 180.0), "j4": (-180.0, 180.0)}
MARGIN_DEG = 0.5          # planned legs stay this far inside the limits (the arm stood at 93.9 and stopped at 95.1)
MAX_STEP_DEG = 25.0       # between two samples of a leg (8 mm / a few degrees of yaw): more is a branch jump or the axis
SAMPLE_MM = 8.0
SAMPLE_DEG = 4.0


def limits(layout: Optional[dict] = None) -> Dict[str, Tuple[float, float]]:
    out = dict(JOINT_LIMITS)
    for k, v in ((layout or {}).get("limits", {}).get("joint_limits_deg") or {}).items():
        out[k] = (float(v[0]), float(v[1]))
    return out


def inside(joints: Sequence[float], lim: Dict[str, Tuple[float, float]], margin: float = MARGIN_DEG) -> Optional[str]:
    """None when joints [j1, j2, j3, j4] are inside the limits (with the margin), else which one is not."""
    for name, v in (("j2", joints[1]), ("j3", joints[2]), ("j4", joints[3])):
        lo, hi = lim[name]
        if not lo + margin <= v <= hi - margin:
            return f"{name} = {v:.1f} deg is outside {lo + margin:.0f}..{hi - margin:.0f}"
    return None


def _excess(joints: Sequence[float], lim: Dict[str, Tuple[float, float]], margin: float) -> List[float]:
    """How far each of j2, j3, j4 is outside its limits (0 inside)."""
    out = []
    for name, v in (("j2", joints[1]), ("j3", joints[2]), ("j4", joints[3])):
        lo, hi = lim[name]
        out.append(max(lo + margin - v, v - (hi - margin), 0.0))
    return out


def pose_joints(pose: Sequence[float], branch: int, prev: Optional[Sequence[float]] = None,
                lim: Optional[Dict[str, Tuple[float, float]]] = None) -> List[float]:
    """The joints that put the wrist at `pose` on the elbow `branch`. j4 has 360-degree twins: of those inside the limits the one
    nearest `prev` (the turn the short way, continuity along a leg), else the one nearest 0. Outside the limits there may be none:
    then the nearest to `prev` is returned and the caller's limit check says so. Raises ValueError when out of reach."""
    lim = lim or JOINT_LIMITS
    j = fk.ik(pose, elbow=branch, j2_ref=prev[1] if prev else None, j3_ref=prev[2] if prev else None,
              j4_ref=prev[3] if prev else None)
    reps = [j[3] + 360.0 * k for k in range(-3, 4)]
    lo, hi = lim["j4"]
    ok = [r for r in reps if lo + MARGIN_DEG <= r <= hi - MARGIN_DEG]
    ref = prev[3] if prev else 0.0
    j[3] = min(ok or reps, key=lambda r: abs(r - ref))
    return j


def leg_joints(a: Sequence[float], b: Sequence[float], branch: int, lim: Optional[Dict[str, Tuple[float, float]]] = None,
               start: Optional[Sequence[float]] = None):
    """(joints at a, joints at b, turn) for the leg a -> b on `branch`: `turn` is how far the wrist yaw (j2 + j3 + j4) turns
    over the leg, signed and NOT wrapped, so a turn of 88 degrees is 88, never -272: the direction is the one whose j4 stays inside
    the limits, the shorter when both do."""
    lim = lim or JOINT_LIMITS
    ja = list(start[:4]) if start is not None else pose_joints(a, branch, None, lim)
    jb = fk.ik(b, elbow=branch, j2_ref=ja[1], j3_ref=ja[2])
    short = fk.wrap180(b[5] - fk.fk(ja)[5])                      # the wrist yaw turns by this, or by this minus a full turn
    best = None
    in_place = math.dist(a[:3], b[:3]) <= 1.0                    # only a turn in place may take the long way: a leg that moves keeps its yaw
    for turn in ((short, short - 360.0 if short > 0 else short + 360.0) if in_place else (short,)):
        jb4 = sum(ja[1:4]) + turn - jb[1] - jb[2]
        cand = [jb[0], jb[1], jb[2], jb4]
        if best is None:
            best = (turn, cand)                                  # the short way, unless it ends outside the range
        if not any(_excess(cand, lim, MARGIN_DEG)):
            best = (turn, cand)
            break
    return ja, best[1], best[0]


def leg_poses(a: Sequence[float], b: Sequence[float], turn: Optional[float] = None) -> Iterable[List[float]]:
    """Samples of the straight leg a -> b: x, y, z linear, yaw linear by `turn` degrees (default: the wrapped difference)."""
    turn = fk.wrap180(b[5] - a[5]) if turn is None else turn
    n = max(1, int(max(math.dist(a[:3], b[:3]) / SAMPLE_MM, abs(turn) / SAMPLE_DEG)))
    for k in range(n + 1):
        t = k / n
        yield [a[i] + t * (b[i] - a[i]) for i in range(5)] + [a[5] + t * turn]


def leg_feasible(a: Sequence[float], b: Sequence[float], branch: int, lim: Optional[Dict[str, Tuple[float, float]]] = None,
                 start: Optional[Sequence[float]] = None, margin: float = MARGIN_DEG) -> Optional[str]:
    """None when the controller can drive the leg a -> b on `branch` (a straight line, the yaw turning by the joints' own
    direction); else why not. `start`: the arm's actual joints when `a` is where it stands: a wrist already outside the normal
    range (dragged there by hand) may leave, each joint only toward its range."""
    lim = lim or JOINT_LIMITS
    name = "standard" if branch > 0 else "mirror"
    try:
        ja, jb, turn = leg_joints(a, b, branch, lim, start)
    except ValueError as e:
        return f"unreachable on the {name} elbow branch: {e}"
    prev, prev_ex = ja, _excess(ja, lim, margin)
    if start is None and any(prev_ex):
        return f"{inside(ja, lim, margin)} at the start of the leg on the {name} elbow branch"
    if any(_excess(jb, lim, margin)):
        return f"{inside(jb, lim, margin)} at the end of the leg on the {name} elbow branch"
    rz0 = sum(ja[1:4])
    for k, p in enumerate(leg_poses(a, b, turn)):
        if k == 0:
            continue
        try:
            j = fk.ik(p, elbow=branch, j2_ref=prev[1], j3_ref=prev[2], j4_ref=prev[3])
        except ValueError as e:
            return f"unreachable on the {name} elbow branch: {e}"
        j[3] = (rz0 + (p[5] - a[5])) - j[1] - j[2]            # the yaw turns by the joints' direction, j4 follows
        ex = _excess(j, lim, margin)
        if any(e > pe + 0.01 for e, pe in zip(ex, prev_ex)):
            return f"{inside(j, lim, margin)} at x={p[0]:.0f} y={p[1]:.0f} on the {name} elbow branch"
        step = max(abs(j[i] - prev[i]) for i in (1, 2, 3))
        if step > MAX_STEP_DEG:
            return (f"the joints jump {step:.0f} deg between two samples at x={p[0]:.0f} y={p[1]:.0f} "
                    f"(the axis, or the other elbow branch)")
        prev, prev_ex = j, ex
    return None
