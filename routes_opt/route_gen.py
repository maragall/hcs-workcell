"""
Offline route generator / validator for the Nova S4 cell. Reads layout.json, fk_model.json, speed_model.json.
Touches no hardware and writes only into routes_opt/.

A route is a list of joint waypoints replayed by cycle._waypoints with move_abs_joints_stepped (8-unit sub-moves,
joint-space linear interpolation). Every waypoint and every interpolated sample is checked with the fitted FK:

  * TCP inside the workspace box; TCP above the z floor of its region (safety.py semantics)
  * elbow in the taught branch (j3 in (-180, 0)) with a margin from full extension; joints inside the taught envelope
  * microscope rule (stricter than safety.py, which only looks at the TCP): the plate rectangle (128 x 85 around
    the TCP, long axis along the gripper yaw for the short-edge grip) may be over the microscope region box only at
    or above the hover height and only with the seat yaw (no turning over the microscope); below hover height no
    part of the plate may be over the box. Strict variants also require that any yaw change happens at or above
    hover height ("turn above the microscope hover height") and with the plate's swept circle (r = 76.8 mm) clear of
    the box ("away from the microscope").
  * hotel rule (route A): the plate rectangle stays out of the rack front volume (x >= 245, -190 <= y <= 215;
    the rack's real +y extent is NOT in layout.json, see REPORT.md)
  * consecutive waypoints differ by at most 24 units (limit 25; 24 = 3 full sub-moves, no rounding loss)

Scoring: summed joint travel (deg over j2..j4), column travel (mm), sub-move count, modelled duration at the
50 % (stress) and 20 % profiles, minimum clearance of the plate to the microscope zone.
"""
from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import fk  # noqa: E402
from speed_model import nsub_of, predict_segment  # noqa: E402

LAY = json.load(open(os.path.join(ROOT, "layout.json"), encoding="utf-8"))
SM = json.load(open(os.path.join(HERE, "speed_model.json"), encoding="utf-8"))
MAX_STEP = 24.0                         # layout limit 25; 24 = exactly 3 sub-moves of 8
PLATE_L, PLATE_W = 128.0, 85.0          # brief: 128 x 85 rectangle around the TCP
SWEEP_R = math.hypot(PLATE_L / 2, PLATE_W / 2)   # 76.8 mm, plate corner radius while turning
JOINT_ENVELOPE = {"j1": (20.0, 340.0), "j2": (-18.0, 99.0), "j3": (-179.0, -20.0), "j4": (-40.0, 184.0)}
# taught data: j2 -12.9..93.9, j3 -177.7..-23.7 (routes), j4 -32.2..183.6, j1 30.4..296.4 on the routes, 335.4 at hotel A9
# (z 374 + 25 lift); the physical j4 limit is unknown -> capped at 184
SAMPLE_UNITS = 0.5                      # sampling resolution along a segment (max joint delta units)


# ----------------------------------------------------------------------------- geometry helpers
def plate_corners(x, y, yaw_deg, grip, fwd_offset=0.0):
    """(n,4,2) corners of the plate rectangle. short_edges: long axis along the yaw direction; long_edges: across."""
    t = np.deg2rad(yaw_deg)
    u = np.stack([np.cos(t), np.sin(t)], -1); v = np.stack([-np.sin(t), np.cos(t)], -1)
    hu, hv = (PLATE_L / 2, PLATE_W / 2) if grip == "short_edges" else (PLATE_W / 2, PLATE_L / 2)
    c = np.stack([x, y], -1) + fwd_offset * u
    return np.stack([c + hu * u + hv * v, c + hu * u - hv * v, c - hu * u - hv * v, c - hu * u + hv * v], 1)


def rect_box_overlap(corners, bx, by):
    """Separating-axis test rectangle (n,4,2) vs axis-aligned box. Returns bool (n,)."""
    px, py = corners[..., 0], corners[..., 1]
    sep_x = (px.max(1) < bx[0]) | (px.min(1) > bx[1])
    sep_y = (py.max(1) < by[0]) | (py.min(1) > by[1])
    # rectangle's own axes
    u = corners[:, 0] - corners[:, 3]; v = corners[:, 0] - corners[:, 1]
    boxc = np.array([[bx[0], by[0]], [bx[1], by[0]], [bx[1], by[1]], [bx[0], by[1]]])
    sep = sep_x | sep_y
    for ax in (u, v):
        ax = ax / np.linalg.norm(ax, axis=1, keepdims=True)
        pr = np.einsum("nkj,nj->nk", corners, ax); pb = boxc @ ax.T   # (n,4), (4,n)
        sep |= (pr.max(1) < pb.min(0)) | (pr.min(1) > pb.max(0))
    return ~sep


def rect_box_distance(corners, bx, by):
    """Euclidean distance (n,) between rectangle and box (0 when overlapping)."""
    n = len(corners)
    # rectangle corners to box
    cx = np.clip(corners[..., 0], bx[0], bx[1]); cy = np.clip(corners[..., 1], by[0], by[1])
    d1 = np.hypot(corners[..., 0] - cx, corners[..., 1] - cy).min(1)
    # box corners to rectangle (local frame)
    c = corners.mean(1); u = corners[:, 0] - corners[:, 3]; v = corners[:, 0] - corners[:, 1]
    hu = np.linalg.norm(u, axis=1) / 2; hv = np.linalg.norm(v, axis=1) / 2
    u = u / (2 * hu[:, None]); v = v / (2 * hv[:, None])
    boxc = np.array([[bx[0], by[0]], [bx[1], by[0]], [bx[1], by[1]], [bx[0], by[1]]])
    rel = boxc[None, :, :] - c[:, None, :]                       # (n,4,2)
    lu = np.einsum("nkj,nj->nk", rel, u); lv = np.einsum("nkj,nj->nk", rel, v)
    du = np.maximum(np.abs(lu) - hu[:, None], 0); dv = np.maximum(np.abs(lv) - hv[:, None], 0)
    d2 = np.hypot(du, dv).min(1)
    d = np.minimum(d1, d2)
    d[rect_box_overlap(corners, bx, by)] = 0.0
    return d


def point_box_distance(x, y, bx, by):
    return np.hypot(x - np.clip(x, bx[0], bx[1]), y - np.clip(y, by[0], by[1]))


# ----------------------------------------------------------------------------- the checker
@dataclass
class Rules:
    grip: str = "short_edges"
    strict_turn: bool = True            # yaw may change only at/above the microscope hover height
    sweep_margin_mm: float = 15.0       # plate swept circle must clear the microscope box by this while turning
    #   15 = the most a turn that ENDS above the re-taught polar seat (x 312.5) can have: 312.5 - 76.8 - 220 = 15.7
    plate_fwd_offset_mm: float = 0.0    # plate centre ahead of the TCP along the yaw (unknown; 0 per the brief)
    mic_plate_rule: bool = True         # microscope rule on the plate rectangle (route B); False = TCP only (route A)
    hotel_rule: bool = False            # route A: keep the plate out of the rack structure beside the A1 opening
    hotel_keepout: Tuple = (((250.0, 400.0), (-190.0, 68.0)), ((250.0, 400.0), (206.0, 215.0)))   # list of (x-range, y-range) boxes
    corridor: Optional["Corridor"] = None   # route A: plate must stay inside the region swept by the taught route


@dataclass
class Eval:
    ok: bool
    violations: List[str]
    n_samples: int
    min_lateral_clearance_mm: float        # plate -> microscope box, over samples that are NOT over the box
    min_lateral_below_hover_mm: float      # same, only samples below hover height (the ones that matter)
    min_tcp_clearance_mm: float            # TCP point -> microscope box (safety.py's view), samples below hover
    samples_over_box: int
    min_margin_over_box_mm: float          # z - z_hover while over the box
    turn_clearance_mm: float               # (TCP distance to box - sweep radius) over yaw-changing samples
    max_corner_x: float
    corridor_outside_mm2: float = 0.0      # test points of the plate outside the proven corridor (0 = contained)
    joint_min: List[float] = field(default_factory=list)
    joint_max: List[float] = field(default_factory=list)


class Corridor:
    """Occupancy grid (1 mm) of the xy region swept by a proven plate path, dilated by a margin."""

    def __init__(self, x_range=(80, 400), y_range=(-20, 420), margin_mm=5.0):
        self.x0, self.y0 = x_range[0], y_range[0]
        self.nx, self.ny = int(x_range[1] - x_range[0]) + 1, int(y_range[1] - y_range[0]) + 1
        self.mask = np.zeros((self.ny, self.nx), bool)
        self.margin = margin_mm
        self.gx, self.gy = np.meshgrid(np.arange(self.nx) + self.x0, np.arange(self.ny) + self.y0)

    def add_rectangles(self, corners):
        for C in corners:
            c = C.mean(0); u = C[0] - C[3]; v = C[0] - C[1]
            hu, hv = np.linalg.norm(u) / 2, np.linalg.norm(v) / 2; u = u / (2 * hu); v = v / (2 * hv)
            xmin, xmax = max(int(C[:, 0].min() - self.x0) - 1, 0), min(int(C[:, 0].max() - self.x0) + 2, self.nx)
            ymin, ymax = max(int(C[:, 1].min() - self.y0) - 1, 0), min(int(C[:, 1].max() - self.y0) + 2, self.ny)
            gx = self.gx[ymin:ymax, xmin:xmax] - c[0]; gy = self.gy[ymin:ymax, xmin:xmax] - c[1]
            inside = (np.abs(gx * u[0] + gy * u[1]) <= hu) & (np.abs(gx * v[0] + gy * v[1]) <= hv)
            self.mask[ymin:ymax, xmin:xmax] |= inside

    def finalize(self):
        m = int(round(self.margin)); d = self.mask.copy()
        for dy in range(-m, m + 1):
            for dx in range(-m, m + 1):
                if dx * dx + dy * dy <= m * m:
                    d |= np.roll(np.roll(self.mask, dy, 0), dx, 1)
        self.dilated = d
        return self

    def outside_points(self, corners, spacing=4.0):
        """Number of test points (lattice over each rectangle, `spacing` mm) outside the corridor, per rectangle."""
        out = np.zeros(len(corners), int)
        for i, C in enumerate(corners):
            c = C.mean(0); u = C[0] - C[3]; v = C[0] - C[1]
            hu, hv = np.linalg.norm(u) / 2, np.linalg.norm(v) / 2; u = u / (2 * hu); v = v / (2 * hv)
            a = np.linspace(-hu, hu, int(2 * hu / spacing) + 2); b = np.linspace(-hv, hv, int(2 * hv / spacing) + 2)
            A, B = np.meshgrid(a, b); P = c + A.ravel()[:, None] * u + B.ravel()[:, None] * v
            ix = np.round(P[:, 0] - self.x0).astype(int); iy = np.round(P[:, 1] - self.y0).astype(int)
            ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            inside = np.zeros(len(P), bool); inside[ok] = self.dilated[iy[ok], ix[ok]]
            out[i] = int((~inside).sum())
        return out


class Checker:
    def __init__(self, layout=LAY):
        self.box = {k: tuple(v) for k, v in layout["workspace_box"].items()}
        self.regions = layout["regions"]
        mic = layout["fixtures"]["microscope"]
        self.mic_box = (tuple(self.regions["microscope"]["box"]["x"]), tuple(self.regions["microscope"]["box"]["y"]))
        self.seat = list(mic["pose"]); self.z_hover = mic["pose"][2] + mic["hover_mm"]; self.yaw_seat = mic["pose"][5]
        self.max_yaw_change = self.regions["microscope"].get("max_yaw_change_deg", 0.5)

    def floors(self, x, y):
        f = np.full(len(x), self.box["z"][0], float)
        for name, r in self.regions.items():        # first match wins in safety.region_of; boxes do not overlap here
            bx, by = r["box"]["x"], r["box"]["y"]
            m = (bx[0] <= x) & (x <= bx[1]) & (by[0] <= y) & (y <= by[1])
            f[m] = r.get("z_floor", self.box["z"][0])
        return f

    def sample(self, Q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Interpolate the waypoint list in joint space at SAMPLE_UNITS resolution. Returns samples and segment ids."""
        S, seg = [], []
        for i in range(len(Q) - 1):
            d = float(np.abs(Q[i + 1] - Q[i]).max())
            n = max(2, int(math.ceil(d / SAMPLE_UNITS)) + 1)
            t = np.linspace(0, 1, n)[:, None]
            S.append(Q[i] + t * (Q[i + 1] - Q[i])); seg.append(np.full(n, i))
        if not S:
            return Q.copy(), np.zeros(len(Q), int)
        return np.vstack(S), np.concatenate(seg)

    def evaluate(self, Q: np.ndarray, rules: Rules) -> Eval:
        Q = np.asarray(Q, float)[:, :4]
        viol = []
        steps = np.abs(np.diff(Q, axis=0)).max(1) if len(Q) > 1 else np.zeros(0)
        if len(steps) and steps.max() > 25.0 + 1e-6:
            viol.append(f"waypoint step {steps.max():.1f} > 25")
        S, seg = self.sample(Q)
        m = fk.MODEL
        j1, j2, j3, j4 = S.T
        x, y = fk.fk_xy_from_angles(j2, j3) if False else self._xy(j2, j3)
        z = m["z_scale"] * j1 + m["z_offset"]
        yaw = j2 + j3 + j4 + m["yaw_offset_deg"]
        yaw_w = (yaw + 180) % 360 - 180
        # workspace box / floors
        for ax, arr in (("x", x), ("y", y), ("z", z)):
            lo, hi = self.box[ax]
            if (arr < lo - 1e-6).any() or (arr > hi + 1e-6).any():
                viol.append(f"TCP {ax} outside workspace box [{lo},{hi}]: {arr.min():.1f}..{arr.max():.1f}")
        fl = self.floors(x, y)
        if (z < fl - 1e-6).any():
            k = int(np.argmax(z - fl < -1e-6)); viol.append(f"TCP below region floor at sample {k}: z {z[k]:.1f} < floor {fl[k]:.1f}")
        # elbow branch + envelope
        if not ((j3 > -180.0) & (j3 < 0.0)).all():
            viol.append("elbow leaves the taught branch (j3 outside (-180, 0))")
        for name, arr in zip(("j1", "j2", "j3", "j4"), (j1, j2, j3, j4)):
            lo, hi = JOINT_ENVELOPE[name]
            if arr.min() < lo - 1e-6 or arr.max() > hi + 1e-6:
                viol.append(f"{name} outside envelope [{lo},{hi}]: {arr.min():.1f}..{arr.max():.1f}")
        # microscope rules with the plate rectangle
        bx, by = self.mic_box
        C = plate_corners(x, y, yaw, rules.grip, rules.plate_fwd_offset_mm)
        over = rect_box_overlap(C, bx, by) if rules.mic_plate_rule else ((bx[0] <= x) & (x <= bx[1]) & (by[0] <= y) & (y <= by[1]))
        below = z < self.z_hover - 1e-3
        in_column = np.hypot(x - self.seat[0], y - self.seat[1]) <= 2.0
        bad = over & below & ~in_column
        if bad.any():
            k = int(np.argmax(bad)); viol.append(f"{'plate' if rules.mic_plate_rule else 'TCP'} over the microscope zone below hover height (sample {k}: xy {x[k]:.0f},{y[k]:.0f} z {z[k]:.1f} < {self.z_hover:.1f})")
        dyaw = np.abs((yaw - self.yaw_seat + 180) % 360 - 180)
        bad = over & (dyaw > self.max_yaw_change)
        if bad.any():
            k = int(np.argmax(bad)); viol.append(f"yaw differs from the seat yaw while the plate is over the zone (sample {k}: {dyaw[k]:.1f} deg)")
        dist = rect_box_distance(C, bx, by)
        notover = ~over
        min_lat = float(dist[notover].min()) if notover.any() else float("inf")
        min_lat_below = float(dist[notover & below].min()) if (notover & below).any() else float("inf")
        tcp_d = point_box_distance(x, y, bx, by)
        min_tcp_below = float(tcp_d[below].min()) if below.any() else float("inf")
        # yaw-changing samples: per segment
        turning = np.zeros(len(S), bool)
        for i in range(len(Q) - 1):
            if abs(((Q[i + 1, 1:4].sum() - Q[i, 1:4].sum()) + 180) % 360 - 180) > self.max_yaw_change:
                turning |= seg == i
        turn_clear = float((tcp_d[turning] - SWEEP_R).min()) if turning.any() else float("inf")
        if rules.strict_turn:
            if (turning & below).any():
                k = int(np.argmax(turning & below)); viol.append(f"yaw turn below the microscope hover height (sample {k}: z {z[k]:.1f})")
            if turning.any() and turn_clear < rules.sweep_margin_mm:
                viol.append(f"plate swept circle within {turn_clear:.1f} mm of the microscope box while turning (need {rules.sweep_margin_mm})")
        corridor_out = 0.0
        if rules.corridor is not None:
            n_out = rules.corridor.outside_points(C)
            if (n_out > 0).any():
                k = int(np.argmax(n_out > 0)); corridor_out = float(n_out.sum())
                viol.append(f"plate leaves the proven corridor (sample {k}: xy {x[k]:.0f},{y[k]:.0f} yaw {yaw_w[k]:.0f}; {int((n_out > 0).sum())} samples)")
        if rules.hotel_rule:
            for hx, hy in rules.hotel_keepout:
                if hy[1] <= hy[0]:
                    continue
                hov = rect_box_overlap(C, hx, hy)
                if hov.any():
                    k = int(np.argmax(hov)); viol.append(f"plate enters the rack structure x>={hx[0]:.0f} y {hy[0]:.0f}..{hy[1]:.0f} (sample {k}: xy {x[k]:.0f},{y[k]:.0f} yaw {yaw_w[k]:.0f})")
                    break
        return Eval(ok=not viol, violations=viol, n_samples=len(S), min_lateral_clearance_mm=min_lat,
                    min_lateral_below_hover_mm=min_lat_below, min_tcp_clearance_mm=min_tcp_below,
                    samples_over_box=int(over.sum()), min_margin_over_box_mm=float((z[over] - self.z_hover).min()) if over.any() else float("inf"),
                    turn_clearance_mm=turn_clear, max_corner_x=float(C[..., 0].max()), corridor_outside_mm2=corridor_out,
                    joint_min=[float(v) for v in S.min(0)], joint_max=[float(v) for v in S.max(0)])

    @staticmethod
    def _xy(j2, j3):
        m = fk.MODEL
        t1 = np.deg2rad(j2 + m["a1_deg"]); t12 = np.deg2rad(j2 + j3 + m["a2_deg"])
        return (m["x0"] + m["l1"] * np.cos(t1) + m["l2"] * np.cos(t12), m["y0"] + m["l1"] * np.sin(t1) + m["l2"] * np.sin(t12))


# ----------------------------------------------------------------------------- segmenting and scoring
def segment(via: Sequence[Sequence[float]], max_step: float = MAX_STEP) -> np.ndarray:
    """Split each straight joint-space move into pieces of `max_step` plus a remainder: the sub-move total stays
    ceil(D/8) (no rounding loss) and no waypoint step exceeds the limit."""
    out = [np.asarray(via[0], float)[:4]]
    for a, b in zip(via[:-1], via[1:]):
        a = np.asarray(a, float)[:4]; b = np.asarray(b, float)[:4]
        D = float(np.abs(b - a).max())
        if D < 1e-9:
            continue
        k = int(math.floor(D / max_step + 1e-9))
        fracs = [max_step * i / D for i in range(1, k + 1)]
        if not fracs or fracs[-1] < 1 - 1e-9:
            fracs.append(1.0)
        for f in fracs:
            out.append(a + f * (b - a))
    return np.array(out)


def score(Q: np.ndarray) -> Dict:
    Q = np.asarray(Q, float)[:, :4]
    d = np.abs(np.diff(Q, axis=0))
    nsub = [nsub_of(float(v)) for v in d.max(1)] if len(d) else []
    t50 = sum(predict_segment(row, SM["profiles"]["50"]) for row in d)
    t20 = sum(predict_segment(row, SM["profiles"]["20"]) for row in d)
    return dict(waypoints=int(len(Q)), joint_travel_deg=float(d[:, 1:4].sum()), column_travel_mm=float(d[:, 0].sum()),
                per_joint=[float(v) for v in d.sum(0)], substeps=int(sum(nsub)), max_step=float(d.max()) if len(d) else 0.0,
                t50_s=float(t50), t20_s=float(t20))


def poses_of(Q) -> List[List[float]]:
    return [[round(v, 3) for v in fk.fk(q)] for q in Q]


def as_trajectory(Q, provenance: str, note: str = "") -> Dict:
    Q = np.asarray(Q, float)
    d = np.abs(np.diff(Q[:, :4], axis=0))
    return {"joints": [[round(float(v), 4) for v in q[:4]] + [0.0, 0.0, 0.0, 0.0] for q in Q],
            "poses": poses_of(Q), "spread_deg": 0.0, "max_step_deg": float(d.max()) if len(d) else 0.0, "demos": 0,
            "provenance": provenance, "note": note}


# ----------------------------------------------------------------------------- route B candidates
def hover(fix: dict, pose_key: Optional[str] = None) -> List[float]:
    p = list(fix[pose_key]["pose"] if pose_key else fix["pose"])
    p[2] += fix["hover_mm"]
    return p


def build_route_b_candidates(chk: Checker):
    mic = LAY["fixtures"]["microscope"]; pol = LAY["fixtures"]["polar"]
    Hm = hover(mic); Hp = hover(pol, "pose_at_0")
    Qm = fk.ik(Hm, 1, j4_ref=178.0); Qp = fk.ik(Hp, 1, j4_ref=4.0)
    yaw_m, yaw_p = Hm[5], Hp[5]
    cands = []

    def east(xE, yE, h_entry, overlapped, label, via_first=None):
        z_top = Hm[2] + h_entry
        via = [Qm]
        if h_entry > 0:
            via.append([Qm[0] + h_entry, Qm[1], Qm[2], Qm[3]])
        try:
            if via_first is not None:        # pass through the taught route's corridor (its wp 4: x 186, y 353)
                via.append(fk.ik([via_first[0], via_first[1], z_top, 0, 0, yaw_m], 1, j4_ref=Qm[3]))
            E1 = fk.ik([xE, yE, z_top, 0, 0, yaw_m], 1, j4_ref=Qm[3])
            E2 = fk.ik([xE, yE, z_top, 0, 0, yaw_p], 1, j4_ref=Qp[3])
        except ValueError:
            return None
        if math.hypot(xE - Hp[0], yE - Hp[1]) > 15.0:
            return None                         # the descent must be essentially vertical above the stage (<= 15 mm drift)
        via.append(E1)
        if overlapped:
            via.append(Qp)                      # turn + descent + <= 15 mm lateral in one straight joint move
        else:
            via.append(E2)                      # turn in place at z_top
            via.append(Qp)                      # descent above the stage (plate aligned, <= 15 mm lateral drift)
        return dict(label=label, via=via, Q=segment(via), xE=xE, yE=yE, h_entry=h_entry, overlapped=overlapped)

    rules_strict = Rules(strict_turn=True); rules_relaxed = Rules(strict_turn=False)
    best = {}
    for overlapped in (False, True):
        for h_entry in (0.0, 16.0):
          for via_first in (None, (186.0, 353.0)):
            key = ("overlap" if overlapped else "strict", int(h_entry), "viaT" if via_first else "")
            for xE in np.arange(230, 356, 5.0):
                for yE in np.arange(150, 346, 5.0):
                    c = east(float(xE), float(yE), h_entry, overlapped, f"east-{key[0]}-h{key[1]}{'-viaT' if via_first else ''}", via_first)
                    if c is None:
                        continue
                    ev = chk.evaluate(c["Q"], rules_relaxed if overlapped else rules_strict)
                    if not ev.ok:
                        continue
                    # relaxed candidates must still keep the swept circle clear of the box while turning
                    if overlapped and ev.turn_clearance_mm < rules_relaxed.sweep_margin_mm:
                        continue
                    sc = score(c["Q"])
                    cost = sc["t50_s"] - 0.01 * min(ev.turn_clearance_mm, 40)   # tie-break toward more clearance (0.1 s per 10 mm)
                    if key not in best or cost < best[key][0]:
                        best[key] = (cost, c, ev, sc)
    for key, (cost, c, ev, sc) in sorted(best.items()):
        cands.append(dict(name=c["label"], family="east", via=[list(map(float, v)) for v in c["via"]], Q=c["Q"], eval=ev, score=sc,
                          params=dict(turn_point_xy=[c["xE"], c["yE"]], h_entry_mm=c["h_entry"], overlapped=c["overlapped"], via_taught_corridor=("viaT" in c["label"])),
                          rules="relaxed (turn during the descent, swept circle >= 10 mm from the zone)" if c["overlapped"] else "strict"))

    # taught corridor, re-segmented: south exit, turn near (160, 80) as today, descend at (300,160), approach the stage
    z_top = Hm[2]
    try:
        S1 = fk.ik([165, 186, z_top, 0, 0, yaw_m], 1, j4_ref=Qm[3])
        T1 = fk.ik([160, 80, z_top, 0, 0, yaw_m], 1, j4_ref=Qm[3])
        T2 = fk.ik([160, 80, z_top, 0, 0, -52.5], 1, j4_ref=Qp[3])      # the taught route's yaw at its turn point (wp 20)
        D1 = fk.ik([300, 160, z_top, 0, 0, yaw_p], 1, j4_ref=Qp[3])
        D2 = fk.ik([300, 160, 180, 0, 0, yaw_p], 1, j4_ref=Qp[3])
        via = [Qm, S1, T1, T2, D1, D2, Qp]
        Q = segment(via)
        cands.append(dict(name="taught-corridor-resegmented", family="corridor", via=[list(map(float, v)) for v in via], Q=Q,
                          eval=chk.evaluate(Q, rules_strict), score=score(Q), params=dict(turn_point_xy=[160, 80]), rules="strict"))
    except ValueError as e:
        print("corridor candidate failed:", e)

    # existing routes for comparison (evaluated as they are)
    for name in ("microscope_to_polar", "microscope_to_polar_fast"):
        Q = np.asarray(LAY["trajectories"][name]["joints"], float)[:, :4]
        cands.append(dict(name=f"current:{name}", family="taught", via=None, Q=Q, eval=chk.evaluate(Q, rules_strict), score=score(Q),
                          params={}, rules="strict (as evaluated; the taught route predates these rules)"))
    return cands, dict(Hm=Hm, Hp=Hp, Qm=Qm, Qp=Qp)


# ----------------------------------------------------------------------------- route A candidates
def build_route_a_candidates(chk: Checker):
    pol = LAY["fixtures"]["polar"]; hot = LAY["fixtures"]["hotel"]
    Hs = hover(pol, "pose_at_90")
    a1 = list(hot["taught"]["A1"]); ax, mm, lift = hot["approach"]["axis"], hot["approach"]["mm"], hot["lift_mm"]
    He = [a1[0] + ax[0] * mm, a1[1] + ax[1] * mm, a1[2] + lift, 0.0, 0.0, a1[5]]
    Qs = fk.ik(Hs, 1, j4_ref=4.0); Qe = fk.ik(He, 1, j4_ref=40.0)
    # Rack geometry is not in layout.json, so the rack cannot be modelled. What IS known is the region the taught
    # route A plus the cycle's J move to the approach point has swept 984 times without contact: the plate must stay
    # inside that proven corridor (dilated 5 mm). Anything outside needs a physical check before it can be trusted.
    # The corridor is the taught route, plus the two legs the cycle executes around it today: the J move from the CURRENT
    # stage hover to the route's first waypoint (cycle._arrive; the hover moved when the seat was re-taught) and the J move
    # from the route's last waypoint to the approach point. The first leg is not yet proven with the re-taught seat; it is
    # what the cell will sweep at its next run with the existing route regardless.
    Qt = np.vstack([Qs, np.asarray(LAY["trajectories"]["polar_to_hotel"]["joints"], float)[:, :4], Qe])
    St, _ = chk.sample(Qt)
    xt, yt = chk._xy(St[:, 1], St[:, 2]); yawt = St[:, 1:4].sum(1)
    corridor = Corridor(margin_mm=5.0); corridor.add_rectangles(plate_corners(xt, yt, yawt, "long_edges")); corridor.finalize()
    print("route A: proven corridor = %d mm2 swept by hover->taught route->J leg (+5 mm dilation)" % int(corridor.mask.sum()))
    rules = Rules(grip="long_edges", strict_turn=False, mic_plate_rule=False, hotel_rule=False, corridor=corridor)
    cands = []
    Q = segment([Qs, Qe])
    cands.append(dict(name="direct", family="A", via=[Qs, Qe], Q=Q, eval=chk.evaluate(Q, rules), score=score(Q), params={}, rules="inside the proven corridor; TCP outside the microscope box"))
    best = None
    for xv in np.arange(200, 321, 5.0):
        for yv in np.arange(180, 321, 5.0):
            for yaw_v in (Hs[5], -75.0, -60.0, -45.0, (Hs[5] + He[5]) / 2, -20.0, He[5]):
                try:
                    V = fk.ik([xv, yv, Hs[2], 0, 0, yaw_v], 1, j4_ref=20.0)
                except ValueError:
                    continue
                Q = segment([Qs, V, Qe])
                ev = chk.evaluate(Q, rules)
                if not ev.ok:
                    continue
                sc = score(Q)
                if best is None or sc["t50_s"] < best[0]:
                    best = (sc["t50_s"], V, Q, ev, sc, (xv, yv, yaw_v))
    if best:
        _, V, Q, ev, sc, prm = best
        cands.append(dict(name="one-via", family="A", via=[Qs, list(V), Qe], Q=Q, eval=ev, score=sc,
                          params=dict(via_xy=[float(prm[0]), float(prm[1])], via_yaw=float(prm[2])), rules="inside the proven corridor; TCP outside the microscope box"))
    Q = np.asarray(LAY["trajectories"]["polar_to_hotel"]["joints"], float)[:, :4]
    # the taught route is followed by a J move to the approach point: add it as a final (joint-space) leg for a fair time
    Qfull = np.vstack([Qs, Q, Qe])
    cands.append(dict(name="current:hover+polar_to_hotel+J approach", family="taught", via=None, Q=Qfull, eval=chk.evaluate(Qfull, rules),
                      score=score(Qfull), params={}, rules="defines the proven corridor"))
    return cands, dict(Hs=Hs, He=He, Qs=Qs, Qe=Qe)


# ----------------------------------------------------------------------------- reporting
def table(cands, title):
    print(f"\n== {title}")
    hdr = "%-30s %-14s %4s %8s %8s %6s %7s %7s %9s %9s %9s %8s  %s" % ("candidate", "turn/via xy", "wp", "jtrav", "ztrav", "nsub", "t50 s", "t20 s", "lat<hov", "tcp<hov", "turn clr", "overbox", "status")
    print(hdr)
    for c in cands:
        s, e = c["score"], c["eval"]
        f = lambda v: ("%9.1f" % v) if np.isfinite(v) else "      n/a"
        pxy = c["params"].get("turn_point_xy") or c["params"].get("via_xy")
        print("%-30s %-14s %4d %8.1f %8.1f %6d %7.1f %7.1f %s %s %s %8d  %s" % (c["name"], ("%.0f,%.0f" % tuple(pxy)) if pxy else "-", s["waypoints"], s["joint_travel_deg"], s["column_travel_mm"],
              s["substeps"], s["t50_s"], s["t20_s"], f(e.min_lateral_below_hover_mm), f(e.min_tcp_clearance_mm), f(e.turn_clearance_mm),
              e.samples_over_box, "OK" if e.ok else "; ".join(e.violations)))


def dump(cands, path, extra=None):
    out = []
    for c in cands:
        e = c["eval"].__dict__
        out.append(dict(name=c["name"], family=c["family"], rules=c["rules"], params=c["params"], score=c["score"], eval=e,
                        joints=[[round(float(v), 4) for v in q] for q in c["Q"]], poses=poses_of(c["Q"]), via=c["via"]))
    with open(path, "w", encoding="utf-8") as f:
        json.dump(dict(candidates=out, **(extra or {})), f, indent=1)


def main():
    chk = Checker()
    print("microscope hover (seat + hover_mm): %s  z_hover %.3f  yaw %.3f" % ([round(v, 3) for v in hover(LAY['fixtures']['microscope'])[:3]], chk.z_hover, chk.yaw_seat))
    cb, info_b = build_route_b_candidates(chk)
    print("route B start joints (taught branch):", [round(v, 3) for v in info_b["Qm"]], " end:", [round(v, 3) for v in info_b["Qp"]])
    table(cb, "route B: microscope hover -> polar hover (stress profile 50 %, model t = sum over 8-unit sub-moves)")
    # recommended: the strict east route with lateral entry at the hover height (the band PLACEMENT.md documents as free,
    # z 300..335, and the height the taught route used for 492 cycles). h16 and the overlapped variant are written too.
    byname = {c["name"]: c for c in cb if c["eval"].ok}
    outputs = [("east-strict-h0", "microscope_to_polar_opt.json",
                "Joint-space straight moves: microscope hover -> turn point (yaw fixed at the seat yaw, z = hover height) -> yaw turn in place "
                "above the stage at hover height -> descent to the polar hover (plate aligned, <= 15 mm lateral drift). Every rule of the brief holds."),
               ("east-strict-h16", "microscope_to_polar_opt_h16.json",
                "Variant: 16 mm straight up first, swing and turn at hover + 16 mm, so the reverse replay ends with a purely vertical 16 mm "
                "drop onto the microscope hover. Costs 2 sub-moves each way and leaves the documented free band z 300..335."),
               ("east-overlap-h0", "microscope_to_polar_opt_overlap.json",
                "Variant: the yaw turn happens DURING the descent above the stage (173 mm from the microscope seat, swept circle 15.7 mm "
                "clear of the zone box). Violates the brief's 'turn above hover height' rule by design; needs that rule relaxed explicitly.")]
    for name, fname, how in outputs:
        rec = byname.get(name)
        if not rec:
            print("no feasible", name); continue
        prov = ("Generated offline 2026-10-19 by routes_opt/route_gen.py from the fitted FK (routes_opt/fk_model.json) and layout.json "
                "(microscope seat taught 2026-10-19 20:42, polar pose_at_0 taught 2026-10-19 20:53). NOT replayed on the arm. " + how +
                " Validate per routes_opt/REPORT.md section 7 before any use; re-run route_gen.py after any re-teach.")
        tr = as_trajectory(rec["Q"], prov, note="%s; turn point xy %s; %d sub-moves; modelled %.0f s at 50 %%, %.0f s at 20 %%" %
                           (name, rec["params"]["turn_point_xy"], rec["score"]["substeps"], rec["score"]["t50_s"], rec["score"]["t20_s"]))
        tr["candidate"] = name; tr["via_points_joints"] = rec["via"]
        tr["rules_checked"] = ["workspace box", "region z floors", "plate rectangle never over the microscope box below hover height",
                               "seat yaw while the plate is over the microscope box",
                               "yaw turn only at/above hover height with the plate's swept circle >= 15 mm from the box" if "strict" in name else
                               "yaw turn during the descent (relaxed), swept circle >= 15 mm from the box",
                               "elbow in the taught branch", "joint envelope of the taught data (j4 <= 184)", "waypoint step <= 24"]
        with open(os.path.join(HERE, fname), "w", encoding="utf-8") as f:
            json.dump(tr, f, indent=1)
        print("wrote %s  (%s)" % (fname, name))
    rec = byname.get("east-strict-h0")
    if rec:
        print("\nrecommended east-strict-h0: via points and per-leg cost (50 %% profile)")
        via = [np.asarray(v) for v in rec["via"]]
        for v in via:
            print("  via ", [round(float(a), 3) for a in v], "->", [round(a, 1) for a in fk.fk(v)[:3]], "yaw %.2f" % fk.fk(v)[5])
        for i in range(len(via) - 1):
            d = np.abs(via[i + 1] - via[i]); Qseg = segment([via[i], via[i + 1]]); sc = score(Qseg)
            print("  leg %d: d(j1..j4) = %s  waypoints %d  sub-moves %d  %.1f s (20 %%: %.1f s)" % (i + 1, [round(float(a), 1) for a in d], sc["waypoints"] - 1, sc["substeps"], sc["t50_s"], sc["t20_s"]))
        print("  waypoints:")
        for i, q in enumerate(rec["Q"], 1):
            pp = fk.fk(q); print("   %2d  j %s  xyz %s  yaw %7.2f" % (i, [round(float(a), 2) for a in q], [round(a, 1) for a in pp[:3]], pp[5]))
        print("  joint min %s max %s" % ([round(v, 1) for v in rec["eval"].joint_min], [round(v, 1) for v in rec["eval"].joint_max]))
    dump(cb, os.path.join(HERE, "candidates_microscope_to_polar.json"), dict(hover_microscope=info_b["Hm"], hover_polar=info_b["Hp"]))

    ca, info_a = build_route_a_candidates(chk)
    print("\nroute A start joints:", [round(v, 3) for v in info_a["Qs"]], " end (A1 approach):", [round(v, 3) for v in info_a["Qe"]])
    table(ca, "route A: polar hover (stage at 90, long edges) -> hotel A1 approach point")
    oks = [c for c in ca if c["eval"].ok and c["family"] == "A"]
    reca = min(oks, key=lambda c: c["score"]["t50_s"]) if oks else None
    if reca:
        prov = ("Generated offline 2026-10-19 by routes_opt/route_gen.py from the fitted FK and layout.json (polar pose_at_90, hotel A1 + approach). "
                "NOT replayed. The plate stays inside the region swept by the current stage hover -> taught route A -> J approach leg (the route part: 984 traversals), dilated 5 mm; the rack geometry itself is not in layout.json. "
                "Ends exactly at the A1 approach point, so the cycle's J move there becomes a no-op.")
        tr = as_trajectory(reca["Q"], prov, note="%s; %d sub-moves; modelled %.0f s at 50 %%" % (reca["name"], reca["score"]["substeps"], reca["score"]["t50_s"]))
        tr["candidate"] = reca["name"]; tr["via_points_joints"] = [list(map(float, v)) for v in reca["via"]]
        with open(os.path.join(HERE, "polar_to_hotel_opt.json"), "w", encoding="utf-8") as f:
            json.dump(tr, f, indent=1)
        print("\nwrote polar_to_hotel_opt.json  (%s)" % reca["name"])
    dump(ca, os.path.join(HERE, "candidates_polar_to_hotel.json"), dict(hover_polar90=info_a["Hs"], a1_approach=info_a["He"]))


if __name__ == "__main__":
    main()
