"""
Build the vision-gate reference library from logged cycles and measure how well each gate separates its states.

For every run since --since, the per-step images saved by telemetry are labelled from the step name and the
stage angle timeline, then scored with the same normalised cross-correlation the gates use:

  gate/state                 source frame
  microscope/plate_present   pick:microscope lower (before pick), place:microscope placed (after release)
  microscope/empty           place:microscope lower (before place)
  polar@0/plate_present      pick:polar lower at 0 deg, place:polar placed at 0 deg
  polar@0/empty              place:polar lower at 0 deg
  polar@90/...               same at 90 deg
  gripper/short_edges        pick:microscope lifted, pick:polar lifted at 0 deg
  gripper/long_edges         pick:polar lifted at 90 deg, pick:hotel lifted

Output: per gate/state, the correlation of every frame against K seed references (true class) and against the
opposite state's references (other class), the resulting threshold, and K references copied into
reference_images/. Nothing is deleted.

    python build_refs_from_runs.py --since 20261015_0300 --k 12 --max-per-class 400
"""
import argparse
import json
import os
import random
import re
import shutil
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from nova_cell.lower_check import _prep, ncc  # noqa: E402

RUNS = os.path.join(HERE, "runs")
REFS = os.path.join(HERE, "reference_images")

PAT = re.compile(r"^(\d{8})_(.+)\.jpg$")


def polar_timeline(events):
    """[(t, angle_after)] from 'polar rotate' messages; the first message also gives the angle before it."""
    tl = []
    for r in events:
        if r["kind"] == "polar" and r["step"] == "rotate":
            m = re.match(r"(\d+) -> (\d+)", r.get("msg", ""))
            if m:
                if not tl:
                    tl.append((0.0, int(m.group(1))))
                tl.append((r["t"], int(m.group(2))))
    if not tl:
        tl.append((0.0, 0))
    return tl


def angle_at(tl, t):
    a = tl[0][1]
    for tt, ang in tl:
        if tt <= t:
            a = ang
        else:
            break
    return a


def label(name, angle):
    s = name
    if s == "pick_microscope_lower_nogate":
        return ("microscope", "plate_present")
    if s == "place_microscope_placed":
        return ("microscope", "plate_present")
    if s == "place_microscope_lower_nogate":
        return ("microscope", "empty")
    g = f"polar@{angle}"
    if s == "pick_polar_lower_nogate" or s == "place_polar_placed":
        return (g, "plate_present")
    if s == "place_polar_lower_nogate":
        return (g, "empty")
    if s == "pick_microscope_lifted":
        return ("gripper", "short_edges")
    if s == "pick_polar_lifted":
        return ("gripper", "short_edges" if angle == 0 else "long_edges")
    if s.startswith("pick_hotel") and s.endswith("lifted"):
        return ("gripper", "long_edges")
    return None


def collect(since):
    items = {}
    for d in sorted(os.listdir(RUNS)):
        if not d[:4].isdigit() or d < since:
            continue
        ev = os.path.join(RUNS, d, "events.jsonl")
        img_root = os.path.join(RUNS, d, "images")
        if not (os.path.exists(ev) and os.path.isdir(img_root)):
            continue
        events = [json.loads(l) for l in open(ev, encoding="utf-8")]
        t0 = min(r["t"] for r in events)
        tl = polar_timeline(events)
        for cyc in sorted(os.listdir(img_root)):
            for f in sorted(os.listdir(os.path.join(img_root, cyc))):
                m = PAT.match(f)
                if not m:
                    continue
                t = t0 + int(m.group(1)) / 1000.0
                lab = label(m.group(2), angle_at(tl, t))
                if lab:
                    items.setdefault(lab, []).append((t, os.path.join(img_root, cyc, f)))
    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="20261015_0300")
    ap.add_argument("--k", type=int, default=12, help="references per gate/state")
    ap.add_argument("--max-per-class", type=int, default=400)
    ap.add_argument("--write", action="store_true", help="copy references into reference_images/")
    a = ap.parse_args()
    random.seed(7)
    items = collect(a.since)
    print("labelled frames:", {f"{g}/{s}": len(v) for (g, s), v in sorted(items.items())})

    prepped = {}
    for key, lst in items.items():
        lst.sort()
        if len(lst) > a.max_per_class:
            step = len(lst) / a.max_per_class
            lst = [lst[int(i * step)] for i in range(a.max_per_class)]
        arrs = []
        for t, p in lst:
            try:
                arrs.append((t, p, _prep(open(p, "rb").read())))
            except Exception:
                pass
        prepped[key] = arrs

    report = {}
    for (gate, state), arrs in sorted(prepped.items()):
        if len(arrs) < a.k + 5:
            print(f"{gate}/{state}: only {len(arrs)} frames, skipped")
            continue
        n = len(arrs)
        seed_idx = [int(i * (n - 1) / (a.k - 1)) for i in range(a.k)]        # spread over the whole period
        seeds = [arrs[i][2] for i in seed_idx]
        rest = [x for i, x in enumerate(arrs) if i not in seed_idx]
        same = [max(ncc(x[2], s_) for s_ in seeds) for x in rest]
        other_states = [k2 for k2 in prepped if k2[0] == gate and k2[1] != state]
        other = []
        for k2 in other_states:
            for x in prepped[k2][: 200]:
                other.append(max(ncc(x[2], s_) for s_ in seeds))
        pair = min(ncc(seeds[i], seeds[j]) for i in range(len(seeds)) for j in range(i + 1, len(seeds)))
        thr = max(0.55, pair - 0.08)
        tp = sum(1 for v in same if v >= thr) / len(same)
        fp = (sum(1 for v in other if v >= thr) / len(other)) if other else None
        sep = (min(same) - max(other)) if other else None
        report[f"{gate}/{state}"] = {
            "frames": n, "seed_pairwise_min": round(pair, 3), "threshold": round(thr, 3),
            "same_min": round(min(same), 3), "same_p1": round(sorted(same)[max(0, len(same) // 100)], 3),
            "same_median": round(st.median(same), 3),
            "other_max": round(max(other), 3) if other else None, "other_median": round(st.median(other), 3) if other else None,
            "true_pass_rate": round(tp, 4), "false_pass_rate": round(fp, 4) if fp is not None else None,
            "separation": round(sep, 3) if sep is not None else None,
        }
        print(f"{gate}/{state}: n={n} thr={thr:.3f} same[min p1 med]={min(same):.3f} {report[f'{gate}/{state}']['same_p1']:.3f} "
              f"{st.median(same):.3f}  other[max med]={(max(other) if other else float('nan')):.3f} "
              f"{(st.median(other) if other else float('nan')):.3f}  pass={tp:.3f} falsepass={fp}")
        if a.write:
            d = os.path.join(REFS, gate, state)
            os.makedirs(d, exist_ok=True)
            for old in os.listdir(d):
                if old.startswith("ref_") and old.endswith(".png"):
                    os.remove(os.path.join(d, old))
            for i, si in enumerate(seed_idx, 1):
                shutil.copy(arrs[si][1], os.path.join(d, f"ref_{i:03d}_from_runs.jpg"))
    json.dump(report, open(os.path.join(RUNS, "gate_report.json"), "w", encoding="utf-8"), indent=2)
    print("written", os.path.join(RUNS, "gate_report.json"))


if __name__ == "__main__":
    main()
