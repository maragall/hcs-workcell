"""
Slot-occupancy check from the front camera at the hotel approach point, calibrated on logged runs.

Labels: the approach before a hotel *place* sees an empty slot; the approach before a hotel *pick* sees an
occupied slot. Frames come from the continuous recording (scan_camera) at the time of the approach move.

Method: the same normalised cross-correlation as the gates, but as a two-class decision: score against K
"empty" references and K "occupied" references, decide by the larger score, and report the margin. Also
reports the best single-class threshold for reuse by `live_detection`-free gating.

    python slot_occupancy.py --since 20261015_0300 --k 10 --write
"""
import argparse
import bisect
import json
import os
import shutil
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from nova_cell.lower_check import _prep, ncc  # noqa: E402

RUNS = os.path.join(HERE, "runs")
REFS = os.path.join(HERE, "reference_images")


def frames_for(run, label_steps):
    ev_path = os.path.join(RUNS, run, "events.jsonl")
    idx_path = os.path.join(RUNS, run, "video", "scan_camera", "index.json")
    if not (os.path.exists(ev_path) and os.path.exists(idx_path)):
        return []
    events = [json.loads(l) for l in open(ev_path, encoding="utf-8")]
    idx = json.load(open(idx_path))
    ts = [t for n, t in idx]
    out = []
    for r in events:
        if r["kind"] != "move":
            continue
        for step, lab in label_steps:
            if r["step"] == step:
                t = r["t"] + 0.3                      # just after the approach move settled
                i = max(0, min(bisect.bisect_left(ts, t), len(ts) - 1))
                if abs(ts[i] - t) <= 2.0:
                    out.append((lab, t, os.path.join(RUNS, run, "video", "scan_camera", f"{idx[i][0]:06d}.jpg")))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="20261015_0300")
    ap.add_argument("--slot", default="A1")
    ap.add_argument("--k", type=int, default=10)
    ap.add_argument("--max-per-class", type=int, default=300)
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    steps = [(f"place:hotel:{a.slot} approach", "empty"), (f"pick:hotel:{a.slot} approach", "occupied")]
    items = {"empty": [], "occupied": []}
    for run in sorted(os.listdir(RUNS)):
        if run[:4].isdigit() and run >= a.since:
            for lab, t, p in frames_for(run, steps):
                items[lab].append((t, p))
    print("frames:", {k: len(v) for k, v in items.items()})
    prepped = {}
    for lab, lst in items.items():
        lst.sort()
        if len(lst) > a.max_per_class:
            step = len(lst) / a.max_per_class
            lst = [lst[int(i * step)] for i in range(a.max_per_class)]
        prepped[lab] = [(t, p, _prep(open(p, "rb").read())) for t, p in lst if os.path.exists(p)]
    if min(len(v) for v in prepped.values()) < a.k + 5:
        sys.exit("not enough frames")
    seeds = {}
    for lab, arrs in prepped.items():
        n = len(arrs)
        si = [int(i * (n - 1) / (a.k - 1)) for i in range(a.k)]
        seeds[lab] = ([arrs[i][2] for i in si], set(si))
    report = {}
    for lab, arrs in prepped.items():
        other = "occupied" if lab == "empty" else "empty"
        margins, same_scores = [], []
        for i, (t, p, x) in enumerate(arrs):
            if i in seeds[lab][1]:
                continue
            s_same = max(ncc(x, r) for r in seeds[lab][0])
            s_other = max(ncc(x, r) for r in seeds[other][0])
            same_scores.append(s_same)
            margins.append(s_same - s_other)
        correct = sum(1 for m in margins if m > 0) / len(margins)
        report[lab] = {"n": len(margins), "accuracy_two_class": round(correct, 4),
                       "margin_min": round(min(margins), 4), "margin_p5": round(sorted(margins)[len(margins) // 20], 4),
                       "margin_median": round(st.median(margins), 4),
                       "same_min": round(min(same_scores), 4), "same_median": round(st.median(same_scores), 4)}
        print(lab, report[lab])
    json.dump(report, open(os.path.join(RUNS, "slot_occupancy_report.json"), "w"), indent=2)
    if a.write:
        for lab, arrs in prepped.items():
            d = os.path.join(REFS, f"slot_{a.slot}", lab)
            os.makedirs(d, exist_ok=True)
            for i, si in enumerate(sorted(seeds[lab][1]), 1):
                shutil.copy(arrs[si][1], os.path.join(d, f"ref_{i:03d}.jpg"))
        print("references written under", os.path.join(REFS, f"slot_{a.slot}"))


if __name__ == "__main__":
    main()
