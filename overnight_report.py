"""
Read-only overnight watch. Summarises the live loop into runs/OVERNIGHT.md every `--every` seconds and never
starts, stops or talks to the arm. Safe to leave running next to run_forever.py.

    python overnight_report.py --since 20261019_0540 --every 1800
"""
import argparse
import json
import os
import statistics as st
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")


def loop_alive() -> bool:
    """The live loop holds runs/run.lock; probing the lock is the one check that cannot be fooled by a stale PID."""
    import msvcrt
    path = os.path.join(RUNS, "run.lock")
    try:
        f = open(path, "a+")
    except OSError:
        return False
    try:
        msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
    except OSError:
        f.close()
        return True
    msvcrt.locking(f.fileno(), msvcrt.LK_UNLCK, 1)
    f.close()
    return False


def runs_since(since: str):
    return [d for d in sorted(os.listdir(RUNS)) if d[:4].isdigit() and d >= since and os.path.isdir(os.path.join(RUNS, d))]


def events(run):
    out = []
    try:
        with open(os.path.join(RUNS, run, "events.jsonl"), encoding="utf-8") as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    except OSError:
        pass
    return out


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))] if xs else float("nan")


def summarise(since: str) -> str:
    lines = [f"# Overnight watch  (written {time.strftime('%Y-%m-%d %H:%M:%S')}, runs since {since})", ""]
    lines.append(f"- loop alive (run.lock held): **{loop_alive()}**")
    free = __import__("shutil").disk_usage(RUNS).free / 1e9
    lines.append(f"- disk free on runs volume: {free:.0f} GB")
    runs = runs_since(since)
    ev = [e for r in runs for e in events(r)]
    cycles = [e for e in ev if e["kind"] == "cycle" and e["step"] == "done"]
    dts = [e["dt"] for e in cycles]
    if dts:
        lines.append(f"- cycles done: {len(dts)}; dt median {st.median(dts):.1f} s, min {min(dts):.1f}, max {max(dts):.1f}; "
                     f"last cycle {cycles[-1]['cycle']} at {time.strftime('%H:%M:%S', time.localtime(cycles[-1]['t']))}")
    else:
        lines.append("- cycles done: 0")
    stops = [e for e in ev if e["kind"] == "stop"]
    faults = [e for e in ev if e["kind"] == "fault"]
    lines.append(f"- stops: {len(stops)}" + "".join(f"\n    - {time.strftime('%H:%M:%S', time.localtime(e['t']))} c{e['cycle']} {e['step']}: {e['msg']}" for e in stops[-10:]))
    lines.append(f"- faults: {len(faults)}" + "".join(f"\n    - {time.strftime('%H:%M:%S', time.localtime(e['t']))} c{e['cycle']} {e['step']}: {e['msg']} {e.get('detail', '')}" for e in faults[-10:]))
    reissued = [e for e in ev if e["kind"] == "move" and "re-issu" in str(e.get("msg", ""))]
    lines.append(f"- SDK stops re-issued: {len(reissued)}")
    grasps = [e for e in ev if e["kind"] == "jaw" and "close" in e["step"] and isinstance(e.get("jaw"), (int, float))]
    if grasps:
        by = {}
        for e in grasps:
            by.setdefault(e.get("grip", e["step"]), []).append(e["jaw"])
        for k, v in by.items():
            lines.append(f"- grasp {k}: n {len(v)}, median {st.median(v):.3f}, min {min(v):.3f}, max {max(v):.3f}")
    lines.append("")
    lines.append("## Slot gate (front camera at the hotel approach)")
    sv = [e for e in ev if e["kind"] == "vision" and e["step"].endswith("slot_view") and e.get("score_empty") is not None]
    refs = [e for e in ev if e["kind"] == "vision" and e["step"].endswith("slot_ref")]
    lines.append(f"- references collected this watch: {len(refs)}" + (f" (last: {refs[-1]['msg']})" if refs else ""))
    for label in ("place", "pick"):
        rows = [e for e in sv if e["step"].startswith(label)]
        if not rows:
            continue
        dec = {}
        for e in rows:
            dec[e["msg"]] = dec.get(e["msg"], 0) + 1
        se, so = [e["score_empty"] for e in rows], [e["score_occupied"] for e in rows]
        lines.append(f"- {label} approach: n {len(rows)}, decisions {dec}; score_empty median {st.median(se):.3f} "
                     f"(p5 {q(se, 0.05):.3f}), score_occupied median {st.median(so):.3f} (p95 {q(so, 0.95):.3f})")
    lines.append("")
    lines.append("## Seated check (front camera at release, shadow)")
    sf = [e for e in ev if e["kind"] == "vision" and e["step"].endswith("seated_front") and e.get("score") is not None]
    by = {}
    for e in sf:
        by.setdefault(e.get("gate"), []).append((e["score"], e.get("thr"), e.get("safe")))
    for g, rows in by.items():
        sc = [r[0] for r in rows]
        unsafe = sum(1 for r in rows if r[2] is False)
        lines.append(f"- {g}: n {len(rows)}, score median {st.median(sc):.3f}, min {min(sc):.3f}, thr {rows[-1][1]}, unsafe verdicts {unsafe}")
    lines.append("")
    lines.append("## Supervisor log tail")
    try:
        tail = open(os.path.join(RUNS, "forever.log"), encoding="utf-8").read().splitlines()[-6:]
    except OSError:
        tail = []
    lines += [f"    {t}" for t in tail]
    alert = os.path.join(RUNS, "ALERT_SUPERVISOR.txt")
    if os.path.exists(alert):
        lines.append("")
        lines.append("## ALERT_SUPERVISOR.txt tail")
        lines += [f"    {t}" for t in open(alert, encoding="utf-8").read().splitlines()[-3:]]
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default=time.strftime("%Y%m%d_%H%M"))
    ap.add_argument("--every", type=float, default=1800.0)
    ap.add_argument("--once", action="store_true")
    a = ap.parse_args()
    while True:
        text = summarise(a.since)
        with open(os.path.join(RUNS, "OVERNIGHT.md"), "w", encoding="utf-8") as f:
            f.write(text)
        print(text, flush=True)
        if a.once:
            return 0
        time.sleep(a.every)


if __name__ == "__main__":
    sys.exit(main())
