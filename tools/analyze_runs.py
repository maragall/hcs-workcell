"""
Aggregate every run's events.jsonl into cycle, step and fault statistics.

    python tools/analyze_runs.py [--since 20261015_0300] [--out runs/analysis.json]
"""
import argparse
import json
import os
import statistics as st
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # the repo root; this file is in tools/
RUNS = os.path.join(HERE, "runs")


def load(since):
    for d in sorted(os.listdir(RUNS)):
        if not d[:4].isdigit() or d < since:
            continue
        p = os.path.join(RUNS, d, "events.jsonl")
        if not os.path.exists(p):
            continue
        with open(p, encoding="utf-8") as f:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                r["run"] = d
                yield r


def q(xs, p):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    k = (len(xs) - 1) * p
    i = int(k)
    return xs[i] if i == len(xs) - 1 else xs[i] + (xs[i + 1] - xs[i]) * (k - i)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="20261015_0300")
    ap.add_argument("--out", default=os.path.join(RUNS, "analysis.json"))
    a = ap.parse_args()
    rows = list(load(a.since))
    runs = sorted({r["run"] for r in rows})
    cycles = [r for r in rows if r["kind"] == "cycle"]
    stops = [r for r in rows if r["kind"] == "stop"]
    faults = [r for r in rows if r["kind"] == "fault"]
    alerts = [r for r in rows if r["kind"] == "alert"]
    moves = [r for r in rows if r["kind"] == "move" and "dt" in r]
    jaws = [r for r in rows if r["kind"] == "jaw"]

    step_dt = defaultdict(list)
    step_dev = defaultdict(list)
    for m in moves:
        key = m["step"].split("[")[0]
        step_dt[key].append(m["dt"])
        if "dev" in m:
            step_dev[key].append(m["dev"])
    route_time = defaultdict(list)
    for m in moves:
        if "[" in m["step"]:
            route_time[m["step"].split("[")[0]].append(m["dt"])

    # per-cycle time budget: sum of move dt grouped by family
    fam = Counter()
    for m in moves:
        s = m["step"]
        if "[" in s:
            fam["route:" + s.split("[")[0]] += m["dt"]
        elif "hover" in s or "lift" in s or "lower" in s or "slide" in s or "approach" in s or "clear" in s:
            fam["fixture:" + s.split()[0]] += m["dt"]
        else:
            fam["other:" + s] += m["dt"]
    n_cycles = max(1, len(cycles))
    budget = {k: round(v / n_cycles, 1) for k, v in sorted(fam.items(), key=lambda kv: -kv[1])}

    jaw_closed = defaultdict(list)
    for j in jaws:
        if j["step"] == "close":
            jaw_closed[j.get("grip", "?")].append(j["jaw"])

    first_t = min(r["t"] for r in rows) if rows else 0
    last_t = max(r["t"] for r in rows) if rows else 0
    out = {
        "runs": len(runs), "first_run": runs[0] if runs else None, "last_run": runs[-1] if runs else None,
        "wall_hours": round((last_t - first_t) / 3600, 2),
        "cycles_completed": len(cycles),
        "cycle_dt": {"median": round(q([c["dt"] for c in cycles], 0.5), 1), "p10": round(q([c["dt"] for c in cycles], 0.1), 1),
                     "p90": round(q([c["dt"] for c in cycles], 0.9), 1), "min": round(min([c["dt"] for c in cycles], default=0), 1),
                     "max": round(max([c["dt"] for c in cycles], default=0), 1)},
        "stops": [{"run": s["run"], "step": s["step"], "msg": s.get("msg", "")[:160], "state": s.get("state")} for s in stops],
        "faults": Counter(f.get("msg", "") for f in faults),
        "alerts": [a_.get("msg", "")[:160] for a_ in alerts][-10:],
        "moves": len(moves),
        "time_budget_per_cycle_s": budget,
        "step_dt": {k: {"n": len(v), "median": round(st.median(v), 2), "p90": round(q(v, 0.9), 2), "max": round(max(v), 2)}
                    for k, v in sorted(step_dt.items(), key=lambda kv: -sum(kv[1]))},
        "step_dev_mm": {k: {"median": round(st.median(v), 3), "p90": round(q(v, 0.9), 3), "max": round(max(v), 3)}
                        for k, v in step_dev.items()},
        "jaw_closed": {k: {"n": len(v), "mean": round(st.mean(v), 3), "sd": round(st.pstdev(v), 4), "min": min(v), "max": max(v)}
                       for k, v in jaw_closed.items()},
        "waypoints_per_cycle": {k: round(len(v) / n_cycles, 1) for k, v in route_time.items()},
    }
    json.dump(out, open(a.out, "w", encoding="utf-8"), indent=2, default=str)
    print(json.dumps({k: out[k] for k in ("runs", "wall_hours", "cycles_completed", "cycle_dt", "moves")}, indent=1))
    print("time budget per cycle (s):")
    for k, v in budget.items():
        print(f"  {v:7.1f}  {k}")
    print("stops:", len(stops))
    for s_ in out["stops"][-8:]:
        print("  ", s_["run"], s_["step"], "|", s_["msg"][:110])
    print("faults:", dict(out["faults"]))
    print("jaw closed:", out["jaw_closed"])
    print("dev p90/max by step:", {k: (v["p90"], v["max"]) for k, v in out["step_dev_mm"].items() if v["max"] > 0.3})
    print("written", a.out)


if __name__ == "__main__":
    main()
