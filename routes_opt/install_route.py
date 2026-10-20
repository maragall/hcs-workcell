"""
Copy a generated route into layout.json under a NEW trajectory name so `run.py replay <name>` can drive it.
Never overwrites an existing trajectory; never touches the originals. Dry run unless --yes is given.

    python routes_opt/install_route.py microscope_to_polar_opt.json --name microscope_to_polar_opt --yes
    python run.py replay microscope_to_polar_opt --speed 0.10 --step --max-dev 2
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LAYOUT = os.path.join(ROOT, "layout.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", help="route json in routes_opt/ (layout trajectory format)")
    ap.add_argument("--name", required=True, help="new trajectory name (must not exist yet)")
    ap.add_argument("--yes", action="store_true", help="actually write layout.json (a timestamped backup is kept)")
    a = ap.parse_args()
    src = a.file if os.path.isabs(a.file) else os.path.join(HERE, a.file)
    tr = json.load(open(src, encoding="utf-8"))
    for k in ("joints", "poses", "spread_deg", "max_step_deg", "demos"):
        assert k in tr, f"{src} lacks {k}"
    assert all(len(j) == 8 for j in tr["joints"]) and all(len(p) == 6 for p in tr["poses"])
    with open(LAYOUT, encoding="utf-8") as f:
        raw = f.read()
    lay = json.loads(raw)
    if a.name in lay["trajectories"]:
        raise SystemExit(f"trajectory '{a.name}' already exists in layout.json; pick another name")
    entry = {k: tr[k] for k in ("joints", "poses", "spread_deg", "max_step_deg", "demos")}
    entry["note"] = f"{tr.get('provenance', '')} | {tr.get('note', '')} | installed {time.strftime('%Y-%m-%d %H:%M:%S')} from {os.path.basename(src)}"
    print(f"{os.path.basename(src)}: {len(entry['joints'])} waypoints, max step {entry['max_step_deg']:.2f}, first {[round(v, 1) for v in entry['poses'][0][:3]]} last {[round(v, 1) for v in entry['poses'][-1][:3]]}")
    if not a.yes:
        print(f"dry run: would add trajectories.{a.name} to {LAYOUT}. Re-run with --yes to write.")
        return
    backup = LAYOUT + ".bak." + time.strftime("%Y%m%d_%H%M%S")
    shutil.copy2(LAYOUT, backup)
    lay["trajectories"][a.name] = entry
    with open(LAYOUT, "w", encoding="utf-8") as f:
        json.dump(lay, f, indent=2)
    print(f"added trajectories.{a.name}; backup at {backup}")


if __name__ == "__main__":
    main()
