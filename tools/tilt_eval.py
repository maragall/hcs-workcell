"""How well does perception/tilt.py tell a flat plate from a misplaced one at each seat, with the pictures recorded so far?

For every station under reference_images/seat/<station>/ a table over its *_front.jpg pictures: label (the file name's first
part: flat, held, or the operator's placement note q1..q4q1q2, all of which count as misplaced), angle, mean row, left-third
row, right-third row, inlier count, the verdict against the station's flat reference(s) and the two differences it rests on
(dh px, da deg). `held` pictures (the plate still in the fingers before release) get a verdict too, for information: they are
level by construction but may sit at another height. Then the separation: the largest dh and da among the flats (leave-one-out
when 2 or more flats exist, else 0) against the smallest among the misplaced, and the misplaced pictures whose far edge lies
within the flat band (the far edge cannot see a plate resting on the near lip alone).

One PNG per picture goes to --out/<station>/<name>.png (default: the scratchpad folder tilt/): the band, the edge points
(green inliers, red rejected) and the fitted line, so the measurement can be checked by eye.

    python -X utf8 tools/tilt_eval.py                  the tables once
    python -X utf8 tools/tilt_eval.py --watch           re-run every 60 s, print only when the pictures changed
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from perception import tilt as T  # noqa: E402

OUT = os.path.join(os.environ.get("TEMP", HERE), "claude", "C--Users-ASUS-Downloads-bioyond-arm-setup",
                   "087b0f72-90ad-4c92-a3ac-198cc1178181", "scratchpad", "tilt")


def pictures():
    return sorted(glob.glob(os.path.join(T.REFS, "*", "*_front.jpg")))


def signature():
    return tuple((p, os.path.getmtime(p)) for p in pictures())


def evaluate(out):
    lines = []
    for station in sorted({os.path.basename(os.path.dirname(p)) for p in pictures()}):
        paths = sorted(glob.glob(os.path.join(T.REFS, station, "*_front.jpg")))
        st = T.Station.load(station)
        flat_edges = {p: T.far_edge(cv2.imread(p), st.band) for p in paths if os.path.basename(p).startswith("flat_")}
        os.makedirs(os.path.join(out, station), exist_ok=True)
        rows = []
        for p in paths:
            name = os.path.basename(p)[:-len("_front.jpg")]
            label = name.split("_")[0]
            img = cv2.imread(p)
            if label == "flat" and len(st.flats) >= 2:                      # leave-one-out: judge a flat against the others
                v = T.Station(station, [e for q, e in flat_edges.items() if q != p]).verdict(img)
            else:
                v = st.verdict(img)
            T.draw(img, v.edge, os.path.join(out, station, name + ".png"), f"{label} -> {v.state}")
            rows.append((label, name.split("_", 1)[1] if "_" in name else "", v))
        ref = st.ref
        lines.append(f"\n== {station}: {len(paths)} pictures, {len(st.flats)} flat reference(s)"
                     + (f"  ref angle {ref['angle']:+.2f} deg  row {ref['row']:.1f}  left {ref['left']:.1f}  right {ref['right']:.1f}" if ref else "  NO FLAT REFERENCE -> every verdict unsure"))
        lines.append(f"{'label':8s} {'stamp':16s} {'angle':>7s} {'row':>7s} {'left':>7s} {'right':>7s} {'n':>4s}  {'verdict':9s} {'dh px':>6s} {'da deg':>6s}")
        for label, stamp, v in rows:
            e = v.edge
            if e.ok:
                lines.append(f"{label:8s} {stamp:16s} {e.angle:+7.2f} {e.row:7.1f} {e.left:7.1f} {e.right:7.1f} {e.n:4d}  {v.state:9s} {v.dh:6.1f} {v.da:6.2f}")
            else:
                lines.append(f"{label:8s} {stamp:16s} {'edge not found':>31s} {e.n:4d}  {v.state:9s}")
        if ref:
            flats = [v for label, _, v in rows if label == "flat" and v.edge.ok]
            bad = [(label, v) for label, _, v in rows if label not in ("flat", "held") and v.edge.ok]
            fh = max([v.dh for v in flats], default=0.0) if len(flats) >= 2 else 0.0
            fa = max([v.da for v in flats], default=0.0) if len(flats) >= 2 else 0.0
            if bad:
                lines.append(f"separation  height: flats within {fh:.1f} px, misplaced from {min(v.dh for _, v in bad):.1f} px (median {np.median([v.dh for _, v in bad]):.1f})"
                             f"   angle: flats within {fa:.2f} deg, misplaced from {min(v.da for _, v in bad):.2f} deg (median {np.median([v.da for _, v in bad]):.2f})")
                missed = [label for label, v in bad if v.state != "misplaced"]
                lines.append(f"misplaced seen {sum(v.state == 'misplaced' for _, v in bad)}/{len(bad)}" + (f", not seen in the far edge: {' '.join(missed)}" if missed else ""))
            lines.append(f"bands: flat dh <= {T.FLAT_PX} px and da <= {T.FLAT_DEG} deg; misplaced dh >= {T.BAD_PX} px or da >= {T.BAD_DEG} deg")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=OUT, help="folder for the PNGs")
    ap.add_argument("--watch", action="store_true", help="re-run every 60 s, print only when the pictures changed")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    seen = None
    while True:
        sig = signature()
        if sig != seen:
            seen = sig
            print(time.strftime("%Y-%m-%d %H:%M:%S"), f"{len(sig)} pictures, PNGs in {a.out}")
            print(evaluate(a.out), flush=True)
        if not a.watch:
            break
        time.sleep(60)


if __name__ == "__main__":
    main()
