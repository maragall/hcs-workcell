"""How well does perception/occupancy.py separate full from empty at each station, with the pictures recorded so far?

For every station under reference_images/occupancy/ with at least one empty and one full picture of its camera
(perception.occupancy.camera_for: down for microscope and polar, front for the slots):
  empty-vs-empty   leave-one-out: each empty scored against the nearest of the other empties (needs 2 or more empties)
  full-vs-empty    each full scored against the nearest of all empties
  noise            the station's empty noise N (max leave-one-out score, or DEFAULT_NOISE marked * when only one empty exists)
  margin           min full score - max empty score, then the same in units of the noise (/ sd of the empty-vs-empty scores
                   when 3 or more empties exist, else / N); negative = overlap, the station cannot be trusted yet
  verdicts         what the detector says for each empty (leave-one-out) and each full: e = empty, f = full, ? = unsure
With --deep the level-2 PatchCore score (ResNet18 patches, see occupancy.py) is printed the same way in a second table.

One PNG per station goes to --out (default: the scratchpad folder occ/): the two full frames with the ROI drawn, then the
prepared empty ROI, the prepared full ROI and the |difference| heat map, so the ROI can be judged by eye.

    python -X utf8 tools/occupancy_eval.py                  the tables once
    python -X utf8 tools/occupancy_eval.py --deep            with the level-2 columns (needs onnxruntime and --model)
    python -X utf8 tools/occupancy_eval.py --watch           re-run every 60 s, print only when the pictures or the table changed
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
from perception import occupancy as O  # noqa: E402

OUT = os.path.join(os.environ.get("TEMP", HERE), "claude", "C--Users-ASUS-Downloads-bioyond-arm-setup",
                   "087b0f72-90ad-4c92-a3ac-198cc1178181", "scratchpad", "occ")


def stations():
    for d in sorted(glob.glob(os.path.join(O.REFS, "*"))):
        name = os.path.basename(d)
        cam = O.camera_for(name)
        e, f = O.pictures(name, cam, "empty"), O.pictures(name, cam, "full")
        if e and f:
            yield name, cam, e, f


def evaluate(name, cam, empties, fulls, deep=False):
    """Scores and verdicts of one station: dict with the rows of the table, level 1 and (optionally) level 2."""
    st = O.Station.load(name, cam)
    r = {"station": name, "camera": cam, "n_empty": len(empties), "n_full": len(fulls), "noise": st.noise, "calibrated": st.calibrated}
    r["empty"] = list(st.loo)
    r["full"] = [st.distance(O.prepare(cv2.imread(p), st.roi))[0] for p in fulls]
    # leave-one-out verdicts of the empties: the station calibrated without that picture judges it
    r["v_empty"] = []
    for i, p in enumerate(empties):
        if len(empties) < 2:
            r["v_empty"].append("-")
            continue
        sub = O.Station.load(name, cam, paths=empties[:i] + empties[i + 1:])
        r["v_empty"].append(sub.verdict(cv2.imread(p)).state[0])
    r["v_full"] = [st.verdict(cv2.imread(p)).state[0] for p in fulls]
    if deep:
        r["d_empty"] = [O.Station.load(name, cam, paths=empties[:i] + empties[i + 1:]).deep_score(cv2.imread(p))
                        for i, p in enumerate(empties)] if len(empties) > 1 else []
        r["d_full"] = [st.deep_score(cv2.imread(p)) for p in fulls]
    picture(name, st, empties[0], fulls[-1], r["full"][-1])
    return r


def margin(empty, full, noise, calibrated):
    """(absolute margin, margin in noise units, unit label); None when not measurable."""
    if not full:
        return None
    lo = min(full)
    hi = max(empty) if empty else (noise if calibrated else None)
    if hi is None:
        return (None, lo / noise, "xN*")
    m = lo - hi
    if len(empty) >= 3 and np.std(empty) > 1e-9:
        return (m, m / float(np.std(empty)), "sd")
    return (m, m / hi, "xN")


def rng(v):
    v = [x for x in v if x is not None]
    return "n/a" if not v else f"{min(v):.2f}" if len(v) == 1 else f"{min(v):.2f}..{max(v):.2f}"


def table(rows, key_e, key_f, title):
    hdr = f"{'station':<11}{'cam':<6}{'nE':>3}{'nF':>3}  {'noise':<7}{'empty-vs-empty':<16}{'full-vs-empty':<16}{'margin':<20}verdicts (empties | fulls)"
    out = [title, hdr, "-" * len(hdr)]
    for r in rows:
        e, f = r[key_e], r[key_f]
        if key_e == "d_empty":                                    # level 2 has no calibrated noise: show the ratio only
            mg = f"{min(f) / max(e):.1f}x" if e and f and max(e) > 0 else f"{min(f):.2f} (1 empty)" if f else "n/a"
            noise = "-"
        else:
            m = margin(e, f, r["noise"], r["calibrated"])
            mg = "n/a" if m is None else (f"{m[1]:.1f} {m[2]}" if m[0] is None else f"{m[0]:+.2f} ({m[1]:.1f} {m[2]})")
            noise = f"{r['noise']:.2f}{'' if r['calibrated'] else '*'}"
        verd = "".join(r["v_empty"]) + " | " + "".join(r["v_full"]) if key_e == "empty" else "-"
        out.append(f"{r['station']:<11}{r['camera']:<6}{r['n_empty']:>3}{r['n_full']:>3}  {noise:<7}{rng(e):<16}{rng(f):<16}{mg:<20}{verd}")
    return "\n".join(out)


def picture(name, st, empty_path, full_path, s):
    """Side by side: frames with the ROI, prepared empty, prepared full, |difference| heat map."""
    E, F = cv2.imread(empty_path), cv2.imread(full_path)
    h, w = E.shape[:2]
    x0, y0, x1, y1 = int(st.roi[0] * w), int(st.roi[1] * h), int(st.roi[2] * w), int(st.roi[3] * h)
    frames = []
    for im, label in ((E, "empty " + os.path.basename(empty_path)), (F, "full " + os.path.basename(full_path))):
        im = im.copy()
        cv2.rectangle(im, (x0, y0), (x1, y1), (0, 255, 0), 4)
        cv2.putText(im, label, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
        frames.append(cv2.resize(im, (480, int(480 * h / w))))
    pe, pf = O.prepare(E, st.roi), O.prepare(F, st.roi)
    diff = cv2.absdiff(pe, pf)
    heat = cv2.applyColorMap(cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX), cv2.COLORMAP_JET)
    panels = [cv2.cvtColor(pe, cv2.COLOR_GRAY2BGR), cv2.cvtColor(pf, cv2.COLOR_GRAY2BGR), heat]
    panels = [cv2.resize(p, (320, frames[0].shape[0])) for p in panels]
    cv2.putText(panels[2], f"score {s:.2f} noise {st.noise:.2f}{'' if st.calibrated else '*'}", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    top = cv2.hconcat(frames)
    bottom = cv2.hconcat(panels)
    bottom = cv2.copyMakeBorder(bottom, 0, 0, 0, max(0, top.shape[1] - bottom.shape[1]), cv2.BORDER_CONSTANT)
    top = cv2.copyMakeBorder(top, 0, 0, 0, max(0, bottom.shape[1] - top.shape[1]), cv2.BORDER_CONSTANT)
    os.makedirs(OUT, exist_ok=True)
    cv2.imwrite(os.path.join(OUT, f"{name}_{st.camera}.png"), cv2.vconcat([top, bottom]))


def run(deep):
    rows = [evaluate(*s, deep=deep) for s in stations()]
    if not rows:
        return "no station has both an empty and a full picture yet under " + O.REFS
    txt = table(rows, "empty", "full", f"level 1  (noise * = DEFAULT_NOISE, one empty only; bands empty <= {O.LOW} N, full >= {O.HIGH} N)")
    if deep:
        if rows[0].get("d_full") and rows[0]["d_full"][0] is None:
            txt += "\n\nlevel 2: not available (onnxruntime or the model file " + O.MODEL + " is missing)"
        else:
            txt += "\n\n" + table(rows, "d_empty", "d_full", "level 2  PatchCore ResNet18 stage 3 (95th percentile nearest-patch distance; margin = min full / max empty)")
    return txt + f"\n\npictures: {OUT}"


def signature():
    return tuple((p, os.path.getsize(p)) for p in sorted(glob.glob(os.path.join(O.REFS, "*", "*.jpg"))))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--deep", action="store_true", help="also the level-2 PatchCore score")
    ap.add_argument("--model", help="ONNX model file for --deep (default: $OCC_MODEL or models/resnet18-v1-7.onnx)")
    ap.add_argument("--out", help="folder for the PNGs")
    ap.add_argument("--watch", action="store_true", help="re-run every 60 s while pictures arrive; print only on change")
    a = ap.parse_args()
    global OUT
    if a.model:
        O.MODEL = a.model
    if a.out:
        OUT = a.out
    last = (None, None)
    while True:
        sig = signature()
        if sig != last[0]:
            txt = run(a.deep)
            if txt != last[1]:
                print(time.strftime("%H:%M:%S"), f"{len(sig)} pictures\n{txt}\n", flush=True)
            last = (sig, txt)
        if not a.watch:
            break
        time.sleep(60)


if __name__ == "__main__":
    main()
