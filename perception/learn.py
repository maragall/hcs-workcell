"""
Learned classifiers from the frames the cell takes itself. The labels are free: the ledger knows what the arm is doing, so a
front-camera frame at a pick approach shows a plate, one at a place approach shows an empty slot, and so on.

Each task is two sets of saved gate frames (telemetry images, `<counter>_<step>_<name>.jpg`) and a camera region. A task is
trained on the earlier part of the frames and judged on the later part (time split, never shuffled: the lighting of the
test frames was not in training), with the existing one-class correlation gate scored on the same test frames for comparison.

    python -m perception.learn            # every task on every run in runs/ that has images; writes models/
"""
from __future__ import annotations

import fnmatch
import json
import os
import re
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from perception.lower_check import PATCH, _prep, ncc

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNS, MODELS = os.path.join(HERE, "runs"), os.path.join(HERE, "models")
SEATED_BAND = (0.0, 0.55, 1.0, 1.0)          # vision.seated_front.roi: the bottom band, where the plate's edge and the holder sit
SMALL = (40, 30)

# name: (frame pattern for class 1, frame pattern for class 0, region, what class 1 means)
TASKS: Dict[str, Tuple[str, str, Optional[tuple], str]] = {
    "slot_probe":        ("*_probe_hand-*_slot-occupied_*", "*_probe_hand-*_slot-empty_*", None, "a plate is in the hotel slot, hand empty or holding a plate"),
    "slot_occupancy":    ("*_pick_hotel_*_slot_view", "*_place_hotel_*_slot_view", None, "a plate is in the hotel slot"),
    "stage_down":        ("*_pick_microscope_lower_0", "*_place_microscope_lower_0", None, "a plate is on the microscope stage (down camera)"),
    "stage_front":       ("*_pick_microscope_microscope_front", "*_place_microscope_microscope_front", None, "a plate is on the microscope stage (front camera)"),
    "seated_microscope": ("*_place_microscope_seated_front", "*_place_microscope_microscope_front", SEATED_BAND, "the plate sits in the microscope holder"),
    "seated_module":     ("*_place_polar_seated_front", "*_place_polar_polar_*_front", SEATED_BAND, "the plate sits on the rotation module"),
    "seated_hotel":      ("*_place_hotel_*_seated_front", "*_place_hotel_*_slot_view", SEATED_BAND, "the plate sits in the hotel slot"),
}
# A pick approach always has an empty hand and a plate in the slot; a place approach a plate in the hand and an empty slot (the stage and
# the seat likewise). A classifier trained on them may be reading the hand, and the test frames share that fault, so a perfect score proves
# nothing. Only the probe tours (hand empty / plate in hand, at slots with and without a plate) cross the two.
CONFOUNDED = {"slot_occupancy", "stage_down", "stage_front", "seated_microscope", "seated_module", "seated_hotel"}
# Criteria for a task to count as solved (the loop stops when all hold for two rounds in a row)
MIN_TEST_PER_CLASS = 60
MIN_ACCURACY = 0.995


def features(jpeg: bytes, roi=None) -> np.ndarray:
    return cv2.resize(_prep(jpeg, roi), SMALL, interpolation=cv2.INTER_AREA).ravel().astype(np.float32)


def frames(run_dirs: List[str], pattern: str) -> List[str]:
    out = []
    for d in run_dirs:
        root = os.path.join(d, "images")
        if not os.path.isdir(root):
            continue
        for cyc in sorted(os.listdir(root)):
            p = os.path.join(root, cyc)
            out += [os.path.join(p, n) for n in sorted(os.listdir(p)) if fnmatch.fnmatch(n[:-4], pattern)]
    return out


def run_dirs(since: str = "") -> List[str]:
    return [os.path.join(RUNS, n) for n in sorted(os.listdir(RUNS)) if n.startswith("setup_20") and n >= "setup_" + since
            and os.path.isdir(os.path.join(RUNS, n, "images"))]


@dataclass
class Result:
    task: str
    n_train: int
    n_test: Tuple[int, int]
    accuracy: float
    false_1: int                     # class 0 called class 1
    false_0: int                     # class 1 called class 0
    baseline_accuracy: Optional[float]
    solved: bool
    note: str = ""
    by_hand: Optional[Dict[str, float]] = None


def _split(paths: List[str], frac: float = 0.7) -> Tuple[List[str], List[str]]:
    k = int(len(paths) * frac)
    return paths[:k], paths[k:]


def _load(paths: List[str], roi) -> np.ndarray:
    out = []
    for p in paths:
        with open(p, "rb") as f:
            out.append(features(f.read(), roi))
    return np.array(out, dtype=np.float32) if out else np.zeros((0, SMALL[0] * SMALL[1]), np.float32)


def _baseline(train1: np.ndarray, test1: np.ndarray, test0: np.ndarray) -> float:
    """The shipped one-class gate: class 1 if the best correlation with 12 class-1 references reaches the pairwise-threshold rule."""
    refs = train1[:12]
    if len(refs) < 3:
        return float("nan")
    thr = max(0.55, min(ncc(a, b) for i, a in enumerate(refs) for b in refs[i + 1:]) - 0.08)
    best = lambda x: max(ncc(x, r) for r in refs)
    ok = sum(best(x) >= thr for x in test1) + sum(best(x) < thr for x in test0)
    return ok / max(1, len(test1) + len(test0))


def train_task(name: str, runs: List[str], save: bool = True) -> Result:
    p1, p0, roi, meaning = TASKS[name]
    a1, a0 = frames(runs, p1), frames(runs, p0)
    t1, s1 = _split(a1)
    t0, s0 = _split(a0)
    if min(len(t1), len(t0)) < 20 or min(len(s1), len(s0)) < 5:
        return Result(name, len(t1) + len(t0), (len(s1), len(s0)), float("nan"), 0, 0, None, False, "too few frames")
    X1, X0, T1, T0 = _load(t1, roi), _load(t0, roi), _load(s1, roi), _load(s0, roi)
    X = np.vstack([X1, X0])
    y = np.array([1] * len(X1) + [0] * len(X0), dtype=np.int32)
    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_RBF)
    svm.setC(10.0)
    svm.setGamma(1.0 / X.shape[1])
    svm.train(X, cv2.ml.ROW_SAMPLE, y)
    pred = lambda M: svm.predict(M)[1].ravel().astype(int) if len(M) else np.zeros(0, int)
    p_t1, p_t0 = pred(T1), pred(T0)
    false_0, false_1 = int((p_t1 == 0).sum()), int((p_t0 == 1).sum())
    acc = 1.0 - (false_0 + false_1) / (len(T1) + len(T0))
    solved = min(len(T1), len(T0)) >= MIN_TEST_PER_CLASS and acc >= MIN_ACCURACY and false_0 == 0 and false_1 == 0
    by_hand = None
    if any("hand-" in p for p in s1 + s0):          # the crossed frames: each hand state has to be right on its own
        by_hand = {}
        for hand in ("empty", "plate"):
            hits = [(p_t1[i] == 1) for i, p in enumerate(s1) if f"hand-{hand}" in p] + [(p_t0[i] == 0) for i, p in enumerate(s0) if f"hand-{hand}" in p]
            by_hand[hand] = sum(hits) / len(hits) if len(hits) >= 20 else float("nan")
        solved = solved and all(v == v and v >= MIN_ACCURACY for v in by_hand.values())
    if name in CONFOUNDED:
        solved = False
    if save:
        os.makedirs(MODELS, exist_ok=True)
        svm.save(os.path.join(MODELS, f"{name}.xml"))
    return Result(name, len(X), (len(T1), len(T0)), acc, false_1, false_0, _baseline(X1, T1, T0), solved,
                  "confounded with the hand state: not evidence" if name in CONFOUNDED else
                  "" if min(len(T1), len(T0)) >= MIN_TEST_PER_CLASS else f"needs {MIN_TEST_PER_CLASS} test frames per class", by_hand)


def run(since: str = "", save: bool = True) -> List[Result]:
    runs = run_dirs(since)
    results = [train_task(n, runs, save) for n in TASKS]
    if save:
        with open(os.path.join(MODELS, "history.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "runs": len(runs),
                                "results": [r.__dict__ for r in results]}) + "\n")
    return results


def report(results: List[Result]) -> str:
    rows = [f"{'task':20s} {'train':>6s} {'test 1/0':>10s} {'accuracy':>9s} {'false 1':>7s} {'false 0':>7s} {'old gate':>9s}  solved"]
    for r in results:
        base = "" if r.baseline_accuracy is None or r.baseline_accuracy != r.baseline_accuracy else f"{r.baseline_accuracy:.3f}"
        acc = "" if r.accuracy != r.accuracy else f"{r.accuracy:.4f}"
        hands = "" if not r.by_hand else "  by hand: " + ", ".join(f"{k} {v:.3f}" for k, v in r.by_hand.items())
        rows.append(f"{r.task:20s} {r.n_train:6d} {r.n_test[0]:4d}/{r.n_test[1]:<5d} {acc:>9s} {r.false_1:7d} {r.false_0:7d} {base:>9s}  "
                    f"{'YES' if r.solved else 'no'} {r.note}{hands}")
    return "\n".join(rows)


if __name__ == "__main__":
    print(report(run()))
