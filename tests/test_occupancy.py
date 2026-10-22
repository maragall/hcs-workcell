"""The slot occupancy detector on synthetic frames: empty and occupied look different, noise does not."""
from __future__ import annotations

import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from rig import BAD, SAFE, jpeg  # noqa: E402

from perception.occupancy import Occupancy  # noqa: E402

EMPTY, OCCUPIED = SAFE, BAD        # two different synthetic scenes


def detector(n_empty=5, n_occupied=5, min_refs=5):
    tmp = tempfile.mkdtemp()
    occ = Occupancy(tmp, min_refs=min_refs)
    for _ in range(n_empty):
        occ.add_reference("A1", "empty", jpeg(EMPTY))
    for _ in range(n_occupied):
        occ.add_reference("A1", "occupied", jpeg(OCCUPIED))
    return occ, tmp


def test_unknown_until_both_classes_have_enough_references():
    occ, tmp = detector(n_occupied=2)
    try:
        v = occ.verdict("A1", jpeg(EMPTY))
        assert v.decision == "unknown" and not occ.ready("A1") and "need 5" in v.reason
    finally:
        shutil.rmtree(tmp)


def test_decides_each_class_with_a_margin_and_is_ready():
    occ, tmp = detector()
    try:
        e, o = occ.verdict("A1", jpeg(EMPTY, noise=5.0)), occ.verdict("A1", jpeg(OCCUPIED, noise=5.0))
        assert e.decision == "empty" and o.decision == "occupied"
        assert e.confidence > 0.1 and o.confidence > 0.1
        t = occ.self_test("A1")
        assert t.n == 10 and t.wrong == 0 and t.min_gap > 0
        assert occ.ready("A1")
        assert occ.report()[0]["ready"] is True
    finally:
        shutil.rmtree(tmp)


def test_self_test_runs_once_per_reference_set_and_again_after_a_new_reference():
    occ, tmp = detector()
    try:
        first = occ.self_test("A1")
        assert occ.self_test("A1") is first and occ.ready("A1")
        occ.add_reference("A1", "empty", jpeg(OCCUPIED))           # a wrong frame joins: the verdict must change
        again = occ.self_test("A1")
        assert again is not first and again.n == 11 and again.wrong >= 1 and not occ.ready("A1")
    finally:
        shutil.rmtree(tmp)


def test_a_mislabelled_reference_keeps_the_detector_out_of_enforcement():
    occ, tmp = detector(n_empty=4)
    try:
        occ.add_reference("A1", "empty", jpeg(OCCUPIED))          # an occupied frame filed as empty
        assert occ.count("A1", "empty") == 5
        assert occ.self_test("A1").wrong >= 1 and not occ.ready("A1")
    finally:
        shutil.rmtree(tmp)


def test_identical_pictures_under_both_labels_are_unknown_not_occupied():
    """On the simulator every frame is the same; a tie must never block a placement as 'occupied'."""
    occ, tmp = detector(n_empty=0, n_occupied=0)
    try:
        for state in ("empty", "occupied"):
            for _ in range(5):
                occ.add_reference("A1", state, jpeg(EMPTY))
        v = occ.verdict("A1", jpeg(EMPTY))
        assert v.decision == "unknown" and not occ.ready("A1")
    finally:
        shutil.rmtree(tmp)
