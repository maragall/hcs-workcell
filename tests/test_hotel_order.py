"""The hotel plate order, and the swap: the plate on the microscope goes home, the next one comes in."""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from test_sim import Rig  # noqa: E402

from run.hotel import interleave, plate_order  # noqa: E402


def test_interleave_alternates_columns_and_tolerates_uneven_lengths():
    assert interleave(["A1", "A3", "A5"], ["C2", "C4"]) == ["A1", "C2", "A3", "C4", "A5"]
    assert interleave(["A1"], ["C2"], ["B5", "B7"]) == ["A1", "C2", "B5", "B7"]
    assert interleave() == []


def test_plate_order_reads_the_loaded_slots_from_the_positions_file():
    L = {"fixtures": {"hotel": {"loaded": {"A": [1, 3, 5, 7, 9], "C": [2, 4, 6, 8, 10]}}}}
    assert plate_order(L) == ["A1", "C2", "A3", "C4", "A5", "C6", "A7", "C8", "A9", "C10"]


