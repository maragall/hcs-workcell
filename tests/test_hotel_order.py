"""The hotel plate order, and the swap: the plate on the microscope goes home, the next one comes in."""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from test_sim import Rig  # noqa: E402

from procedures.hotel import interleave, plate_order  # noqa: E402


def test_interleave_alternates_columns_and_tolerates_uneven_lengths():
    assert interleave(["A1", "A3", "A5"], ["C2", "C4"]) == ["A1", "C2", "A3", "C4", "A5"]
    assert interleave(["A1"], ["C2"], ["B5", "B7"]) == ["A1", "C2", "B5", "B7"]
    assert interleave() == []


def test_plate_order_reads_the_loaded_slots_from_the_positions_file():
    L = {"fixtures": {"hotel": {"loaded": {"A": [1, 3, 5, 7, 9], "C": [2, 4, 6, 8, 10]}}}}
    assert plate_order(L) == ["A1", "C2", "A3", "C4", "A5", "C6", "A7", "C8", "A9", "C10"]


def test_second_change_returns_the_plate_home_and_fetches_the_next_slot():
    r = Rig()
    try:
        r.run("A1")                                    # plate on the microscope has no home: it goes to A1 and comes back
        s = r.cyc.store.s
        assert s.plate_at == "microscope" and s.home_slot == "A1"
        r.run("C2")                                    # A1 home, then C2 in
        assert s.plate_at == "microscope" and s.home_slot == "C2"
        assert "A1" in s.hotel and "C2" not in s.hotel  # the ledger: A1 holds a plate again, C2 is now empty
        steps = [e["step"] for e in r.events() if e["kind"] == "move"]
        assert any("place:hotel:A1" in st for st in steps) and any("pick:hotel:C2" in st for st in steps)
    finally:
        r.close()


def test_a_place_into_an_occupied_slot_backs_out_and_uses_the_home_slot():
    r = Rig()
    try:
        r.run("A1")                                    # plate home is A1
        s = r.cyc.store.s
        s.hotel["A1"] = "a plate someone left there"   # someone filled the home slot meanwhile (ledger, or the camera on the real cell)
        r.cyc.store.save()
        r.run("C2")                                    # going home to A1 is refused: backs out, places in C2, then brings C2's plate in
        assert s.plate_at == "microscope" and s.home_slot == "C2"
        assert s.hotel["A1"] == "a plate someone left there" and "C2" not in s.hotel
        events = r.events()
        assert any(e["step"] == "slot_occupied" for e in events)
        assert any("place:hotel:C2" in e["step"] for e in events)
    finally:
        r.close()
