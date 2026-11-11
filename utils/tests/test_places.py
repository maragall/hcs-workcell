"""The six-slot fit of the hotel (setup/places.py), on this workcell's own taught records. No arm."""
from __future__ import annotations

import copy
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from run import positions  # noqa: E402
from setup import places  # noqa: E402


def test_six_slots_put_every_seat_within_a_third_of_a_millimetre_of_the_seats_in_use():
    L = positions.load()
    old = copy.deepcopy(L["fixtures"]["hotel"]["taught"])
    got = places.fit_hotel(L)
    new = L["fixtures"]["hotel"]["taught"]
    assert max(abs(new[s][i] - old[s][i]) for s in old for i in range(3)) < 0.35
    assert abs(got["turn_deg"]) < places.MAX_TURN_DEG and all(new[s][5] == 0.0 for s in new)


def test_a_slot_taught_in_the_wrong_place_is_named():
    L = positions.load()
    L["fixtures"]["hotel"]["taught_detail"]["B9"]["pose"][1] += 6.0
    with pytest.raises(places.PlacesError, match="B9"):
        places.fit_hotel(L)


def test_a_hotel_turned_against_the_arm_is_refused():
    L = positions.load()
    for s in places.SLOTS:                                   # the whole hotel turned one degree about B1
        p = L["fixtures"]["hotel"]["taught_detail"][s]["pose"]
        p[0] -= 0.01745 * (p[1] - 0.0)
    with pytest.raises(places.PlacesError, match="turned"):
        places.fit_hotel(L)


def test_a_missing_slot_is_asked_for():
    L = positions.load()
    L["fixtures"]["hotel"]["taught_detail"].pop("C1")
    with pytest.raises(places.PlacesError, match="C1"):
        places.fit_hotel(L)
