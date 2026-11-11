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


# ---- the module seat's angle by force, against what the arm read on 2026-11-10/11 (runs/setup_console_*.log of those nights)
class _Module:
    """A stand-in arm whose module answers a close as it did that night: (degrees pushed off its mark, motor torque) for a wrist angle."""

    def __init__(self, answer):
        self.answer, self.yaw, self.closes, self.mark = answer, None, 0, 0.0

    def set_speed(self, v): pass
    def jaw_to(self, w): pass
    def ext_axis_to(self, i, m): self.mark = m
    def move_linear(self, p): self.yaw = p[5]
    def ext_axis(self, i): return self.mark + self.answer(self.yaw)[0]

    def clamp(self, *a):
        self.closes += 1
        return {"width": 1.5}


def _long_0105(y):       # pushed 2.54 deg at -86.84, 1.36 at -87.84, 0.41 at -88.84, more the other way: the old search gave up after 17 closes
    off = max(0.0, (y + 89.22) * 1.07)
    return off, (-1.0 if off > 0.05 else -0.2)


def _short_0101(y):      # twisted from -88.86 up (0.11, 0.34, 0.55, 0.77, 1.01), left alone from -89.11 down
    return max(0.0, (y + 88.97) * 0.9), (-1.0 if y > -89.0 else -0.2)


def _short_2316(y):      # one-sided: full torque from -89.06 up with the module barely pushed, 0.70 at -89.31, left alone below, down to -92
    return (0.08 if y > -89.2 else 0.0), (-1.0 if y > -89.2 else (-0.7 if y > -89.45 else -0.02))


def _window(y):          # two-sided, 0.4 deg wide at -89.4
    off = max(0.0, abs(y + 89.4) - 0.2)
    return off, (-1.0 if off > 0.05 else 0.1)


@pytest.mark.parametrize("answer, grip, hand, lo, hi", [
    (_long_0105, "long_edges", -86.84, -90.0, -89.22),
    (_short_0101, "short_edges", -88.36, -90.0, -89.0),
    (_short_2316, "short_edges", -89.06, -90.2, -89.45),
    (_window, "long_edges", -86.4, -89.6, -89.2),
    (_window, "long_edges", -91.4, -89.6, -89.2),
    (_window, "long_edges", -89.4, -89.4, -89.4),
])
def test_the_seat_angle_is_found_in_a_few_closes_from_how_far_the_module_was_pushed(answer, grip, hand, lo, hi):
    L = positions.load()
    L["fixtures"]["polar"]["pose_at_90" if grip == "long_edges" else "pose_at_0"]["pose"][5] = hand
    arm = _Module(answer)
    got = places.refine_yaw(arm, L, grip, lambda ip, seconds: {"module": [arm.answer(arm.yaw)[1]]})
    assert lo - 1e-6 <= got["pose"][5] <= hi + 1e-6 and arm.closes <= 5


def test_a_plate_that_twists_the_module_at_every_angle_is_refused_with_what_to_do():
    L = positions.load()
    arm = _Module(lambda y: (0.5, -1.0))
    with pytest.raises(places.PlacesError, match="fit the gripper again"):
        places.refine_yaw(arm, L, "long_edges", lambda ip, seconds: {"module": [-1.0]})
    assert arm.closes <= places.MAX_CLOSES
