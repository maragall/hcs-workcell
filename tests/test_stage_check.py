"""The hotel lift against the headroom, checked by the preflight."""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from rig import layout  # noqa: E402
from procedures.preflight import preflight  # noqa: E402


def test_preflight_refuses_a_hotel_lift_at_or_above_the_headroom():
    L = layout()
    L["fixtures"]["hotel"]["headroom_mm"] = 17
    L["fixtures"]["hotel"]["lift_mm"] = 25.0
    assert any("headroom" in p for p in preflight(L, "/nonexistent"))
    L["fixtures"]["hotel"]["lift_mm"] = 8.0
    assert not any("headroom" in p for p in preflight(L, "/nonexistent"))
