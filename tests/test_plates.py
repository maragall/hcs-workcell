import json
import os

import pytest

from procedures import plates

HERE = os.path.dirname(os.path.abspath(__file__))


def layout():
    with open(os.path.join(os.path.dirname(HERE), "layout.json"), "r", encoding="utf-8") as f:
        return json.load(f)


def test_production_plate_is_the_default_and_matches_the_global_gripper_widths():
    L = layout()
    p = plates.default(L)
    assert p.name == "production" and p.id == 1
    assert p.jaw_open == L["gripper"]["open"]
    assert p.jaw_closed == L["gripper"]["closed"]
    assert p.grab_height_mm == 0.0


def test_lua_vars_carry_the_type_id_on_index_8():
    assert plates.default(layout()).lua_vars() == {8: 1}


GRIPPER = {"open": {"short_edges": 26.0, "long_edges": 6.0}, "closed": {"short_edges": 22.4, "long_edges": 1.45}}


def test_layout_without_plates_section_falls_back_to_the_global_gripper():
    p = plates.default({"gripper": GRIPPER})
    assert p.name == "default" and p.id == 1 and p.jaw_closed["long_edges"] == 1.45


def test_a_type_with_its_own_jaw_block_overrides_the_gripper_widths():
    L = {"gripper": GRIPPER, "plates": {"default": "thin", "types": {
        "thin": {"id": 2, "jaw": {"open": {"short_edges": 20.0, "long_edges": 4.0},
                                  "closed": {"short_edges": 18.0, "long_edges": 0.5}}}}}}
    p = plates.default(L)
    assert p.jaw_open["long_edges"] == 4.0 and p.jaw_closed["short_edges"] == 18.0


def test_duplicate_ids_are_refused():
    L = {"gripper": GRIPPER, "plates": {"default": "a", "types": {"a": {"id": 1}, "b": {"id": 1}}}}
    with pytest.raises(ValueError):
        plates.load(L)
