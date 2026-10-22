"""
Plate types: the grab parameters that change with the plate, not with the fixture.

layout.json `plates.types.<name>`:
    id              integer sent to the controller as SYSINVAR8 (lua/data_plate.lua holds the same numbers)
    length_mm, width_mm, height_mm   plate size (height may be null until measured)
    grab_height_mm  added to every seat pose Z before a pick or a place (the vendor template's VAR7)
    jaw.open / jaw.closed            optional: E1 jaw position per grip ("short_edges", "long_edges"), pendant
                                     readings. Without it the type uses the cell's `gripper.open/closed`.

`plates.default` names the type used when nothing selects one. A layout without a `plates` section
yields one type, "default", on `gripper.open/closed`. `gripper.*` stays the one place the teach and
measure commands write to; a plate type only overrides it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

LUA_PLATE_INDEX = 8          # SYSINVAR index that carries the plate type id


@dataclass(frozen=True)
class PlateType:
    name: str
    id: int
    jaw_open: Dict[str, float]
    jaw_closed: Dict[str, float]
    grab_height_mm: float = 0.0
    length_mm: Optional[float] = None
    width_mm: Optional[float] = None
    height_mm: Optional[float] = None

    def lua_vars(self) -> Dict[int, int]:
        """The system variable that tells lua/mod_Cell which plate it is handling."""
        return {LUA_PLATE_INDEX: self.id}


def load(layout: dict) -> Dict[str, PlateType]:
    g = layout["gripper"]
    section = layout.get("plates")
    if not section:
        return {"default": PlateType("default", 1, dict(g["open"]), dict(g["closed"]))}
    out = {}
    for name, t in section["types"].items():
        if name.startswith("_"):
            continue
        jaw = t.get("jaw") or {}
        out[name] = PlateType(name, int(t["id"]), dict(jaw.get("open") or g["open"]), dict(jaw.get("closed") or g["closed"]),
                              float(t.get("grab_height_mm", 0.0)), t.get("length_mm"), t.get("width_mm"),
                              t.get("height_mm"))
    ids = [p.id for p in out.values()]
    if len(ids) != len(set(ids)):
        raise ValueError(f"plates: duplicate id in {ids}")
    return out


def default(layout: dict) -> PlateType:
    types = load(layout)
    name = layout.get("plates", {}).get("default", "default")
    if name not in types:
        raise KeyError(f"plates.default '{name}' is not one of {sorted(types)}")
    return types[name]
