"""What is on the bench, read from and written to the positions file: the hotel size, the slots that hold plates
(fixtures.hotel.loaded, the list the plate order is built from), the microscopes."""
from __future__ import annotations

import re
from typing import Dict, List

from procedures.hotel import plate_order

ROWS_30 = list(range(1, 11))
ROWS_15 = [1, 3, 5, 7, 9]


def microscopes(layout: dict) -> List[str]:
    """Fixture names of the microscopes, microscope first then microscope_2, _3 ... (taught or not)."""
    names = [n for n in layout["fixtures"] if re.fullmatch(r"microscope(_\d+)?", n)]
    return sorted(names, key=lambda n: int(n.split("_")[1]) if "_" in n else 1)


def label(scope: str) -> str:
    return "Microscope" if scope == "microscope" else f"Microscope {scope.split('_')[1]}"


def slots(layout: dict) -> List[str]:
    h = layout["fixtures"]["hotel"]
    return [f"{c}{r}" for r in h["rows"] for c in h["columns"]]


def loaded_slots(layout: dict) -> List[str]:
    return [f"{c}{r}" for c, rows in layout["fixtures"]["hotel"].get("loaded", {}).items() for r in rows]


def set_hotel_size(layout: dict, n: int) -> None:
    """15 = every other row (1, 3, 5, 7, 9), 30 = rows 1 to 10. Plates in rows that vanish are dropped."""
    h = layout["fixtures"]["hotel"]
    h["rows"] = list(ROWS_15 if n == 15 else ROWS_30)
    set_loaded(layout, [s for s in loaded_slots(layout) if int(s[1:]) in h["rows"]])


def set_loaded(layout: dict, which: List[str]) -> None:
    h = layout["fixtures"]["hotel"]
    loaded: Dict[str, List[int]] = {}
    for s in which:
        col, row = s[:1], int(s[1:])
        if col in h["columns"] and row in h["rows"]:
            loaded.setdefault(col, []).append(row)
    h["loaded"] = {c: sorted(rows) for c, rows in sorted(loaded.items())}


def state(layout: dict) -> dict:
    h = layout["fixtures"]["hotel"]
    return {"size": 15 if h["rows"] == ROWS_15 else 30, "rows": h["rows"], "columns": h["columns"],
            "loaded": loaded_slots(layout), "order": plate_order(layout),
            "taught_columns": [c for c in h["columns"] if f"{c}1" in h["taught"]],
            "microscopes": [label(s) for s in microscopes(layout) if layout["fixtures"][s].get("pose")] or ["Microscope"]}
