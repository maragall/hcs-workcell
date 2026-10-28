"""Is there a slot at this row? The camera looks at the slot position of one row; the answer is learned from the user's
yes or no, one picture at a time, and once the cell has seen the row it answers by itself. A 30-slot hotel has a shelf at
every row, a 15-slot one only at the odd rows, so two even rows tell them apart. Normalised correlation against the pictures
kept per row (reference_images/hotel_rows/row<n>/{occupied,empty}), the check the slot occupancy uses (perception/occupancy.py);
`occupied` here means a slot is there, `empty` that it is not. With pictures of one answer only, a picture that matches
them says that answer; with both, the better match wins."""
from __future__ import annotations

from typing import Optional

from perception.lower_check import _prep, ncc
from perception.occupancy import Occupancy

MIN_MATCH = 0.75


class HotelRows:
    def __init__(self, ref_root: str):
        self._occ = Occupancy(ref_root, min_refs=1)

    @staticmethod
    def _slot(row: int) -> str:
        return f"row{row}"

    def verdict(self, row: int, img: bytes) -> Optional[bool]:
        """True: a slot here. False: none. None: this row has not been seen like this before."""
        cur = _prep(img)
        yes = [ncc(cur, r) for r in self._occ._refs(self._slot(row), "occupied")]
        no = [ncc(cur, r) for r in self._occ._refs(self._slot(row), "empty")]
        best_yes, best_no = max(yes, default=-1.0), max(no, default=-1.0)
        if max(best_yes, best_no) < MIN_MATCH:
            return None
        return best_yes > best_no

    def learn(self, row: int, img: bytes, present: bool) -> None:
        self._occ.add_reference(self._slot(row), "occupied" if present else "empty", img)
