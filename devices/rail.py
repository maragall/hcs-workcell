"""
Rail stub for a future farm: the arm rides a rail between stations, each station a workcell.

Not implemented. It exists so that the place where a rail plugs in is already named:
procedures call `rail.move_to(station)` before the moves at that station, and nothing else changes.
"""
from __future__ import annotations


class Rail:
    def __init__(self, config: dict | None = None):
        self.config = config or {}

    def move_to(self, station: str) -> None:
        raise NotImplementedError("rail: no rail on this cell")
