"""
The station graph: the legs the cycle drives are planned once, checked against the hotel, module and pillar, stored, and replayed.

A station is a named pose the cell goes to over and over: the nest, the hovers of the microscope and the module, and for every hotel
slot its approach point and its mouth (where the claw, with what it holds, is just outside the rack). `plan()` asks the graph first:
when the arm is at one station and the target is another, the stored leg for (from, to, what the claw holds, elbow branch) is
returned without any search. A leg that is not stored is planned as before (so an arm that stopped anywhere still gets a route to
the nearest station) and, when both ends are stations, stored for next time. A stored leg is valid only for the geometry it was
planned in: the key carries a hash of every volume, region and pose of the layout, so re-teaching a seat or editing a volume
discards the old legs by itself.

    python tools/build_stations.py        plans every leg the shuffle uses, offline, and says which have no route
"""
from __future__ import annotations

import json
import os
import threading
from typing import Callable, Dict, List, Optional, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, "station_graph.json")

TOL_XY_MM = 6.0
TOL_Z_MM = 4.0


class StationGraph:
    def __init__(self, provider: Callable[[Optional[str]], Dict[str, List[float]]], path: str = PATH):
        """provider(holding) -> {station name: pose}: the stations as they are for a claw holding `holding` (the mouth of a slot
        depends on what the claw carries)."""
        self.provider, self.path = provider, path
        self._lock = threading.Lock()
        self._stations: Dict[Tuple, Dict[str, List[float]]] = {}
        self.legs: Dict[str, dict] = {}
        self.hits = self.misses = 0
        try:
            with open(path, encoding="utf-8") as f:
                self.legs = json.load(f).get("legs", {})
        except (OSError, ValueError):
            self.legs = {}

    def stations(self, sig: str, holding: Optional[str]) -> Dict[str, List[float]]:
        k = (sig, holding)
        if k not in self._stations:
            try:
                self._stations[k] = self.provider(holding)
            except Exception:
                self._stations[k] = {}
        return self._stations[k]

    def name_of(self, sig: str, holding: Optional[str], pose: List[float]) -> Optional[str]:
        best, best_d = None, 1e9
        for name, p in self.stations(sig, holding).items():
            dxy = max(abs(pose[0] - p[0]), abs(pose[1] - p[1]))
            if dxy <= TOL_XY_MM and abs(pose[2] - p[2]) <= TOL_Z_MM and dxy + abs(pose[2] - p[2]) < best_d:
                best, best_d = name, dxy + abs(pose[2] - p[2])
        return best

    @staticmethod
    def key(sig: str, a: str, b: str, holding: Optional[str], branch: int) -> str:
        return f"{sig[:12]}|{a}|{b}|{holding or 'none'}|{branch}"

    def get(self, key: str):
        leg = self.legs.get(key)
        if leg is None:
            self.misses += 1
            return None
        self.hits += 1
        return leg

    def put(self, key: str, poses: List[List[float]], joint_targets: Dict[int, List[float]], name: str) -> None:
        with self._lock:
            self.legs[key] = {"name": name, "poses": [[round(v, 3) for v in p] for p in poses],
                              "joint_targets": {str(i): [round(v, 4) for v in j] for i, j in joint_targets.items()}}
            self.save()

    def save(self) -> None:
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"_note": "legs between stations (routes/graph.py), planned and checked once; the key starts with the layout's geometry hash",
                       "legs": self.legs}, f, indent=1)
        os.replace(tmp, self.path)
