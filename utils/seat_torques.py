"""Torque of every motor with the arm standing still, read through the torque probe: the contact force at a seat, axis by axis.
    from utils.seat_torques import read          read(ip, seconds) -> {"lift", "shoulder", "elbow", "wrist", "jaw", "module"} mean and spread"""
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.devices import clamp  # noqa: E402

NAMES = {9: "lift", 10: "shoulder", 11: "elbow", 12: "wrist", 6: "jaw", 13: "module"}


def read(ip: str, seconds: float = 2.0) -> dict:
    """Starts the probe, samples, stops it. The arm must stand still: while the probe runs it ignores move commands."""
    def work(b):
        rows, t0 = [], time.time()
        while time.time() - t0 < seconds:
            v = b.sysvars(range(4, 15))
            rows.append({n: v.get(i, 0) / 1000.0 for i, n in NAMES.items()})
            time.sleep(0.05)
        return {n: (round(statistics.mean(r[n] for r in rows), 3), round(statistics.pstdev(r[n] for r in rows), 3)) for n in NAMES.values()}
    return clamp._session(ip, clamp.PORT, work)
