"""The setup's arithmetic. Pure functions on poses (x y z mm, rx ry rz deg), no device."""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple


def _rot(deg: float, x: float, y: float) -> Tuple[float, float]:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return c * x - s * y, s * x + c * y


def wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def grip_in_marker_frame(marker: Sequence[float], grip: Sequence[float]) -> dict:
    """Where the grip pose sits relative to the marker, in the marker's own frame: it stays true when the plate
    is carried to another place. dyaw is the grip yaw minus the marker yaw."""
    dx, dy = _rot(-marker[5], grip[0] - marker[0], grip[1] - marker[1])
    return {"dx": round(dx, 3), "dy": round(dy, 3), "dz": round(grip[2] - marker[2], 3), "dyaw": round(wrap(grip[5] - marker[5]), 3)}


def seat_from_marker(marker: Sequence[float], rel: dict, near_yaw: float, template: Sequence[float]) -> List[float]:
    """The grip pose for a plate whose marker reads `marker` here. The jaw is symmetric, so the yaw is the one of the
    two (yaw, yaw + 180) closest to `near_yaw`, the yaw the fixture had before. rx ry come from `template`."""
    dx, dy = _rot(marker[5], rel["dx"], rel["dy"])
    yaw = wrap(marker[5] + rel["dyaw"])
    if abs(wrap(yaw - near_yaw)) > 90.0:
        yaw = wrap(yaw + 180.0)
    return [round(marker[0] + dx, 3), round(marker[1] + dy, 3), round(marker[2] + rel["dz"], 3), template[3], template[4], round(yaw, 4)]


def moved_mm(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def accept_band(closed: float, below: float, above: float) -> List[float]:
    """The jaw reading a good grip may give: the measured closed width, a little under, a little over."""
    return [round(closed - below, 2), round(closed + above, 2)]


def spread(values: Sequence[float]) -> float:
    return round(max(values) - min(values), 3) if values else 0.0
