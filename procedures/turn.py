"""Turning the claw in place: which way round. The controller picks its own direction for a yaw change and on 2026-10-28 it took the
wrist the long way (through 180 deg, the claw toward the pillar: 'axis 4 collision detected'). A turn is a change of joint 4 only (the
wrist stays put), so it is commanded as joint 4 +- a chosen amount, in the direction that keeps joint 4 inside its limits and the claw
clear of the obstacles, or refused before anything moves."""
from __future__ import annotations

from typing import Callable, List, Optional

# Joint 4 limits assumed until read from the controller: the taught routes keep it between -32 and 184 deg. Wider values would let a
# turn go further than anything the cell has ever done.
J4_MIN, J4_MAX = -40.0, 190.0
STEP_DEG = 3.0


def candidates(d_yaw: float) -> List[float]:
    """The ways to turn by d_yaw: the number as given and the same turn the other way round (shortest first)."""
    other = d_yaw - 360.0 if d_yaw > 0 else d_yaw + 360.0
    return sorted({round(d_yaw, 6), round(other, 6)}, key=abs)


def choose(j4_now: float, d_yaw: float, clear_at: Callable[[float], Optional[str]]) -> float:
    """The amount to turn joint 4 by. `clear_at(turned_deg)` returns None when the claw is clear after turning that far, else why not.
    Raises ValueError with the reasons when neither way works."""
    why = []
    for d in candidates(d_yaw):
        j4 = j4_now + d
        if not J4_MIN <= j4 <= J4_MAX:
            why.append(f"{d:+.0f} deg would take joint 4 to {j4:.0f} (limits {J4_MIN:.0f}..{J4_MAX:.0f})")
            continue
        n = int(abs(d) / STEP_DEG) + 1
        blocked = next((r for r in (clear_at(d * k / n) for k in range(n + 1)) if r), None)
        if blocked:
            why.append(f"{d:+.0f} deg: {blocked}")
            continue
        return d
    raise ValueError("; ".join(why))
