"""
Fake Squid+: waits as long as an acquisition would and does nothing. Same interface as devices.squid.Squid.

On the two-microscope test rig microscope 2 is one of these, so the arm, its lock and the plate list
are exercised on two nests while only microscope 1 images.
"""
from __future__ import annotations

import time
from typing import Optional


class FakeSquid:
    def __init__(self, name: str, acquisition_s: float = 0.0, log=None):
        self.name, self.acquisition_s = name, acquisition_s
        self._log = log or (lambda *a: None)
        self._started: Optional[float] = None
        self.acquisitions = 0
        self.stage_parked = True          # a test sets False for a stage that never parks
        self.park_xy = (0.0, 0.0)         # where home() leaves the stage in Squid's frame

    def home(self) -> None:
        self._log("squid", self.name, "home (fake)")

    def parked(self) -> bool:
        return self.stage_parked

    def position(self) -> tuple:
        return self.park_xy

    def acquire(self) -> None:
        self._started = time.monotonic()
        self.acquisitions += 1
        self._log("squid", self.name, f"acquire (fake, {self.acquisition_s} s)")

    def done(self) -> bool:
        return self._started is not None and time.monotonic() - self._started >= self.acquisition_s

    @property
    def last_dataset(self) -> Optional[str]:
        return f"fake://{self.name}/{self.acquisitions}" if self.acquisitions else None
