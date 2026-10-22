"""devices/squid.py against a stand-in that behaves like Squid's MultiPointController (commit 489bc836):
run_acquisition starts a worker thread and returns; callbacks.signal_acquisition_finished fires at the end,
or at once when the settings are rejected; acquisition_in_progress tells whether the thread runs."""
from __future__ import annotations

import dataclasses
import os
import sys
import threading
import time
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from devices.squid import Squid, SquidError, SquidNotWired  # noqa: E402


@dataclasses.dataclass
class Callbacks:                       # the fields our device touches, shaped like MultiPointControllerFunctions
    signal_acquisition_finished: object = lambda *a, **k: None


class FakeController:
    def __init__(self, seconds=0.05, reject=False):
        self.callbacks, self.seconds, self.reject = Callbacks(), seconds, reject
        self.base_path = self.experiment_ID = None
        self.thread = None

    def set_base_path(self, path):
        self.base_path = path

    def start_new_experiment(self, experiment_id):
        self.experiment_ID = experiment_id + "_2026-10-22_12-00-00.000000"

    def run_acquisition(self, acquire_current_fov=False):
        if self.reject:                                     # Squid: validation failed, signal finished, no thread
            self.callbacks.signal_acquisition_finished()
            return

        def work():
            time.sleep(self.seconds)
            self.callbacks.signal_acquisition_finished()
        self.thread = threading.Thread(target=work, daemon=True)
        self.thread.start()

    def acquisition_in_progress(self):
        return bool(self.thread and self.thread.is_alive())

    def request_abort_aquisition(self):
        pass


class FakeStage:
    def __init__(self, x=50.0, y=0.0):
        self.x, self.y = x, y

    def get_pos(self):
        return types.SimpleNamespace(x_mm=self.x, y_mm=self.y, z_mm=1.0)


class FakeMicroscope:
    homed = False
    stage = FakeStage()

    def home_xyz(self):
        self.homed = True


def test_unwired_squid_refuses_every_call():
    s = Squid("microscope", {"base_path": "/tmp/x"})
    with pytest.raises(SquidNotWired):
        s.home()
    with pytest.raises(SquidNotWired):
        s.acquire()
    assert s.last_dataset is None


def test_acquire_returns_at_once_and_done_follows_the_finished_signal():
    mpc, scope = FakeController(seconds=0.1), FakeMicroscope()
    s = Squid("microscope", {"base_path": "/data/run"}, microscope=scope, controller=mpc)
    s.home()
    assert scope.homed
    t0 = time.monotonic()
    s.acquire()
    assert time.monotonic() - t0 < 0.05                    # the call does not wait for the acquisition
    assert not s.done() and mpc.acquisition_in_progress()
    assert s.last_dataset.replace("\\", "/").startswith("/data/run/microscope_2026")
    deadline = time.monotonic() + 2
    while not s.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert s.done()                                        # the ack: Squid's own finished callback set it


def test_parked_needs_measured_park_numbers_then_compares_the_stage_position():
    scope = FakeMicroscope()
    with pytest.raises(SquidError, match="park_xy_mm / park_tolerance_mm not set"):
        Squid("microscope", {"park_xy_mm": [50.0, 0.0]}, microscope=scope).parked()
    s = Squid("microscope", {"park_xy_mm": [50.0, 0.0], "park_tolerance_mm": 0.5}, microscope=scope)
    scope.stage = FakeStage(50.3, -0.2)
    assert s.parked()
    scope.stage = FakeStage(50.0, 12.0)                     # the acquisition moved it
    assert not s.parked()


def test_a_rejected_acquisition_is_an_error_not_a_finished_one():
    s = Squid("microscope", {"base_path": "/data/run"}, microscope=FakeMicroscope(), controller=FakeController(reject=True))
    with pytest.raises(SquidError, match="did not start"):
        s.acquire()
