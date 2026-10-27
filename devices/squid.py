"""
One Squid+ microscope, driven in-process through the Squid repo (github.com/Cephla-Lab/Squid).

Call names confirmed against Squid commit 489bc836, software/control:
    microscope.py            Microscope.build_from_global_config(simulated=False), .home_xyz(), .close()
    core/multi_point_controller.py
                             MultiPointController.set_base_path(path), .start_new_experiment(experiment_id),
                             .run_acquisition(), .request_abort_aquisition(), .experiment_ID, .base_path;
                             finished: callbacks.signal_acquisition_finished (MultiPointControllerFunctions)

The controller needs the live and autofocus controllers and the objective store the Squid GUI builds, so it
is built where Squid runs and handed in: Squid(name, config, microscope=..., controller=...). Without them
every call raises SquidNotWired, so a real microscope cannot be driven by accident. Tests use devices.fake_squid.

config (layout.json squid.<name>): base_path, experiment_id (default: the microscope name), park_xy_mm and
park_tolerance_mm (where home_xyz leaves the stage, measured on the cell; null = parked() refuses).
"""
from __future__ import annotations

import dataclasses
import os
import threading
from typing import Optional


class SquidNotWired(RuntimeError):
    pass


class SquidError(RuntimeError):
    """The microscope refused or lost an acquisition."""


class Squid:
    def __init__(self, name: str, config: dict, log=None, microscope=None, controller=None):
        self.name, self.config = name, config
        self._log = log or (lambda *a: None)
        self._scope, self._mpc = microscope, controller
        self._finished = threading.Event()
        if controller is not None:
            # the controller calls this when the acquisition is complete (same hook the GUI uses)
            controller.callbacks = dataclasses.replace(controller.callbacks, signal_acquisition_finished=self._on_finished)

    def _need(self, what):
        if what is None:
            raise SquidNotWired(f"{self.name}: build the Squid microscope and MultiPointController where Squid runs "
                                "and pass them to Squid(...); see devices/squid.py")
        return what

    def _on_finished(self, *a, **k) -> None:
        self._finished.set()
        self._log("squid", self.name, "acquisition finished")

    def home(self) -> None:
        """Park the stage so the arm can reach the holder."""
        self._need(self._scope).home_xyz()
        self._log("squid", self.name, "homed")

    def parked(self) -> bool:
        """Stage within park_tolerance_mm of park_xy_mm, the place home() leaves it. Both are measured on the cell."""
        xy, tol = self.config.get("park_xy_mm"), self.config.get("park_tolerance_mm")
        if xy is None or tol is None:
            raise SquidError(f"{self.name}: squid.{self.name}.park_xy_mm / park_tolerance_mm not set; "
                             "home the stage on the cell, read its position and write both into layout.json")
        pos = self._need(self._scope).stage.get_pos()
        return max(abs(pos.x_mm - xy[0]), abs(pos.y_mm - xy[1])) <= tol

    def position(self) -> tuple:
        """The stage's (x, y) in Squid's frame, mm."""
        pos = self._need(self._scope).stage.get_pos()
        return (pos.x_mm, pos.y_mm)

    def acquire(self) -> None:
        """Start the acquisition that is configured on this microscope; returns once Squid's worker thread runs.
        Squid fires signal_acquisition_finished without starting when its settings fail validation: that is an error here."""
        mpc = self._need(self._mpc)
        self._finished.clear()
        mpc.set_base_path(self.config["base_path"])
        mpc.start_new_experiment(self.config.get("experiment_id", self.name))
        mpc.run_acquisition()
        if not mpc.acquisition_in_progress() and self._finished.is_set():
            raise SquidError(f"{self.name}: acquisition did not start (Squid rejected its settings)")
        self._log("squid", self.name, f"acquisition started: {self.last_dataset}")

    def done(self) -> bool:
        """True once Squid reported the end; also true if its thread died without reporting."""
        mpc = self._need(self._mpc)
        return self._finished.is_set() or not mpc.acquisition_in_progress()

    def abort(self) -> None:
        self._need(self._mpc).request_abort_aquisition()

    @property
    def last_dataset(self) -> Optional[str]:
        mpc = self._mpc
        if mpc is None or not mpc.experiment_ID:
            return None
        return os.path.join(mpc.base_path, mpc.experiment_ID)
