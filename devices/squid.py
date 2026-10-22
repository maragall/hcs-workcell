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

config (layout.json squid.<name>): base_path, experiment_id (default: the microscope name).
"""
from __future__ import annotations

import dataclasses
import os
import threading
from typing import Optional


class SquidNotWired(RuntimeError):
    pass


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

    def acquire(self) -> None:
        """Start the acquisition that is configured on this microscope; returns at once."""
        mpc = self._need(self._mpc)
        self._finished.clear()
        mpc.set_base_path(self.config["base_path"])
        mpc.start_new_experiment(self.config.get("experiment_id", self.name))
        mpc.run_acquisition()
        self._log("squid", self.name, f"acquisition started: {self.last_dataset}")

    def done(self) -> bool:
        self._need(self._mpc)
        return self._finished.is_set()

    def abort(self) -> None:
        self._need(self._mpc).request_abort_aquisition()

    @property
    def last_dataset(self) -> Optional[str]:
        mpc = self._mpc
        if mpc is None or not mpc.experiment_ID:
            return None
        return os.path.join(mpc.base_path, mpc.experiment_ID)
