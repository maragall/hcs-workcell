# hcs-workcell

Plate-handling workcell: Bioyond Polar arm (the variant without cameras: branch `no-camera`), plate hotel, rotation module, up to
four Squid+ microscopes. Design docs, figures and plans live in `~/CEPHLA/AI-docs/hcs-workcell`, not here.

## Structure (2026-11-14, no cameras)

Polar is the arm with its hotel and rotation module. The workflow is three parts (docs/workflow.md): setup once, teach each
instrument, run jobs. The top level is what a user touches; everything else is nested.

```
demo.py  polar.py  cli.py  cli.bat
setup/   teach/   run/   utils/   docs/
```

Imports go one way: utils.devices, utils.colour, utils.prompt < run.planner < run < setup, teach, polar.py < cli.py and the scripts in utils/.
Nothing imports `cli`, and no `__init__.py` holds code.

- `polar.py`: the driver language. `Polar().move(source, destination)`, `pick`, `place`, `where`; `Microscope()` (`wait`, `home`,
  `back`, `acquire`) talks to a running Squid through its control server (`utils/devices/squid_link.py`, TCP 5050). Places are slots ("A1"), "module",
  or a taught instrument. `demo.py` is the example.
- `cli.py`: `python cli.py setup | teach NAME | doctor | status | positions`. `cli.bat` runs it with the installed environment
  (`cli setup`), so nothing is activated. `setup/install.bat` installs (batch, not PowerShell: the users' PCs block scripts).
- `run/layout.json`: the positions file, every number. Data, not code. `run/hardware.json`: the CAD numbers.
- `setup/` (`python cli.py setup`, ends with READY): `sections.py` is the three sections in order: connect (the arm found on the
  network, connected with retries, enabled, no controller error; every Squid of `layout.json squid` answering on its control
  server; a device that does not answer is asked again in place, `--no-squid` skips Squid), places (`places.py`: the module's two
  seats and six hotel slots shown by hand, the seats' yaw by force, the hotel fitted, routes planned again), preset (the arm's
  program written, parsed, uploaded, read back). `runner.py` prints the steps, `cell.py` is the arm and the positions file,
  `discover.py` finds the arm and asks Squid, `doctor.py` the read-only check of the PC (Windows, .NET, the vendor package part by
  part, BioyondWorks' Lua, the arm, Squid), `sim.py` the fakes for `--sim`.
- `teach/seat.py` (`python cli.py teach NAME`): the arm goes loose, the user seats the gripper on the plate in the instrument, Enter;
  the seat is recorded, its routes to the rotation module planned and stored, the arm's program written again and uploaded.
- `run/`: the arm moves with the checks between them. `Cycle` (`cycle.py`) is one class whose methods are grouped by file:
  `motion.py`, `grasp.py`, `hotel.py`, `health.py`. Also `corrector.py` (`StopSafe`, the operator's questions), `predictor.py` (the jaw
  checks, the drift monitor), the ledger (`state.py`), telemetry, recovery, plate types,
  `positions.py` (loads and saves `layout.json`), `loop.py` (several microscopes, `python -m run`).
  - `run/planner/`: `plan.py` gives the waypoints between two positions, every leg checked by `volumes.py` (boxes from `layout.json
    volumes`; the hotel is solid). `fk.py` the kinematics, `joints.py` the limits, `safety.py` the workspace and speed profiles.
  - `run/preset/routes.json`: the stored routes, every slot and instrument to and from the module.
  - `run/controller/`: what runs on the arm's controller. `mod_Polar.lua` executes a whole pick, place or module turn (jobs 31..34)
    from the table POLAR that `run/lua.py` appends to `data.lua`; `mod_TorqueProbe.lua` is the jaw's force close. Speeds and blending
    in the Lua are not calibrated yet (`layout.json lua`), and `polar.py` still drives the arm from the PC.
- `utils/`: out of the way. `devices/` (one thin file per machine and a fake for each: `arm.py` the vendor SDK, `controller.py` BRCAPI,
  `clamp.py` the force close, `squid_link.py` a running Squid), `colour.py` and `prompt.py` (the terminal), `tests/` (`python -m pytest utils/tests -q`), and standalone scripts: arm tests
  (`test_api.py`, `test_workflow.py`, `test_lua.py`), `plan_preset_routes.py`, figures, measurements.

## Rules that hold

- Never run anything against the real arm from a session; the fakes exist for that.
- There is no camera, and no code path may assume one. Nothing sees whether a seat is occupied: the ledger (`run/state.py`) is what
  says a seat is free. Every grasp is still gated: the force close and the jaw width after closing, the carry check, the module's
  angle read back. Keep it that way when adding a procedure.
- The vendor wheel (`bioyondsdk`, Windows only, needs pythonnet and the .NET Desktop Runtime 8) is how the arm is driven. It has
  nothing to do with cameras and is needed on every real cell.
- The product is one microscope. The loop is written for several (one task each, one arm lock); a second one is a new
  fixture entry in `layout.json` and a new name in `python -m run --scopes`, nothing else.
- A new plate type is one block under `plates.types` in `layout.json`.
- Tests: `python -m pytest utils/tests -q`. `test_sim.py`: 9 of its 18 tests fail, as they did before the cameras were removed (the
  rig's taught route no longer ends at the microscope hover); not yet repaired.
