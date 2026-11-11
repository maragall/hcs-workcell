# hcs-workcell

Plate-handling workcell: Bioyond Polar arm + Nova vision box, plate hotel, rotation module, up to four Squid+
microscopes. Design docs, figures and plans live in `~/CEPHLA/AI-docs/hcs-workcell`, not here.

## Structure (2026-11-10)

Polar is the arm with its hotel and rotation module. The workflow is three parts (docs/workflow.md): setup once, teach each
instrument, run jobs. One package per part, three packages underneath. Each calls only the ones below it.

- `polar/`: the driver language. `Polar().move(source, destination)`, `pick`, `place`, `where`. Places are slots ("A1"), "module",
  or a taught instrument. `demo.py` is the example; `tools/test_api.py` runs every method on the arm.
- `setup/` (`python cli.py setup`): the one-time setup of the arm as a terminal program. `runner.py` prints the steps,
  `sections.py` is the sections in order, `cameras.py` the camera calibration steps, `cell.py` the two devices and the positions file.
- `teach/` (`python cli.py teach NAME`): the arm goes loose, the user seats the plate in the instrument by its short edges, Enter;
  the seat is recorded and its routes to the rotation module are planned and stored in `preset/routes.json`.
- `run/`: the arm moves with the checks between them. `Cycle` (`cycle.py`) is one class whose methods are grouped by file:
  `motion.py` (moves), `grasp.py` (jaw), `hotel.py` (slots), plus `vision/gates.py`. Also safety, ledger (`state.py`), telemetry,
  recovery, plate types, `positions.py` (loads and saves `layout.json`). `loop.py` is the asyncio main loop for several
  microscopes (`python -m run`).
- `planner/`: `plan.py` gives the waypoints between two positions, every leg checked against the claw, the plate and the links by
  `volumes.py` (boxes from `layout.json volumes`; the hotel is solid). `fk.py` is the arm's kinematics, `joints.py` the joint limits,
  `graph.py` the stored legs. `tools/plan_preset_routes.py` writes `preset/routes.json`: every slot to and from the module.
- `vision/`: photo in, verdict out (`occupancy.py`, `tilt.py`, `predictor.py`, `hub.py`), the camera gates of the Cycle (`gates.py`),
  the corrector (`corrector.py`: what to do on a doubt, `StopSafe`), and `calibration/` (the marker, the capture plan, the solve,
  the upload to the camera box). Vision never stops the loop unless a gate is enforced; occupancy checks are optional and off.
- `devices/`: one thin file per machine and a fake for each. `arm.py` is the vendor SDK, `controller.py` BRCAPI, `clamp.py` the
  force close, `vision_box.py` the camera box.
- `layout.json`: the positions file. Data, not code. `preset/`: the stored routes. `lua/`: what runs on the controller.
- `tools/`: standalone scripts (tests on the arm, figures, measurements). `tests/`: offline tests on the fakes.
- `docs/notes/`: older design notes, kept for reference.

## Hardware handoff (for the session on the cell PC, 2026-10-22)

The refactor and the new checks were built and tested offline on the fakes only. On the cell, in this order, each
step cheap to back out of:

1. `python cli.py status`, then `python cli.py preflight`: the moved imports and the vendor SDK load on Windows.
2. `python cli.py run --mode supervised --slot A1 --cycles 1`: the one-microscope cycle through the new packages.
   If this passes, the refactor broke nothing.
3. The fingers changed (2026-10-22): `python cli.py measure-grip short_edges` and `long_edges` rewrite `gripper.closed`;
   delete `reference_images/gripper/` so the held-plate references are collected again; re-teach the `PLATE_*` values
   in `lua/data_plate.lua`. Then measure and write into `layout.json`: `plate.height`, the finger stand-off above a
   gripped plate, and from those the hotel lift for the 30-slot hotel (17 mm headroom; today's `fixtures.hotel.lift_mm`
   25 is wrong for it). Re-teach the row-1 and row-9 points per column; rows 1..10 are interpolated from them.
   `fixtures.hotel.loaded` lists the 10 plates (odd rows in A, even in C); `python cli.py positions` shows the order.
   The plates differ in colour and weight: the occupancy detector is per slot and only enforces after a clean
   self-test, so watch `python cli.py occupancy` before trusting it.
3b. The second microscope: `python cli.py record-pose microscope_2`, then `regions.microscope_2` (box, z floor), then
   `trace` + `normalise microscope_2_to_polar`. `python cli.py positions` lists what is still NOT TAUGHT.
4. Measure `routes.safe_z_mm` (a height that clears the hotel, the stage and the holder) and try `python cli.py goto`
   between two taught positions.
5. Import `lua/data_plate.lua` into the BioyondWorks project, teach the four `PLATE_*` values, run one pick and place
   through the controller job path with SYSINVAR8 = 0.
6. Let the cell collect its references over a day of supervised runs, then `python cli.py occupancy`. The gates with
   `enforce: "auto"` start blocking on their own once trained; set `enforce: false` on any gate that blocks wrongly.
7. `python -m run --scopes microscope --slots A1 --mode supervised`: the asyncio loop on the real arm with fake imaging.
   Then two nests: `--scopes microscope,microscope_2 --fake-scopes microscope_2 --slots A1,A3`, once `microscope_2`
   has its own fixture entry. The dead microscope always pretends for a short fixed time (`--acquisition-s`), never
   the real acquisition time.
8. Real imaging: build the Squid `Microscope` and `MultiPointController` where Squid runs and pass them to
   `devices.squid.Squid(...)` (call names confirmed, see that file's docstring); `layout.json squid.<name>.base_path`.
9. The stage camera check (`Gates._stage_clear`, references `stage/to_0` and `stage/to_90`) and the hotel headroom
   rule (`fixtures.hotel.headroom_mm`) are new: expect preflight to refuse until the hotel lift is re-taught.

Anything that fails in 1 or 2 is a refactor mistake: report it with the run folder's `events.jsonl`.

## Rules that hold

- Never run anything against the real arm or the vision box from a session; the fakes exist for that.
- Every arm move is gated: correlation check before lowering, jaw check after closing. Keep it that way when
  adding a procedure.
- The product is one microscope. The loop is written for several (one task each, one arm lock); a second one is a new
  fixture entry in `layout.json` and a new name in `python -m run --scopes`, nothing else.
- A new plate type is one block in `lua/data_plate.lua` and the same numbers under `plates.types` in `layout.json`.
- Tests: `python -m pytest tests -q`. `test_sim.py` takes minutes (real sleeps in the jaw procedures).
