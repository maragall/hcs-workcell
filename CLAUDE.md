# hcs-workcell

Plate-handling workcell: Bioyond Polar arm + Nova vision box, plate hotel, rotation module, up to four Squid+
microscopes. Design docs, figures and plans live in `~/CEPHLA/AI-docs/hcs-workcell`, not here.

## Structure (2026-11-10)

Polar is the arm with its hotel and rotation module. The workflow is three parts (docs/workflow.md): setup once, teach each
instrument, run jobs. Imports go one way only: devices < planner, vision < run < setup, teach, polar < cli.py, tools. No package
imports `cli` or `tools`, and no `__init__.py` holds code.

- `polar/` (`api.py`): the driver language. `Polar().move(source, destination)`, `pick`, `place`, `where`. Places are slots ("A1"), "module",
  or a taught instrument. `demo.py` is the example; `tools/test_api.py` runs every method on the arm.
- `setup/` (`python cli.py setup`): the one-time setup of the arm as a terminal program. `runner.py` prints the steps,
  `sections.py` is the sections in order, `cameras.py` the camera calibration steps, `cell.py` the two devices and the positions file, `doctor.py` the read-only check of the PC.
- `teach/` (`seat.py`, `python cli.py teach NAME`): the arm goes loose, the user seats the plate in the instrument by its short edges, Enter;
  the seat is recorded and its routes to the rotation module are planned and stored in `preset/routes.json`.
- `run/`: the arm moves with the checks between them. `Cycle` (`cycle.py`) is one class whose methods are grouped by file:
  `motion.py` (moves), `grasp.py` (jaw), `hotel.py` (slots), plus `vision/gates.py`. Also the ledger (`state.py`), telemetry,
  recovery, plate types, `positions.py` (loads and saves `layout.json`). `loop.py` is the asyncio main loop for several
  microscopes (`python -m run`).
- `planner/`: `plan.py` gives the waypoints between two positions, every leg checked against the claw, the plate and the links by
  `volumes.py` (boxes from `layout.json volumes`; the hotel is solid). `fk.py` is the arm's kinematics, `joints.py` the joint limits, `safety.py` the workspace, floors and speed profiles,
  `graph.py` the stored legs. `tools/plan_preset_routes.py` writes `preset/routes.json`: every slot to and from the module.
- `vision/`: photo in, verdict out (`occupancy.py`, `predictor.py`, `hub.py`), the camera gates of the Cycle (`gates.py`),
  the corrector (`corrector.py`: what to do on a doubt, `StopSafe`) with its terminal prompt, and `calibration/` (the marker, the capture plan, the solve,
  the upload to the camera box). Vision never stops the loop unless a gate is enforced; occupancy checks are optional and off.
- `devices/`: one thin file per machine and a fake for each. `arm.py` is the vendor SDK, `controller.py` BRCAPI, `clamp.py` the
  force close, `vision_box.py` the camera box.
- `layout.json`: the positions file. Data, not code. `preset/`: the stored routes. `lua/`: what runs on the controller.
- `tools/`: standalone scripts (tests on the arm, figures, measurements). `tests/`: offline tests on the fakes.
- `docs/`: `workflow.md` is the guide; `INCIDENTS.md` and `VENDOR_SDK.md` are records.

## Rules that hold

- Never run anything against the real arm or the vision box from a session; the fakes exist for that.
- Every arm move is gated: correlation check before lowering, jaw check after closing. Keep it that way when
  adding a procedure.
- The product is one microscope. The loop is written for several (one task each, one arm lock); a second one is a new
  fixture entry in `layout.json` and a new name in `python -m run --scopes`, nothing else.
- A new plate type is one block in `lua/data_plate.lua` and the same numbers under `plates.types` in `layout.json`.
- Tests: `python -m pytest tests -q`. `test_sim.py` takes minutes (real sleeps in the jaw procedures).
