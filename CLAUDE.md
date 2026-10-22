# hcs-workcell

Plate-handling workcell: Bioyond Polar arm + Nova vision box, plate hotel, rotating stage, up to four Squid+
microscopes. Design docs, figures and plans live in `~/CEPHLA/AI-docs/hcs-workcell`, not here.

## Structure (2026-10-22)

Four packages plus the main loop. Each box calls only the boxes below it; `perception` imports no device.

- `run.py`: the main loop, asyncio. One task per microscope (`run_scope`), one arm shared (`Workcell.arm_lock`),
  one ledger per microscope (`state_<scope>.json`), one telemetry folder per microscope (`runs/<run>/<scope>/`).
  The blocking arm procedures run in threads. A fault in one task stops the others at their next step.
- `cli.py`: teaching (`record-pose NAME` stores where the arm is now), references, tests, one-microscope runs.
- `layout.json`: the positions file. Data, not code. `procedures/positions.py` loads and saves it.
- `procedures/`: the arm moves with the checks between them. `Cycle` (`cycle.py`) is one class whose methods are
  grouped by file: `motion.py` (moves), `grasp.py` (jaw), `gates.py` (camera and health checks), `hotel.py` (slots);
  `cycle.py` keeps the constructor, pick, place, the step tables and `run`. Plus recovery, safety, ledger, telemetry,
  plate types (`plates.py`, mirrored by `lua/data_plate.lua`, chosen by SYSINVAR8), preflight.
- `perception/`: photo in, verdict out. `lower_check.py` is the correlation gate before every lowering; `occupancy.py`
  the slot detector. Gates with `enforce: "auto"` in `layout.json` block only once trained (`Gates._enforce`).
- `devices/`: one thin file per machine and a fake for each. `nova.py` is the one import for arm + vision box.
- `routes/`: `plan.py` picks a taught trajectory or lift / travel at `routes.safe_z_mm` / lower, every leg checked;
  `Motion.move_to(position)` executes it, to the hover only. `cli.py goto NAME` drives it by hand.

## Rules that hold

- Never run anything against the real arm or the vision box from a session; the fakes exist for that.
- Every arm move is gated: correlation check before lowering, jaw check after closing. Keep it that way when
  adding a procedure.
- A new microscope is a new fixture entry in `layout.json` and a new name in `run.py --scopes`; no loop change.
- A new plate type is one block in `lua/data_plate.lua` and the same numbers under `plates.types` in `layout.json`.
- Tests: `python -m pytest tests -q`. `test_sim.py` takes minutes (real sleeps in the jaw procedures).
