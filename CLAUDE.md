# hcs-workcell

Plate-handling workcell: Bioyond Polar arm + Nova vision box, plate hotel, rotation module, up to four Squid+
microscopes. Design docs, figures and plans live in `~/CEPHLA/AI-docs/hcs-workcell`, not here.

## Structure (2026-10-22)

Four packages plus the main loop. Each box calls only the boxes below it; `perception` imports no device.

- `run.py`: the main loop, asyncio. Each plate change returns the microscope's plate to its home slot and fetches the
  next (`--order interleave` walks `fixtures.hotel.loaded`). One task per microscope (`run_scope`), one arm shared (`Workcell.arm_lock`),
  one ledger per microscope (`state_<scope>.json`), one telemetry folder per microscope (`runs/<run>/<scope>/`).
  The blocking arm procedures run in threads. A fault in one task stops the others at their next step.
- `cli.py`: teaching (`record-pose NAME` stores where the arm is now), references, tests, one-microscope runs.
- `layout.json`: the positions file. Data, not code. `procedures/positions.py` loads and saves it.
- `procedures/`: the arm moves with the checks between them. `Cycle` (`cycle.py`) is one class whose methods are
  grouped by file: `motion.py` (moves), `grasp.py` (jaw), `gates.py` (camera and health checks), `hotel.py` (slots);
  `cycle.py` keeps the constructor, pick, place, the step tables and `run`. Plus recovery, safety, ledger, telemetry,
  plate types (`plates.py`, mirrored by `lua/data_plate.lua`, chosen by SYSINVAR8), preflight.
- `perception/`: photo in, verdict out. `lower_check.py` is the correlation gate before every lowering (down camera,
  plus the front camera as a second view); `occupancy.py` the slot detector. Self-training gates share `Gates._learned_gate`
  and block only once trained (`enforce: "auto"`). With `vision.hub.record` on, every verdict and every step outcome go
  to `hub_outbox/` for cephla-hub; the drift statistics (`predictor.py`) are the live prediction that runs today.
- `devices/`: one thin file per machine and a fake for each. `nova.py` is the one import for arm + vision box.
- `calibration/`: our replacement for the vendor's Windows calibration tool (HALCON 12 maths, two text files on the
  box). The target is the 40 mm marker on the practice plate (`marker.py`, cv2.aruco). `plan.around` is the one capture
  plan (three heights, x/y spread, four yaws, ends where it started); `solve.run` is one joint least squares over the
  camera model (`CamParam.dat`, HALCON area_scan_polynomial), the camera-in-tool pose (`CamInTool.dat`, HALCON pose
  type 2 = Rz(g) Ry(b) Rx(a)) and the marker-in-base pose; a SCARA cannot fix the camera's offset along tool z, so the
  marker is pinned at base z = 0 as the vendor's file implies. `upload` SFTPs to
  `/home/pi/Nova/data/calibration/<camera>/active/` with credentials from `layout.json vision.ssh`, `verify` reads the
  marker from two poses. `python3 -m calibration.validate` renders the marker through the vendor's files in
  `vendor_data/` and solves it back (`synth.py`). `cli.py calibrate <camera> read|capture|upload|check`.
- `bringup/` (`python cli.py setup [--sim] [--from SECTION]`): the one-click setup as a terminal program. `runner.py` prints the
  steps and waits for Enter on the user's; `sections.py` is the sections in order, each line calling what `cli.py`, `Cycle` and
  `exercise` already call; `geometry.py` the arithmetic (stage seat from the practice plate's marker); `cell.py` the two devices, the positions file
  and the arm's readout; `cameras.py` the calibration steps; `hand.py` the arm loose or locked. `perception/hotel_size.py`
  tells 15 from 30 slots by pictures. Tests: `tests/test_bringup.py`.
- `routes/`: `plan.py` picks a taught trajectory, else lift / travel / lower, every leg checked against the claw (205 mm ahead of the
  reported wrist pose) and the links by `volumes.py` (obstacles and claw from `layout.json volumes`, which come from the CAD, see
  `hardware.json`). With `routes.safe_z_mm` the travel is at that height; with it null the planner searches a path around the volumes;
  `Motion.move_to(position)` executes it, to the hover only. `cli.py goto NAME` drives it by hand.
- `tools/`: standalone scripts that are not part of the loop: `analyze_runs.py`, `build_refs_from_runs.py`, `record_slot.py`,
  `teach_daemon.py`, `torque_probe.py`, `run_matrix.py`, `review_queue.py`, `placement_diagram.py` (redraws `docs/placement.png`).
  There is no GUI: `python cli.py setup` replaces it, `python cli.py recover` is its Recover button.
- Stage safety, two layers, no clocks. Keep-out: `regions.stage_sweep` (`keep_out: true`), the stage's travel envelope at
  its closest approach to the arm; `Safety` refuses any target, segment or joint move whose tool point, elbow or links
  (elbow via `routes_opt/fk`) enter it. Interlock: `run.py` homes the stage and waits for `parked()` inside the arm lock;
  `Motion` enters a `needs_parked` region only while `parked()` is True; `acquire()` runs after the lock is released.
  To measure on the cell (null = refused): `regions.stage_sweep.box`, `squid.<name>.park_xy_mm`, `park_tolerance_mm`.

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
7. `python run.py --scopes microscope --slots A1 --mode supervised`: the asyncio loop on the real arm with fake imaging.
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
  fixture entry in `layout.json` and a new name in `run.py --scopes`, nothing else.
- A new plate type is one block in `lua/data_plate.lua` and the same numbers under `plates.types` in `layout.json`.
- Tests: `python -m pytest tests -q`. `test_sim.py` takes minutes (real sleeps in the jaw procedures).
