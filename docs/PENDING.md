# Pending (2026-11-02)

## Running
- `python tools/keep_running.py --speed supervised` keeps `cli.py shuffle` going: random plates, microscope <-> slots A1..A9, B1..B9.
  It logs to `runs/keep_running.log`, restarts after a stop with a growing wait, and gives up (leaves the arm, writes
  `runs/SUPERVISOR_GAVE_UP.txt`) when the same reason stops it 4 times in a row. Stop it: create `runs/STOP`.
- Column C is out of the shuffle. The module and microscope are taught on the elbow branch j3 < 0, as are A and B; C is on j3 > 0. Coming back
  from C the arm reaches the module on the C branch and the planner has no route to the microscope from there. Fix when wanted: after the plate
  is put down on the module (claw empty) the arm goes to the nest and returns, which puts it on the taught branch (the shuffle start does
  this already for a plate left in the claw).

## Branches (nova_cell)
- `hotel-2026-11-02`: today's work (the branch the loop runs from).
- `cleanup-2026-11-02`: deletions, not merged: offline route optimizer, learning-loop tooling, manual and placement builders. Run the tests on it, then merge.
- `dedup-2026-10-26`, `proven-2026-10-22`: the two old worktrees, removed from disk; their history is on these branches.
- Loose files from the parent folder are in `removed_loose_files_2026-11-02.zip`.

## Still to do
1. Tilt detection after seating, front camera.
2. Vision offset correction: `get_pose_offset` fails with box error 1002; every pick uses the taught pose.
3. Seated-front references are stale on the new hotel; the plate-in-gripper check says "not held" while the jaw says held. Auto-confirmed doubts also
   save the picture as a reference: make them log only.
4. Station graph: its key included volatile flags (`plate_on_module`), so no stored leg matched. Fix the hash, build both elbow branches,
   measure one shuffle, delete it if planning time does not fall.
5. Planner: keep the collision checker; routine legs from a table keyed by elbow branch; search for recovery only.
6. Taught-route replay (`routes/trajectory.py`, taught branches in `plan()` and `hotel.py`, `polar_to_hotel*` in layout.json): delete.
7. Ledger: cut to plate_at, grip, step in progress, hotel occupancy.
8. Tests: after the cleanup branch is merged, repair or delete the failing ones (the second-run wizard test, the simulator test).
9. Measure the slot lip (3 mm is an estimate); `stress` speed, a lidded plate, many cycles.
10. A plate in the claw was released while the arm sat stopped after a refusal (jaw 24.8 at 00:23, arm 'not enabled' at 00:27, jaw -0.18 at 00:36). Probable cause: the controller drops the servos some minutes after the last command and the gripper loses its hold. Unconfirmed. Never leave a plate in a stopped claw for minutes; keep_running restarts within 10-45 s and gives up after 4 identical stops. Find out whether a read-only connection keeps the servos up.
11. Older: stage speed 0.08, collision values from BioyondWorks, camera decision for the microscope check, video policy for the 44 GB on D:, box
    calibration scale, real Squid.
