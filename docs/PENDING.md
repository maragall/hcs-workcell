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

## Must not be forgotten (operator, 2026-11-02)
- The carried plate passes in the plane of the other slots and could rotate into a plate sitting in a neighbouring slot. Not a risk today
  (one plate in the whole cell), a certain collision once the hotel holds several. Cause on my side: since the hotel went from one solid
  box to its real parts (trays, posts, roof, base), the planner treats the space BETWEEN trays as free, and a plate standing in an occupied
  slot is not an obstacle at all. A planned route may cross the rack interior, or turn the claw, at plate height. Fix, in this order:
  1. the planner keeps the whole rack envelope solid (its mouth targets stay outside it, margin 3 mm); only the explicit hotel moves (mouth to
     approach, slide in, lower, row change) use the permissive part model;
  2. every slot the ledger says is occupied is an obstacle (plate box, with its lid, in its tray) for those explicit moves too;
  3. no claw turn while the plate is within reach of the rack front (the turn happens at the module, never at a mouth);
  4. a test with a loaded hotel: every pair of mouths, the plate carried, neighbours occupied.

## Still to do
0. Force latch on the grasp (operator idea, 2026-11-02): the controller's Lua torque probe (`lua/torque_probe/TorqueProbe.bin`, PC side `tools/torque_probe.py`) reads the gripper torque and the vendor's GripperEmptyDet verdict. It is NOT loaded now (prog_state stopped, lua_ready false) and the Python loop does not read torque. To use it: load the probe in BioyondWorks, check that SDK moves still work while it runs, measure the holding torque of a clamped plate against an empty close (`tools/torque_probe.py close ... ` and `burst`), threshold between them, read it after every close next to the width latch.
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
