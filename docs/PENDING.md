# Pending work and decisions (2026-11-01, end of the hotel/planner session)

## Decisions recorded today

1. A seat pose is the wrist's x, y, z, yaw. The plate centre is 189.2 mm ahead of the wrist along the claw (CAD: 185). Zeroing a recorded yaw t moves
   the wrist so the plate stays put: dy = 189.2*sin(t), dx = 189.2*(cos(t)-1). (Seat yaw -5.5 deg = 18 mm of plate.) A taught yaw is turned to 0 about the plate, never by keeping the wrist y
   (the 18 mm "bent column B"). `tools/fit_hotel_grid.py` fits the pivot (189.2; CAD 185): columns at y +134.1 / -0.1 / -134.0.
2. The hotel is modelled as a hotel, not a box: `layout.volumes.obstacles.hotel.parts` (`tools/hotel_volumes.py`, from the STEP): trays with a
   3 mm lip, rear posts, roof, base. Slot length along x: 138.0 mm (x 412.6 to 550.6); posts 40 mm deep; 88 mm between posts for an 85.1 mm plate.
3. The tray set follows the hotel configuration (odd rows = 15 slots, even rows, or all ten = 30 slots), not a guess: tools/hotel_volumes.py reads
   fixtures.hotel.rows. Operator 2026-11-02: the hotel can be configured as evens or as 30 slots, so nothing needs confirming.
4. Lift above a seat is 7 mm (lip 3 + clearance), not 25. The lip height (3 mm) is the operator's estimate, not measured.
5. The mouth is the wrist x at which the front of what the claw carries is 5 mm short of the hotel's first part (x 412.6 less its 1 mm margin):
   plate on the long edges 251.9 mm ahead of the wrist -> wrist x 154.7; empty fingers 205 ahead -> never behind the approach point (x 191). The
   hard-coded STAGE_MM is gone. Row changes go
   out to the mouth, change height there, and go back in (joint legs where an end is beside the arm's axis).
6. Station graph (`routes/graph.py`, `tools/build_stations.py`, `station_graph.json`): legs between named stations are planned once, checked,
   stored and replayed; `plan()` searches only when the arm is not at a station.
7. REMOVED (operator, bloat): second front-camera view before lowering (`lower_check.front`); slot occupancy check in the cycle, CLI and
   exercise tool (the ledger is the guard). Kept: the seated-front check (base for tilt detection). `perception/occupancy.py` and its
   photos remain on disk because the setup's hotel-size routine (HotelRows) imports them.
8. Plate variety (colour, lid): dropped as an item. Plates share a shape; no trained classifier remains that would need per-variant data. Lid adds
   2.43 mm (the lift window fits it).
9. SCRATCHED by the operator 2026-11-02: restore the learning loop (the job is now a stress test, random slots <-> microscope); train or retire the
   plate-in-gripper gate; vision-based recovery correction; run the real shuffle with the station graph (it is running); check the final slide-in
   and lowering against the hotel parts with a clearance per placement; measure the real lip and the tray above a row; legs between hotel columns;
   confirm the odd-row trays; the original 5 deg yaw fault (the operator taught it by hand); held-out evaluation of classifiers.

## Pending - vision

1. Tilt detection after seating, front camera (operator request): flat vs tilted per slot, trained on flat and deliberately tilted examples.
2. Vision-based recovery correction (operator: "maybe too complex"): cheap first step = log a front-camera frame at every collision; learn
   from those frames before any automatic sideways correction.
3. Vision offset correction: `get_pose_offset` fails with box error 1002; the cell is in commissioning mode and carries on with the taught pose.
   Diagnose (marker visible? reference stored? workspace name?) or build a marker-free down-camera offset.
4. plate_in_gripper gate reports "not held" while the jaw reads 3.71 (held): train it or retire it.
5. seated_front references are stale on the new hotel model (not_seated_front on every column A slot): re-collect.
6. Held-out evaluation of any classifier that stays (the old occupancy 98.6/100 was in-sample).

## Pending - motion and planner

7. Station graph: 52 legs planned, 10 had no route (every column C leg back to the module). Cause suspected: the offline build used one elbow
   branch for all stations, but column C is on the other branch (teach joints j2 +18 / j3 +65 against -17 / -65 in A). Build both branches.
8. Run the real shuffle (microscope -> module -> hotel slots) with the station graph and the new hotel model; it has not run since the changes.
9. The final slide-in and lowering into a slot is not checked against the hotel parts (only the approach and row changes are). Add the check,
   model the lip as a rim, and print a clearance per placement.
10. Measure the real slot lip and the tray above a row (assumption 3, decision 4).
11. `stress` speed profile on all 15 slots; a lidded plate at a seat; repeated cycles. Only one pass at `fast` (B) and `supervised` (A, C) was run.
12. Plan legs between hotel columns (mouth to mouth) if the shuffle ever needs them; today every plate goes through the module.
13. Slow planner detours: A5 went "via the nest" with a 14.5 s joint leg; the station graph should remove these once built for both branches.
14. Cause of the original 5 degree yaw fault is still unexplained (hypotheses only).

## Pending - cell state and ledger

15. Audit the ledger (`StateStore`): keep plate_at, grip, step in progress, hotel{slot: plate}; the per-slot `contents` table, plate ids,
    home_slot and the start-of-shuffle sync with layout.json look like bloat (not audited).
16. `fixtures.microscope.plate_on_stage` and `fixtures.hotel.active_slots` (currently column B only) need setting for the next run; all 15 seats
    are now verified with a plate.
17. No learning loop is running (converge / probe tours); the learn-from-overnight-loops rule applies before any unattended run.

## Pending - tests and housekeeping

18. Tests: `test_a_second_run_recognises_the_hotel_without_asking` stops at the cameras.toml step; `test_the_whole_setup_runs_on_the_simulator`
    fails (FakeVision.frame_full missing); a full run after today's changes had several failures that have not been read yet.
19. None of today's code and layout changes is committed; nothing is pushed.
20. The STEP file is used for the hotel only; the module, the pillar and the microscope footprint can use it the same way.

## Open since earlier sessions (outstanding-operator-items)

21. Stage speed 0.02 -> 0.08 (awaiting "stage 8"); collision detection values from BioyondWorks; fast-profile validation "go"; camera decision for
    the microscope seated check; video policy for the 44 GB of frames on D:; presence for supervised passes.
22. Earlier deferred: box calibration scale, tighter limits, 15-slot grip_back, real Squid.
