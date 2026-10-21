# STATUS 2026-10-20 17:20 (loop running, operator in the building)
- 16:45 operator: with the plate held free on the table the pads sat on the skirt (which protrudes past the plate wall
  at the corners; the two grooves are in the wall). Hand-adjusted into the grooves at the same z (46.5), so the skirt
  landing is lateral placement, not height. Plate then hand-placed at the microscope.
- 16:58 arm brought from the table to the microscope hover: straight lift to z 331, then a stepped joint move. The
  seat's drag-taught joints (elbow -187.2) sit past the elbow soft limit (-180): error 3571 4 mm short. Cleared and
  finished on the route's own first-waypoint joints (elbow -177.6, j4 178.6). Jaw end stop measured: 29.28.
- 17:04 first supervised cycle (gates and vision correction off as last night). Microscope pick stalled 22.87-22.89
  at every z and x; y+0.4 gave 22.59. The search was judging trials against the free-plate width 22.40 instead of the
  holder signature 22.50, so its 22.54 hits were rejected and the attempt failed twice (safe stop at the hover).
  Fixed (commit b5c0a0e), microscope search range 0.8 mm, stage short-edge opening 29 (per-fixture open_width).
- 17:07 to 17:17 FIRST CLEAN FULL CYCLE with the production plate (run 20261020_170741, 592 s incl. restart):
  microscope pick 22.54 first close, place at 0, turn, long-edge pick at 90 **1.61** (was 2.21 to 2.46 last night),
  A1 place and pick **1.61**, place at 90, turn to 0, short-edge pick at 0 **22.61** with the 29 opening (the descent
  that collided at 23:08 yesterday went clean), place at the microscope. No faults.
- 17:18 LOOP LAUNCHED: `run_forever.py random --profile fast` (40/20/8 %), random A-column slots, recording 1 fps,
  overnight_report.py still writing runs/OVERNIGHT.md. Stop with runs/STOP. Stress profile (50/25/10) only after a
  clean block of cycles at fast.
- 17:18 to 17:52 five clean loop cycles (A1, A5, A7, A7, A5; 393 to 425 s). Widths: microscope 22.42 to 22.55,
  stage at 90 1.55 to 1.61, hotel 1.56 to 1.61, stage at 0 22.33 to 22.74. A borderline 22.42 had sent the search
  down 0.3 mm (seat now 264.39); grasp_ok now accepts anything between the plate width and the signature, and the
  microscope search is lateral only (commit 89ea085). Stopped at a boundary, 22/22 sim tests pass, relaunched 17:59
  detached (Start-Process, PID 28692, log D:\nova_cell_runs\forever_stdout.log) so no tool timeout can kill it.
- Open: sim tests read the real controller's error list (not hermetic; one test failed on the live 3571 at 17:03), rerun
  when the arm is idle; plate position at 0 is now covered by the 3.3 mm clearance, the 22.6x widths at 0 will show
  the drift cycle by cycle in OVERNIGHT.md.

# STATUS 2026-10-20 04:10 (operator present, no cycle running)
- 02:50 operator back: suspected the long-edge grip too wide and too soft. Jaw close changed to one command from
  open to the taught value (vendor behaviour, 3 s instead of 10). First hand test on the table: 1.64, "much tighter".
- 03:30 to 04:05 the controller's gripper torque readout is working: BioyondWorks 1.1 installed on this PC
  (`C:\Bioyond\RobStudio`), a stand-alone Lua probe (`lua/torque_probe/`, importable `TorqueProbe.bin`) published to
  the controller (the vendor's project exported first by the operator as "original-bioyond"), `torque_probe.py` reads
  width and torque at 10 Hz over the JSON port.
- Result: holding torque is 70 % of rated after EVERY close method (one command, stepped, SDK or Lua, target 1.45 or
  0.8): the servo pushes at its configured cap. Stepping never weakened the grip; the soft feel was the drag-mode
  re-hold. The remaining problem is where the pads land: 1.62 to 1.67 on a free plate, 2.21 to 2.46 in the stage nest
  and at A1. Next: torque and width at the stage and A1 after a real pick, and a look at the pads vs the groove.
- Constraint learned: SDK motion is refused while a Lua program runs; BioyondWorks holds the one SDK session.
  Program stop/start costs ~3 s. Details in lua/README_LUA.md.
- Arm: moved by hand by the operator (pose ~97, 127, 143), stage hand-turned to ~103 deg; plate on the table in the
  fingers at 1.66. Lua probe running, servos enabled, auto.

# STATUS 2026-10-19 22:00 (operator away until late evening)
- Evening: new gripper fingers and the production plate. Grips measured: short edges 22.40, long edges 1.45; the fingers on the wall above the groove read 22.37 / 1.50, so jaw width cannot prove the groove. Plate: 84.57 x 125.03 between the grip edges, 85.14 x 127.46 over the skirt; height and groove height not given.
- Seats re-taught by drag with the new plate: microscope (124.0, 399.2, 265.2), polar0 (312.5, 304.5, 66.8), polar90 (312.4, 305.8, 65.6, stage at exactly 90), A1 (293.3, 137.2, 74.6), A9 (291.1, 136.9, 379.1), C9 (293.5, -128.4, 379.0). Row pitch 38.1 mm, column pitch 132.7, rack level. Via point in front of the hotel (127.1, -14.1, 431.5). Region floors lowered to the new seats; workspace ceiling 435.
- Code: rows above 1 go retreat (to x = climb_x 127) -> column-only climb -> slide in; `--slot random` draws from fixtures.hotel.active_slots (A column tonight; B/C wait for the FK planner because their approach points sit near the shoulder axis at (126, 0)). Stage angle synced from readback at start. Front-camera frames lit by the LED (validated). Tactile centering search: when the jaw stalls wider than the plate, the seat is walked in 0.2 mm steps along x then y until the fingers meet the plate at its width, and the working offset is written back to layout.json.
- Wrist branch: dragging left the wrist one turn from the routes' branch (j4 -181.75 vs 178.6, same yaw); a route replay would have spun the plate at the hover. Unwound the empty open gripper at z 425, back at the hover on the right branch.
- Offline route study (routes_opt/): SDK joint order is column, shoulder, elbow, wrist; FK fitted to 0.04 mm; recommended route B 22 waypoints, 56 s one way at 50 % vs 93 s; nothing replayed yet. Microscope seat is 0.8 mm inside the reach limit.
- 21:46 first cycle with the new plate stopped at the microscope pick: jaw stalled at 22.77 (plate 0.2 mm off-centre in the clamped holder). 21:55 restarted with the centering search.
- 22:16 the first cycle with the new plate picked at the microscope (22.51), carried to the stage, placed, rotated to 90. The long-edge pick at the stage stalled at 2.33 on every trial (nominal 1.45, 12 trials, spread 0.09).
- 22:27 to 22:57 INCIDENT, my bug: the search's yaw trials at the stage were refused by the wrist (near its limit) and my yaw-splitting loop re-issued tiny moves from the readback for 30 minutes; the arm crept 4.2 mm below the seat and the controller's wrist collision detection stopped it (error 3873). Killed the run, cleared, opened the fingers, lifted to the hover. Plate on the stage, no visible damage in the camera frames; check the stage nest and the plate by eye. Fixes: every linear move now verifies yaw too (0.5 deg) and stops on refusal; the search's parts are counted once and targeted from the goal pose, never from the readback; yaw removed from the stage search.
- 23:04 to 23:08 the resumed cycle completed the stage pick at 90 (2.21), route A, A1 place, A1 pick (2.46), route A back, place at 90, turn to 0. The short-edge pick at 0 then ended in a second wrist collision stop: the fingers (open 26, 1.8 mm clearance per side) met the plate, whose position at 0 after the robot's place at 90 and the turn is not where the hand-taught pose_at_0 says. Recovered upward; jaw works; plate on the stage; arm parked above it at z 161, enabled, jaw open. NO further automated stage picks tonight. Details and plan in docs/INCIDENTS.md.
- Grasp acceptance now: per-fixture signature when learned (microscope 22.50 +- 0.08), otherwise a band per grip (short 22.3 to 22.9, long 1.35 to 2.5). Groove-or-wall cannot be read from the width (0.03 to 0.05 mm apart); it goes to the camera layer: the held-plate front view after each lift now auto-collects references.
- Faults seen while teaching: stage servo fault 0x3230 when driven while hand-turned (cleared), error 3874 (stage over speed in drag), 3571 (soft limit) once.

# STATUS 2026-10-19 05:55
- 04:44 the slot gate stopped the run again ("A1 occupied", plate in gripper at the A1 approach point, no contact). Cause found: the room light went off between cycles 512 and 513; the gate's references were daylight recorder frames from 10-18, and under the night LED the empty slot scored 0.56 against "empty" and 0.81 against "occupied". Same-pipeline night frames agree with each other at 0.97.
- Fix: the slot gate now collects its own references live, from the same camera call at the same approach pose (place approach = empty, pick approach = occupied), 5 quickly then one per 30 min up to 24, so the set covers a day's lighting. Old daylight references moved to D:\nova_cell_runs\refs_old_20261019. Pick side also logs a verdict (shadow). Simulator tests pass (26).
- 05:48 relaunched: supervisor PID 17732, stress profile, resumed from the approach point; A1 place and pick completed, grasp 2.84.
- Overnight jobs left running (both read-only, never touch the arm): the loop above, and overnight_report.py (PID 31156) writing runs/OVERNIGHT.md every 30 min: cycles, dt, stops, faults, grasp statistics, slot-gate scores, seated_front verdicts.
- Seated_front shadow: 12 references per gate collected; first 5 verdicts per gate all safe, scores 0.994 to 0.997 against thresholds 0.58 to 0.84.

# STATUS 2026-10-19 03:15
- Stress run live on A1 at 50/25/10 % (supervisor PID 26708): full cycles 398 s and 394 s, zero stops. Lighting-robust edge gate in use; slot gate on A1; seated_front check collecting references in shadow mode; run lock prevents a second loop.
- Earlier tonight: slot gate false 'occupied' traced to lighting (raw intensity NCC); release tolerance split from grasp tolerance (0.3 vs 0.08); a duplicate supervisor was killed before it reached the arm.

# STATUS 2026-10-19 01:40
- Unattended run ended 10-18 21:23 after 492 cycles (disk full). Arm parked holding the plate above the rotating stage, step place:polar, disabled.
- runs/ moved to D:
ova_cell_runs (43.8 GB) with a junction at nova_cell/runs; C: has 49.8 GB free again. Recording is now a ring buffer.
- Vision gates proven blind from the taught hovers (see ANALYSIS.md section 9). Camera decision pending.
- Speed/stress profiles and the 13-point route are ready for supervised validation. Grasp tolerance 0.08.
- Repo remote set to github.com/maragall/arm-setup (push pending login). Docs: docs/REPLICATION.md, docs/VENDOR_SDK.md.
- Open measurements: rack front/side by touch, shelf clearance, scope highest obstacle, holder lip, collision sensitivity values, ArUco markers, plate barcode.

# STATUS 2026-10-15 02:00 (previous entries below)

## Tonight
- 03:01 UNATTENDED LOOP STARTED: run_forever.py A1 (microscope <-> A1 ping-pong), 20 % profile, gates/vision correction off, drift warn-only, recording 1 fps. Stop with runs/STOP. Supervisor relaunches only on recoverable stops (3/h, 10/day).
- 02:37 A3 run: collision stop during the joint move from the route end (257,234,103) to the A3 pre-approach (193,137.5,174). The arm stopped at (227,193,121), plate still held (jaw 3.05, was 2.84). Diagnosis: for rows above 1 the pre-approach is higher than the route end, and the joint move climbs while still inside the rack's shelf depth, so the plate sweeps up into a shelf. Fix to apply: retreat in -x to a climb column clear of the shelves, climb vertically there, then approach. ARM LEFT IN PLACE; operator must look before any motion.
- 02:25 FIRST FULL CYCLE through the real cycle code: microscope -> stage -> rotate -> hotel A1 -> back -> microscope. 331 s. Two safe stops on the way, both correct: a 1.5 mm demo-pose mismatch (tolerance now 6 mm on waypoints, joints strict) and a real collision stop when the hotel pick slid in at plate height (pick now mirrors the place). Matrix run A3..C9 started 02:27 with gates and vision correction off.
- Taught: microscope seat+hover, rotating-stage seat (0 and -90) + hover, A1 corrected +2 mm x.
- Rotating stage = robot external axis 2; zero reference 89.808; driven and verified by readback.
- Jaw scale resolved: value = gap; long-edge 2.84/open 6.0, short-edge 24.0/open 26.
- Vendor SDK completion flags are unreliable; every move is verified by readback (robot.py).
- Route A (stage -> hotel) recorded, normalised, trimmed; replayed; full hotel pick/place/pick + return + place on stage ran clean at 30 %.
- Open: route B (stage <-> microscope), reference images, stage rotation + short-edge pick, hand-eye check, collision sensitivity on the pendant.

# STATUS 2026-10-14 23:15

## Decisions taken (correct me)
1. Robot layer moves to the vendor's documented path: BRCAPI JSON protocol on controller port 56788 + a Lua
   module running ON the controller. Motion, gripper, collision supervision and grasp detection live in Lua.
   The PC only sets job variables (setSysVarI) and waits for the done push. The .NET SDK (port 52525) is
   undocumented in the S4 package; keep it only for teaching utilities until the Lua module is loaded.
2. Vision stays on the box's TCP API (own client, no vendor Python). Gates = normalised cross-correlation.
3. Topology: microscope -> polar -> hotel slots. Arm is the agent on the edges, not a node.
4. Poses, floors, speeds, trajectories are data (layout.json / data_cell.lua), never code.

## Discoveries today
- Controller: firmware 1.14, manual mode, disabled after your power cycle, no error, program stopped.
- Gripper = servo external axis E1. Controller offers GripperEmptyDet (torque) and drop detection
  (EnableGripperDet). That is real grasp sensing; the PC-side jaw-value inference becomes a backup.
- The polar stage is probably robot external axis E2 (reads -108 deg now). If true, no separate driver.
- Units: controller reports metres/radians; the .NET SDK reports rz in DEGREES (0.327 rad = 18.75).
  The hotel yaw "-1.028 rad" in the knowledge base may actually be -1.03 deg. Must verify at A1.
- Collision sensitivity is set in BioyondWorks -> Settings -> Collision detection, only while disabled.
  This is a prerequisite you do on the pendant PC; no protocol command exists.
- Vision box: calibration handlers broken (proxy in place), auto-light ON on both cameras (to switch off
  before references), ArUco currently not visible to either camera.
- Why the jaw refused to open: unknown. State was Auto+enabled via SDK, yet ext-axis moves returned false.
  Power cycle done; will retry.

## Built (nova_cell/, git, 4 commits, tests green)
vision client, robot adapter (+simulator), safety (box, z floors, 1 mm deviation stop), lower-check gate,
state machine with resume refusal, predictor (pre-checks + drift), corrector (budgets), trajectory
record/normalise, telemetry, cycle, CLI, 5 offline tests, review_queue.py, DESIGN.md, README.md.

## In progress (background)
- Lua module mod_Cell.lua + data_cell.lua for the controller, template-conformant (agent drafting).
- Next on the PC: brcapi.py client; then robot.py gets a BrcapiRobot behind the same interface.

## Blockers, in priority order
1. Jaw will not open -> no plate in gripper -> no teaching. Retry after power cycle.
2. Collision sensitivity not yet set to max (BioyondWorks, you).
3. Microscope pose, polar@0 pose, polar@90 pose, two routes: not taught.
4. Microscope z floor and holder/rail/stage lip clearances: not measured.
5. Hand-eye calibration error 827.8 on the Pi: unverified; vision correction untrusted until checked.
6. Polar stage interface: E2 hypothesis unverified.

## Open questions for you
- Is the rotating platform wired to the robot controller (external axis), or a separate device?
- Grip widths in mm for short edges and long edges (jaw opening when seated on the plate).
- After the hotel is full: unload back, stop, or wait?
- Should the microscope be in the cell during route teaching, or a dummy holder at its position?

## Your standing instruction
Say "cycled" when the controller is back; I re-enable and open the jaw. Then insert the plate on its
SHORT edges, say "grip". Then: "pose microscope", "pose polar0", "pose polar90", then 3x "go"/"stop"
for the microscope->polar route, 3x for polar->hotel A1.
