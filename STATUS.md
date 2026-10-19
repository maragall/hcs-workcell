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
