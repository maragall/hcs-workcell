# Next steps, filled in (2026-10-19)

Each item: what it means here, what exists, what is missing, and the test that closes it.

## Vision

**Error detection.** Deciding, from an image, that a step did not end as intended: plate not seated on the
microscope, plate absent or skewed on the stage, slot occupied, plate shifted in the fingers.
- Exists: normalised cross-correlation gate (`lower_check.py`) with reference library tooling
  (`build_refs_from_runs.py`), threshold set from the references' own spread, biased to refusing.
- Missing: a camera that sees the seat. From the taught hovers neither gripper camera sees the plate
  (ANALYSIS section 9). Decision needed: fixed external camera on the microscope holder (recommended), or
  inspection poses 90 mm along the gripper axis. The stage-empty check can use the existing route point above
  the stage with no new hardware.
- Closes when: on 20 deliberately wrong placements (missing, skewed, lid on) the gate refuses every one, and on
  50 correct placements it passes at least 49 (false refusals cost a retry, false passes are not allowed).

**Error correction.** Acting on a detection without a human.
- Exists: bounded recovery table (`corrector.py`): one regrasp with fresh offset, three re-images at other LED
  levels, one re-enable, one stage retry, resume from the nearest waypoint, safe stop for everything else.
- Missing: ArUco offsets to correct a drifted fixture (markers not placed, hand-eye unverified); a
  "re-seat" action at the microscope (lift 10 mm, lower again, re-check) once the seated gate exists.
- Closes when: a fixture moved by 3 mm on purpose is corrected automatically for 10 cycles with no stop.

**Error prediction (tentative).** Warning before a failure.
- Exists: per-feature robust baselines (`predictor.py`), 492 cycles of logged features, measured spreads
  (jaw sd 0.001 to 0.007, waypoint deviation p90 0.83 mm).
- Missing: CUSUM on standardized residuals with an estimated cycles-to-threshold output; trained offline on
  the logged runs, then run live. First targets: jaw closed value, per-step durations, waypoint deviation,
  stage readback error.
- Closes when: on the logged data the predictor flags each of the seven SDK stops at least two waypoints
  early, with under one false warning per 50 cycles.

## Robot

**Stress test of the current script, random slot.**
- Exists: 492-cycle baseline at 20/10/5 %; `fast` 40/20/8 and `stress` 50/25/10 profiles; staged
  lowers and slides; 13-via-point route B; grasp tolerance 0.08; `run_matrix.py`.
- Missing: rows above 1 need the retreat-then-climb approach, which needs the rack front face and side
  taught by touch (`record-pose hotel.rack_front`, `hotel.rack_side`); A9 re-verified; random slot selection in
  `run_forever.py` (one flag).
- Order: 3 cycles fast with the operator watching, 3 cycles stress, then unattended stress over A1 and
  A3 to A9, then B and C.
- Closes when: 7 days unattended, zero safe stops, cycle time under 4 minutes.

**Turn to video-guided scripting.**
- Means: every lower, release and grasp is confirmed by a camera, and fixture poses are corrected from
  markers instead of being fixed numbers.
- Depends on the camera decision above, markers on the stage and holder, and the hand-eye check.
- Closes when: the loop runs with `--no-gates` and `--no-vision-correction` removed, for 100 cycles.

## Repo

**Build from the vendor GUI (tentative).** The vendor tool (VisionCalibrationTool) is compiled and its
preview handler on the Pi is broken; we drive the same vision box through its documented TCP API and store
taught positions in its own workspace (teach points, triggers, relations), so the GUI keeps working beside us.
Recommendation: do not build on the GUI; keep it as the vendor's calibration front end and use our CLI for
the cell. The proxy that fixes its preview stays available.

**Document and add a replication interface.**
- Exists: `docs/REPLICATION.md` (parts to unattended loop), `docs/VENDOR_SDK.md`, `DESIGN.md`,
  `ANALYSIS.md`, `README.md`; one CLI (`cli.py`) for status, teaching, grip measurement, routes, references,
  tests and runs; `layout.json` as the single site file.
- Missing: a guided `run.py commission` command that walks an operator through sections 4 to 8 of the
  replication guide with prompts and checks; a `layout.schema.json`; mp4 export of a reference run.
- Closes when: a person who has never seen the cell takes a fresh PC and the guide to a validated A1 cycle
  in one working day, without asking us anything.

## Analysis / ML

**Use the 42 GB of frames.**
- What they contain: 985 hotel approach views (slot face), 492 above-stage views (empty seat), every route
  point 492 times, no views of the plate at the seats.
- Use: slot occupied/empty classifier from the approach view; per-pose "normal scene" baselines for anomaly
  detection on the route; stage-empty gate references; mp4 material for mentors.
- Not usable for: microscope seated check, plate-in-gripper check.
- First deliverable: `build_refs_from_runs.py` already labels frames; add the slot-occupancy pair (place
  approach = empty, pick approach = occupied) and report separation the same way.

**Which models latch onto Bioyond's software.** Their box offers ArUco pose, pose offset against a stored
reference, live detection (marker or circle pair), barcode scan, NG event images, and the no-teach loop
(vision teach, fine tune). We plug in at: `get_pose_offset` for drift correction, `run_live_detection` for
slot occupancy, `scan` for plate identity, and their NG images as labelled failure data. Our own models sit
on top: the gates, the anomaly baselines, the predictors. Nothing of ours needs to run on their Pi.

## Additional

**Shorter routes.** Measured: route B is 617 degrees of joint travel and 240 mm of column; waypoint count
is not the cost, speed is (ANALYSIS section 7). Plan: fast profile, then stress, then 70 % travel in free
space; a geometrically direct route only after the rack and microscope obstacle envelopes are taught, since a
straight joint interpolation can cut through the rack depth.

**Meet Bioyond.** Bring: the broken calibration handlers on the Pi (`'str' object has no attribute
'sendall'`), the SDK completion flag that stops moves on gripper jitter, the undocumented port 52525 SDK
versus their documented BRCAPI, the hand-eye error figure of 827.8 on the Pi, how `MoveLinear` can be a
no-op, and whether the Lua-on-controller path (prepared in `lua/`) is the supported way for customers. Ask
for: register map or confirmation that the rotary platform is E2, gripper torque access, collision
sensitivity semantics.

**Figure out** the three undecided things: camera placement for the seated check; whether motion moves into
the controller's Lua program (removes the SDK fault class, enables blending and torque-based grasp detection)
before or after the stress test; and the microscope move, which re-teaches one seat, one hover and route B,
all through the existing commands.
