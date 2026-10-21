# Incidents

Every contact or near-contact, what the software did, root cause, fix. Newest first.

## 2026-10-20 22:25 wrist collision stop on route B back to the microscope, plate held

- Sequence: cycle 682 of the fast-profile loop (40/20/8 %), 42nd cycle of the day, slot A9. Microscope pick 22.48,
  stage 90 pick 1.64, A9 place and pick 1.55 (held-plate front check scored 0.33 against the references, the first
  miss of the day, shadow only), place at 90, turn, pick at 0 22.61 (held-plate check 0.95), reverse route B. At
  point 36 of 39 (the segment from (183, 321) to (186, 353) at z 335, yaw 114 to 105) the controller reported
  "axis 4 collision detected" twice within 15 ms and dropped all servos. The arm stopped 4 mm into the segment at
  (181.4, 324.9, 334.9) yaw 114.0; the wrist readback 179.2 against a target of 178.59, i.e. pushed past the target.
  Jaw 23.0 with the servo off (closed at 22.61). Same segment had run clean 41 times today.
- Software behaviour: the route replay raised MotionRefused ("abs joint move did not reach"); the supervisor
  classified that text as recoverable and scheduled a relaunch in 60 s. Killed by hand within the minute. Fixes: the
  cycle now asks the controller for its error list before naming a refused move (`_refused`), and the supervisor
  refuses to relaunch while the controller holds any error (`controller_errors()` in run_forever.py).
- Cause (operator, 22:40): the microscope's brightfield illumination had been set from the top, so the transmitted-
  light illuminator arm stood in the route B corridor above the stage. The plate hit it. Nothing in the software
  could have seen it: the route is a fixed joint replay and both gripper cameras look away from the plate at that
  pose. Logged as tech debt (docs/TECH_DEBT.md): a brightfield mode needs its own approach to the microscope.
- Arm left where it stopped, servos off, STOP file set, supervisor dead, until the operator confirms the plate.

## 2026-10-19 23:08 wrist collision stop at the stage, short-edge pick at 0

- Sequence: first full cycle with the production plate. Microscope pick 22.51, route B, place at polar 0, rotate to
  90, long-edge pick (2.21, inside the acceptance band), route A, A1 place and pick (2.46), back to the stage, place
  at 90, rotate to 0, short-edge pick: during the descent from the hover the controller reported a joint-4 collision
  at z 72.8 (6 mm above the seat) and disabled all servos. Yaw had been pushed from -91.26 to -93.59; the jaw read 19.2
  with the servos off and worked normally (26 / 22 / 26) once re-enabled.
- What the fingers hit: the plate on the stage. The fingers open 26 for the short-edge grip, 1.8 mm of clearance per
  side on a 125 mm plate. The plate's position at 0, after the robot's own place at 90 and the stage turn, differs from
  the hand-taught `pose_at_0` by more than that. The two hand teaches (0 and 90) and the nest's slop with this smaller
  plate (127.5 over the skirt, the teaching plate was 128) are the suspects. Not measured yet: a top view of the plate
  in the nest at 0.
- Recovery: cleared, enabled, lifted straight up to the clear height, jaw tested, frames taken. Plate left on the stage.
  Arm parked above it at z 161, jaw open, enabled.
- Decision: no further automated stage picks until the plate position in the nest is measured (see plan below).

## 2026-10-19 22:27 to 22:57 wrist collision stop at the stage, my bug

- The tactile centering search tried yaw offsets at the stage. The wrist did not follow (yaw target -89.95, actual
  -91.3) and my yaw-splitting loop kept re-issuing small moves computed from the readback. 1,177 moves over 30 minutes;
  the z readback crept 4.2 mm below the seat; the controller's wrist collision detection finally stopped it.
- Root causes: linear moves verified x/y/z only, so a refused yaw passed as reached; the loop had no bound and took its
  next target from the readback instead of the goal.
- Fixes (commit 7b273ae): every linear move verifies yaw within 0.5 deg and stops on deviation; the split is a fixed
  count of parts targeted from the goal pose; yaw removed from the stage search; search loops bounded by their trial list.
- Also found: the stage had been pushed to 91.84 by the contact; the start-up sync re-drove it to 90.

## 2026-10-15 02:37 A3 pre-approach climb inside the shelf depth (no contact)

- Joint move from the route end to the A3 pre-approach climbed while the plate was already over the shelf; stopped on
  height. Fix: rows above 1 retreat to `climb_x` and climb with the column only (2026-10-19).

## Plan for the stage pick at 0

Status 2026-10-20 17:20: the jaw's end stop is 29.28, so the stage's short-edge pick and place now open 29
(`fixtures.polar.open_width`), 3.3 mm of clearance per side instead of 1.8. First full cycle after that: descent at 0
clean, grasp 22.61 (nominal 22.40, band 22.3 to 22.9). The measurement below is still worth doing if the widths at 0
drift toward the band edge in OVERNIGHT.md; a 22.9 reading means the plate is 0.25 mm off along the closing axis.

1. Measure where the plate actually sits at 0 after a robot place at 90 and a stage turn: top view with the down
   camera (camera over the nest needs the TCP about 90 mm toward +y, which is outside reach at this x; turn the
   gripper 180 deg at the clear height instead, or use the front camera at a known offset), or by touch: close the
   fingers in 0.2 mm steps from the hover at increasing depths and record the first contact height and width.
2. Compare with `pose_at_0`; if the offset is a constant (nest geometry), fold it into the pose. If it varies cycle to
   cycle (slop), the pick at 0 needs a per-cycle vision fix before the descent, which is the vendor ArUco offset path
   or our own top view.
3. Until then the cycle can run microscope <-> stage at 0 only (no stage turn), or hotel <-> stage at 90 only.
