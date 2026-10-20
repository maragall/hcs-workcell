# Incidents

Every contact or near-contact, what the software did, root cause, fix. Newest first.

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

1. Measure where the plate actually sits at 0 after a robot place at 90 and a stage turn: top view with the down
   camera (camera over the nest needs the TCP about 90 mm toward +y, which is outside reach at this x; turn the
   gripper 180 deg at the clear height instead, or use the front camera at a known offset), or by touch: close the
   fingers in 0.2 mm steps from the hover at increasing depths and record the first contact height and width.
2. Compare with `pose_at_0`; if the offset is a constant (nest geometry), fold it into the pose. If it varies cycle to
   cycle (slop), the pick at 0 needs a per-cycle vision fix before the descent, which is the vendor ArUco offset path
   or our own top view.
3. Until then the cycle can run microscope <-> stage at 0 only (no stage turn), or hotel <-> stage at 90 only.
