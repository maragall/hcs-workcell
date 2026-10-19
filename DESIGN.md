# nova_cell design

## 1. Cross-correlation gate: what it is and what it is not

It is a go / no-go check, not registration. Before the gripper descends at a fixture, the down camera
takes a frame from the hover pose and correlates it with reference frames recorded at that same pose
when lowering was known to be safe. The same mechanism answers three questions:

| Gate | Fixture state compared | Blocks |
|---|---|---|
| safe to lower before a pick | `plate_present` | plate missing, lid on, hand, wrong stage angle |
| safe to lower before a place | `empty` | holder occupied, stage not at its known position |
| seated after a release | `plate_present` | plate tilted or off the holder: the microscope must not scan |
| held after a lift (front camera) | `gripper/<grip>` | plate slipped, grabbed on the wrong edges |

Tuned against false negatives: a threshold derived from the references' own spread, a hard floor,
fewer than three references means "unsafe", any doubt means "unsafe". The cost of a false positive is
one re-image with a different LED level, then an operator look in supervised mode or a safe stop
unsupervised.

## 2. How damage to the microscope is prevented

No single layer is a guarantee. Five independent ones have to fail together:

1. **Controller collision sensitivity at maximum** (pendant / HMI). Hardware stop, independent of our code.
2. **Geometric exclusion** in software: a per-region z floor and a workspace box. The code refuses to
   command any target, or any 5 mm sample of a linear segment, below the microscope floor. The floor
   may be crossed only by the microscope lower step itself, and only after gate 1 passed.
3. **Vision gate** before every descent (section 1).
4. **Speed**: contact moves at 1 %, approach at 1.5 to 2 %, travel at 3 to 5 %, in both profiles.
5. **Pose deviation stop**: after every move the reported pose must be within 1 mm of the target.
   A stall against an obstacle or a collision stop leaves the arm short of target and the cycle stops.

What remains uncovered: a plate that is held off-centre by a few millimetres and clips the holder on
the way down. Mitigation is the held-plate check after each lift and the seated check after release.
A force or compliance sensor on the gripper would close this; the gripper has none.

## 3. How snaps are prevented

- The cycle never calls SwitchMode or EnableDragMode. Both drive the gripper axis at full speed.
- Gripper moves use MoveSingleExtJoint only. Closing is stepped, 0.5 per 0.15 s. Opening is one move.
- No raw drag trace is ever replayed. Trajectories are averaged, smoothed, thinned, and every waypoint
  must be within 25 degrees of the previous one and inside the box.
- Motion calls are never retried. A refusal stops the cycle with state on disk.

## 4. Tolerance model

| Quantity | Source of error | Allowed | Detected by |
|---|---|---|---|
| fixture pose drift | stage moved, marker moved | 25 mm correction, applied in x,y | get_pose_offset vs reference |
| grasp | plate shifted, wrong edge | jaw within 0.35 of 2.92 | jaw end value, held-plate image |
| placement | holder lip, friction | seated check | cross-correlation `plate_present` |
| trajectory repeat | controller | 1 mm at every waypoint | pose deviation |
| slow drift over days | wear, temperature, lighting | z-score 3.5 warn, 6 stop | DriftMonitor |

## 5. Live data contract (what is stored and what the predictor consumes)

One JSONL record per event in `runs/<run>/events.jsonl`, images in `images/c<cycle>/`.

Per move: `step, kind, target[6], actual[6], dt, speed, dev`
Per jaw action: `open|close, jaw, curve[]`
Per vision call: `offset d[6], mag, frame` or `lower_check score, thr, refs, safe` or `seated_check`, `plate_in_gripper`, `slot_view ready`, `barcode`
Per cycle: `dt, state`; per health check: `robot_enabled, drag, vision state, clients`
Per fault: `step, kind, detail` (also in state.json)
Images: ArUco view at every correction, lower-check frame at every attempt, front view after every
lift, down view after every release, slot view before every hotel place, barcode frame.

Live features fed to the predictor after each step, by name:
`dt:<step>`, `jaw:closed`, `vision_mag:<step>`, `lower:<fixture>:<state>`, `gripper:<grip>`, `dt:cycle`.
Level 2 trains on exactly these keys. Nothing else has to change for a trained model to replace
`DriftMonitor`.

## 6. Physical failure cases and the response

| Case | Detected by | Response |
|---|---|---|
| plate missing at pick | lower gate (`plate_present`) | re-image x3, then stop / ask |
| plate present where it should be empty | lower gate (`empty`) | stop / ask |
| grasp missed | jaw < 2.57 | open, lift, re-vision, one retry, then stop |
| grasp on obstruction | jaw > 3.27 | same as above |
| plate slipped in gripper | held-plate image, jaw drift | stop / ask |
| plate not seated after release | seated check | stop / ask; microscope must not scan |
| fixture or marker moved | vision magnitude > 25 mm | stop |
| ArUco not found | VisionError | ask in supervised, stop unsupervised |
| stage did not rotate | polar feedback DI, lower gate at the other angle | one retry, then stop |
| collision or stall | pose deviation > 1 mm, controller stop | stop |
| controller lost enable | state check | one re-enable, no motion retry |
| drag mode switched on by someone | health check | stop |
| vision box down | status call fails | stop |
| hotel slot occupied | two-class correlation at the approach (references the cell collects itself: place approach = empty, pick approach = occupied, spread over the day), vendor live detection as fallback | ask / stop |
| hotel slot empty before a pick | same check, opposite class | log (stop once enforced) |
| wrong plate | barcode mismatch (when ids are used) | log, continue, flag |
| power loss mid-step | state.json step_done false | refuses to resume until inspected |
| lighting change | lower-check scores drift | warn, then stop |

Cases still open: a lid left on a plate (needs its own reference state), a plate rotated 180 degrees
in the holder (needs the ArUco marker to be asymmetric), two plates stacked.

## 7. Test design

Each level gates the next.

1. **Offline**: `tests/test_sim.py` against the simulator and a fake vision box. Runs in CI.
2. **Gate calibration**: at each fixture, record 5 safe references and 5 deliberately unsafe scenes
   (plate missing, plate skewed, lid on, hand in view). Require every unsafe scene to score below
   threshold and every safe one above. Adjust margin or ROI until true.
3. **Fixture acceptance, supervised, 1 %**: 10 pick-and-place repetitions at each fixture with the
   plate re-seated by hand each time. Record jaw values, vision offsets, pose deviations. Pass:
   10/10 seated checks pass, jaw spread under 0.15, no deviation over 0.5 mm.
4. **Route acceptance**: 10 traversals of each trajectory with the plate held. Pass: no refusal,
   waypoint deviation under 1 mm.
5. **Supervised loop**: 20 full cycles with an operator watching. Pass: zero StopSafe.
6. **Unsupervised overnight**: 8 hours, real polar driver, nobody present. Pass: zero alerts.
7. **Seven days**: the level 1 target. Any StopSafe counts as a failure and gets a root cause.

## 8. Where the engineering hours go

In order of risk retired per hour: controller collision sensitivity and z floors (minutes, retires
the catastrophic case); reference images and the gate calibration test (an afternoon); trajectories
(an afternoon); polar stage interface (unknown, blocks unsupervised mode); hand-eye verification
(half a day, blocks vision correction). Deep learning and 3D sensing come only after the loop has
produced a week of logs.
