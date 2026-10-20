# Offline route optimisation study: microscope <-> rotating stage (route B) and stage -> hotel A1 (route A)

Date 2026-10-19 (evening). Everything here was computed from files only: `layout.json`, `runs/*/events.jsonl`, the
three drag traces in `runs/`, and the project's `nova_cell/safety.py`. No connection to the robot or the vision box.
Nothing under `runs/`, `layout.json` or `state.json` was modified. **None of the generated routes has been replayed on
the arm.** Section 7 lists what has to be verified before the first replay.

Files in `routes_opt/`:

| file | what |
|---|---|
| `fk.py` | `fk(joints) -> pose`, `ik(pose, elbow, ...) -> joints`, fitted constants embedded |
| `fk_model.json` | the fitted model and its residual table |
| `fit_fk.py` | reproduces the fit from the traces and the layout |
| `speed_model.py`, `speed_model.json` | AJ waypoint duration model from 53,924 logged moves |
| `route_gen.py` | constraint checker, candidate search, scoring; writes the files below |
| `microscope_to_polar_opt.json` | **recommended route B** (22 waypoints, layout-trajectory format + provenance) |
| `microscope_to_polar_opt_h16.json` | variant with a vertical 16 mm final approach onto the microscope hover |
| `microscope_to_polar_opt_overlap.json` | faster variant that turns the plate while descending above the stage (needs a rule relaxation) |
| `polar_to_hotel_opt.json` | route A: stage hover (90 deg) -> A1 approach point, 5 waypoints |
| `candidates_*.json` | every candidate with joints, poses, scores, clearances and violations |
| `install_route.py` | copies a route into `layout.json` under a new name (dry run unless `--yes`) |

## 1. Summary

* The SDK's joint order is **j1 = column (mm), j2 = shoulder, j3 = elbow, j4 = wrist**, not "shoulder, elbow, column,
  wrist" as the brief assumed. The data is unambiguous (section 2). Everything below uses the data's order.
* FK fitted from 4,068 logged joint/pose pairs: a 200 mm + 200 mm SCARA with its shoulder axis at (126.01, 0.01) in the
  robot frame, TCP on the wrist axis, z = j1 + 38.60 mm, yaw = j2 + j3 + j4. Held-out residuals: 0.04 mm rms xy on the
  route-A trace, 0.005 mm on the microscope teach pose, 0.008 mm on the two polar seats taught tonight. Good enough for
  everything that follows (the cycle's own pose band is 6 mm).
* Time per waypoint at the 50 % profile is **~0.9 s per 8-unit sub-move, almost independent of distance** (the arm is
  fast; `move_abs_joints_stepped` splits every waypoint into 8-unit SDK calls, each verified by polling). Route B's
  93.5 s one way (observed) = 101 sub-moves. Summed joint travel matters only through the sub-move count.
* Recommended route B (`east-strict-h0`): 22 waypoints, 60 sub-moves, **56 s one way at 50 % vs 93 s today** (88 vs
  149 s at 20 %); 315 deg of joint travel instead of 669; the plate never turns over the microscope; the turn happens at
  hover height above the stage, 18 mm beyond the zone box for the plate's swept circle; the descent is a near-vertical
  column move above the stage. Per cycle (two traversals) this is about 74 s of the current ~395 s stress cycle.
* A variant that turns the plate during the descent (`east-overlap-h0`, 14 waypoints, 37 sub-moves) is **35 s** one way,
  but it violates the brief's "turn above the microscope hover height" rule by construction. It is written out so the
  decision can be taken explicitly; it is not the recommendation.
* The 13-point `microscope_to_polar_fast` variant is not what it claims: its step 10 -> 11 is a 128.5 mm column move
  (`max_step_deg: 25.0` in the file is wrong), it would be refused by the 25-unit rule, and it would save only 15 %
  (79 s vs 93 s) because it still needs 84 sub-moves.
* Route A: a 5-waypoint route from the stage hover (90 deg) to the A1 approach point inside the corridor the cell
  already sweeps: 10.5 s vs 14.2 s (hover connector + taught route + the cycle's J move) at 50 %.
* Two findings that are not about routes but matter: the microscope seat is **0.8 mm inside the arm's reach limit**
  (r = 399.21 of 399.99 mm), and the seat taught tonight sits in the **mirror elbow branch** (j3 = -187.2, past full
  extension) while the hover and every route are in the other branch (section 3).

## 2. Forward kinematics fit

### 2.1 Which joint is which

| test | result |
|---|---|
| correlation of each joint with pose z | j1 1.000, j2 0.25, j3 0.05, j4 0.74 -> j1 is the column |
| `z - j1` over 5,078 samples | 38.599 mm, slope 0.99999 |
| `rz - (j2 + j3 + j4)` | mean -0.001 deg, sd 0.06, outliers only on fast drag samples |
| planar fit with angle sign +1 / -1 | rms 0.24 mm / 38 mm -> all angles positive, same sense as rz |

Route B's own numbers say the same: its first 8 waypoints keep j1 at 296.38 while z stays at 334.9; waypoints 24-32
change j1 by 128.5 while z drops by 128.2 mm; j4 goes 178.6 -> -4.5 while the yaw turns 183 deg.

### 2.2 Model and fitted values (`fk_model.json`)

```
x = x0 + l1 cos(j2 + a1) + l2 cos(j2 + j3 + a2)      x0 = 126.012 mm   l1 = 199.991 mm  a1 = -0.001 deg
y = y0 + l1 sin(j2 + a1) + l2 sin(j2 + j3 + a2)      y0 =   0.010 mm   l2 = 199.995 mm  a2 = -179.997 deg
z = z_scale j1 + z_offset                            z_scale = 0.99999   z_offset = 38.599 mm
rz = j2 + j3 + j4 + yaw_offset                        yaw_offset = 0.000 deg
tool offset along the yaw: fitted 0.006 mm -> 0      (the reported pose is the wrist axis)
```
Because a2 = -180 deg, j3 = -180 is the fully extended arm and the reach is r = 400 |sin(j3/2)| from the shoulder axis.
Nominally the arm is 200 + 200 mm with its base 126 mm behind the robot-frame origin in x.

Fit set: `trace_microscope_to_polar_01` (884 samples) + `discarded_trace_polar_to_hotel_01` (3,184); one robust pass
dropped 12 samples with > 1 mm residual (read lag while the arm was being dragged fast). The run logs' AJ records carry
no joints or poses (only `dt`, `dev`), so they could not be used; the drag traces, the layout waypoints and the teach
pairs are the only joint/pose pairs on disk.

### 2.3 Residuals

| set | role | n | xy rms | xy p95 | xy max | z rms | z max | yaw rms | yaw max |
|---|---|---|---|---|---|---|---|---|---|
| trace_microscope_to_polar_01 | fit | 884 | 0.087 | 0.009 | 0.930 | 0.039 | 0.670 | 0.033 | 0.500 |
| discarded_trace_polar_to_hotel_01 | fit | 3184 | 0.263 | 0.012 | 13.5 (lag) | 0.073 | 3.9 (lag) | 0.075 | 3.9 (lag) |
| trace_polar_to_hotel_01 | **held out** | 941 | 0.040 | 0.009 | 0.415 | 0.022 | 0.421 | 0.012 | 0.180 |
| layout microscope_to_polar (39 wp) | held out | 39 | 0.528 | 0.870 | 2.029 | 0.074 | 0.339 | 0.029 | 0.089 |
| layout polar_to_hotel (5 wp) | held out | 5 | 0.042 | 0.068 | 0.072 | 0.051 | 0.114 | 0.001 | 0.002 |
| layout polar_to_hotel_demo_full (11 wp) | held out | 11 | 0.062 | 0.125 | 0.145 | 0.034 | 0.114 | 0.003 | 0.010 |
| microscope joints_at_teach (20:42 tonight) | held out | 1 | 0.005 | | | 0.001 | | 0.000 | |
| polar pose_at_0 joints_at_teach (20:53 tonight) | held out, after the fit | 1 | 0.004 | | | 0.001 | | 0.000 | |
| polar pose_at_90 joints_at_teach (21:04 tonight) | held out, after the fit | 1 | 0.008 | | | 0.002 | | 0.001 | |

Units mm and deg. The p95 of a few hundredths of a mm with a larger rms shows the error is a handful of lag outliers,
not model error. The 39-waypoint layout route is off by up to 2 mm because `trajectory.normalise` smooths joints and
poses separately (a known property of that file, not of the arm). `ik(fk(q))` reproduces all 5,078 samples exactly.

**Verdict: the FK is confidently determined.** Anything below 1 mm is irrelevant for route planning; the cycle checks
waypoints against a 6 mm band.

## 3. Reach and elbow-branch findings at the microscope

* Microscope seat (124.03, 399.21): r = 399.21 mm from the shoulder axis; the reach is 399.99 mm. The seat is at the
  edge of the workspace. Consequence: a vision correction of more than +0.8 mm radially (roughly +y) at the microscope
  is unreachable and the controller will refuse it; at this radius 1 mm of radial error corresponds to 7 deg of elbow.
  This also explains the logged "no motion on a 17.1 mm linear move" fallbacks at the microscope hover: linear moves
  next to the extension singularity are where controllers give up.
* The seat was taught tonight at joints (226.6, 93.87, **-187.17**, 183.64): j3 is 7.2 deg past full extension, i.e. the
  mirror elbow configuration. The hover (taught 10-14 at j3 = -177.6) and every taught route are in the other branch
  (j3 in (-180, 0)). The same seat pose in the route's branch is (226.6, 86.71, -172.86, 176.48): identical TCP and yaw,
  elbow displaced by about 25 mm. The cycle lowers from the hover with a linear move; which branch the controller uses
  for that move, and whether the elbow clears the microscope body in both, must be verified at 1 % (section 7).
  The generated routes keep the taught branch (j3 in (-180, 0), minimum margin 7 deg from extension at the hover).

## 4. Duration model for waypoint moves (`speed_model.json`)

How a waypoint executes (`cycle._waypoints` -> `robot.move_abs_joints_stepped`, max_step 8): the largest joint delta d
(mm for j1, deg for the others) is split into `nsub = int(d/8 + 0.999)` equal sub-moves, each a separate
`MoveAbsJointAsync` verified by 0.1 s joint polling, then one `_wait_still` and a pose read. The log gives `dt` per
waypoint; the deltas come from the layout trajectory (unchanged since 10-15 02:04) and the direction from the preceding
lift step. 53,924 AJ records: 41,854 at the 20 % profile (10-15 .. 10-18, 10-19 02:17) and 12,070 at 50 % (10-19 from
02:28; identified from the approach speed logged on the linear moves, 0.1 vs 0.25).

Model per sub-move: `t = t0 + max(dz/v_z, d23/v_23, d4/v_4)`, fitted on the 85 per-waypoint medians of each profile
(robust to the SDK MotionStop re-issues):

| profile | t0 | v_z | v_23 | v_4 | rms on medians | route B one way modelled / observed | route A modelled / observed |
|---|---|---|---|---|---|---|---|
| 50 % (stress) | 0.85 s | 101 mm/s | 70 deg/s | 83 deg/s | 0.05 s | 93.3 / 93.5 s | 7.6 / 7.4 s |
| 20 % | 0.59 s | 12.4 mm/s | 5.3 deg/s | 6.4 deg/s | 0.38 s | 148.6 / 148.4 s | 13.6 / 13.6 s |

The "speeds" are effective rates for 4-8 unit sub-moves that never reach cruise speed; do not read them as axis
velocities. The zero-motion AJ call (route B waypoint 1, reached from the hover) takes 0.90-0.92 s at both profiles.
What the data says plainly: **at 50 % a sub-move costs ~0.9 s whatever its length (2.66 s for 16 mm of column in 3
sub-moves; 2.78 s for 16 deg of wrist in 3 sub-moves; 1.85 s for any 2-sub-move waypoint).** Route time is therefore
`0.9 s x sum(ceil(d_i / 8))` plus small motion terms; the only way to be faster with the current replay code is fewer
sub-moves, which means less joint travel and moving several axes in the same segments. The model is calibrated on
exactly the sub-move sizes the generated routes use (<= 8 units), so no extrapolation is involved in the predictions
below. Its validity at 10 % (the first replay speed) is untested; expect it to be slower than the 20 % column.

Where route B's 101 sub-moves go today: 30 for the column (240 mm done as separate waypoints), 23 for the 183 deg wrist
turn, the rest for the 210 deg of shoulder+elbow swing, almost nothing overlapped.

## 5. Constraints as implemented (`route_gen.py`)

Each waypoint and every joint-space sample between waypoints (0.5 unit spacing) is checked:

1. TCP inside `workspace_box`; TCP above the z floor of its region (same semantics as `safety.py`).
2. Elbow in the taught branch (j3 in (-180, 0)); joints inside the taught envelope: j2 [-18, 99], j3 [-179, -20],
   j4 [-40, **184**], j1 [20, 340] (j1 335 is proven by hotel A9). The j4 limit is unknown; 183.6 is the largest value
   ever logged (the seat teach), so 184 is a cap, not a known limit.
3. Microscope rule, on the **plate rectangle** (128 x 85 mm around the TCP; long axis along the gripper yaw for the
   short-edge grip, across it for the long-edge grip; see the assumptions below): no part of the plate may be over the
   microscope region box (x 60..220, y 250..450) below the hover height 331.9 mm, except the vertical column within 2 mm
   of the seat; while any part of the plate is over the box the yaw must equal the seat yaw within 0.5 deg and z must be
   at or above the hover height. This is the brief's "outside the zone except purely vertical motion above the seat",
   read as a volume: the zone box below hover height is forbidden, lateral motion at or above hover height is allowed
   (it is what the taught route does and what PLACEMENT.md's "free corridor z 300..335" describes).
4. Yaw turn ("strict" candidates): any segment that changes the yaw must stay at or above hover height, and the plate's
   swept circle (radius 76.8 mm) must clear the zone box by >= 15 mm everywhere along it. 15 mm is the most a turn that
   ends above the re-taught stage seat (x 312.5) can have (312.5 - 76.8 - 220 = 15.7); the recommended route turns in
   place at (315, 290) and gets 18.2 mm, then drifts <= 15 mm laterally during the descent.
5. Waypoint step <= 24 units (limit 25; 24 is exactly three 8-unit sub-moves, so no sub-move is wasted on rounding).
6. Route A only: the plate must stay inside the region swept by the taught route A (984 traversals without contact)
   plus the two legs the cycle executes around it: the J move from the current stage hover to the route's first waypoint
   (`cycle._arrive`; the hover moved 4.3 mm when the seat was re-taught at 21:22, so this leg is what the cell will sweep
   at its next run anyway, not yet proven) and the J move from the last waypoint to the approach point; all dilated
   5 mm. The rack geometry is not in `layout.json`, so this "proven corridor" is the only defensible model; the microscope rule for route A is applied to the TCP only, because the
   taught route A already sweeps plate corners to x 183 at z 103 every cycle (the box is a TCP proxy there).

Assumptions that are not in the data and need a look (section 7): plate orientation per grip (inferred from the rack
column pitch of 130-137 mm, the camera notes in ANALYSIS.md section 9 and the jaw-width ratio); plate centre on the TCP
(the layout's hotel box is drawn that way; the fitted tool offset is 0, so if the fingers hold the plate ahead of the
wrist axis every plate-clearance number shifts by that offset along the yaw; `Rules.plate_fwd_offset_mm` exists for it).

## 6. Candidates and scores

Route B, microscope hover (124.03, 399.21, 331.89, yaw 90.33) -> stage hover (312.51, 304.53, 101.21, yaw -91.26), both
the `seat + hover_mm` of the fixtures as of 21:22 tonight (microscope taught 20:42, polar pose_at_0 20:53). "lat<hov" = minimum distance of the plate rectangle to the
zone box over all samples below hover height; "tcp<hov" the same for the TCP (safety.py's view); "turn clr" = TCP
distance to the box minus the swept radius while the yaw changes (negative = turning over the box).

| candidate | turn point | wp | joint travel deg | column mm | sub-moves | 50 % s | 20 % s | lat<hov mm | tcp<hov mm | turn clr mm | rule status |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **east-strict-h0 (recommended)** | 315, 290 | 22 | 314.5 | 230.7 | 60 | **56.3** | 88.2 | 48.8 | 92.5 | 18.2 | all rules OK |
| east-strict-h16 (vertical 16 mm arrival) | 315, 290 | 24 | 314.5 | 262.7 | 64 | 60.0 | 92.6 | 48.8 | 92.5 | 18.2 | all rules OK |
| east-strict-*-viaT (swing via the taught point 186, 353) | 315, 290 | +1 | 340.5 | | +2 | +1.9 | +3.6 | 48.8 | 92.5 | 18.2 | all rules OK |
| east-overlap-h0 (turn during the descent) | 315, 290 | 14 | 311.2 | 230.7 | 37 | **34.6** | 61.4 | 16.6 | 92.5 | 15.7 | turn below hover height: **needs the rule relaxed** |
| east-overlap-h16 | 315, 290 | 16 | 311.2 | 262.7 | 41 | 38.3 | 65.4 | 16.6 | 92.5 | 15.7 | same |
| taught corridor, re-segmented (turn at 160, 80 as today) | 160, 80 | 27 | 603.7 | 230.7 | 72 | 68.0 | 119.2 | 45.6 | 92.5 | 28.9 | all rules OK |
| current `microscope_to_polar` (39 wp) | | 39 | 669.2 | 240.8 | 101 | 93.3 (obs. 93.5) | 148.6 (obs. 148.4) | 16.2 | 88.1 | -76.8 | turns 48 deg over the zone (wp 1-7, z 335) |
| current `microscope_to_polar_fast` (13 wp) | | 13 | 652.3 | 240.8 | 84 | 78.8 | 132.3 | 0.2 | 73.3 | -76.8 | step 128.5 > 25; same turn over the zone |

Recommended route, leg by leg (50 % / 20 %):

| leg | what | joint deltas j1 mm, j2, j3, j4 deg | waypoints | sub-moves | time |
|---|---|---|---|---|---|
| 1 | swing from the hover to the turn point at hover height, yaw held at 90.33 (j4 compensates) | 0, 59.9, 53.0, 6.9 | 3 | 8 | 7.7 / 15.6 s |
| 2 | wrist turn in place at (315, 290, 331.9): 90.33 -> -91.26 | 0, 0, 0, 181.6 | 8 | 23 | 21.7 / 40.6 s |
| 3 | descent to the stage hover, plate aligned, 15 mm lateral drift | 230.7, 4.9, 6.6, 1.7 | 10 | 29 | 26.9 / 32.1 s |

Waypoints (j1 mm, j2..j4 deg -> x, y, z, yaw): 1 (293.3, 86.71, -172.86, 176.48 -> 124.0, 399.2, 331.9, 90.3);
2 (293.3, 62.71, -151.61, 179.24 -> 213.9, 377.7, 331.9, 90.3); 3 (293.3, 38.71, -130.37, 181.99 -> 287.8, 325.0, 331.9,
90.3); 4 (293.3, 26.83, -119.85, **183.36** -> 315.0, 290.0, 331.9, 90.3); 5-12 wrist only, 24 deg per waypoint, to
(293.3, 26.83, -119.85, 1.76 -> 315.0, 290.0, 331.9, -91.3); 13-22 column 24 mm per waypoint to (62.61, 31.74,
-126.45, 3.45 -> 312.5, 304.5, 101.2, -91.3). Joint ranges used: j1 62.6..293.3, j2 26.8..86.7, j3 -172.9..-119.9,
j4 1.8..183.4. All 22 poses pass `nova_cell.safety.Safety.check_target` (run offline). The route is symmetric, so the
reverse replay (stage -> microscope) is the same list reversed, exactly as the cycle does today.

Route A, stage hover at 90 deg (312.41, 305.83, 99.97, yaw -91.09; pose_at_90 taught 21:21:58) -> A1 approach
(193.34, 137.17, 99.63, yaw -1.00):

| candidate | via | wp | joint travel deg | sub-moves | 50 % s | 20 % s | status |
|---|---|---|---|---|---|---|---|
| direct straight joint move | | 5 | 161.9 | 11 | 10.5 | 21.4 | leaves the proven corridor around (303, 296): rejected |
| **one-via (recommended)** | 255, 255, yaw -45 | 5 | 161.9 | 11 | 10.5 | 21.4 | inside the proven corridor, TCP >= 28 mm from the microscope box |
| current: hover connector + `polar_to_hotel` + the cycle's J move | | 1 + 5 + 1 | 162.3 | 15 | 14.2 (route alone obs. 7.4, J move obs. 4.1) | 25.7 | defines the corridor |

Waypoints of the recommended route A (j1 mm, j2..j4 deg -> x, y, z, yaw): 1 (61.38, 32.20, -127.12, 3.83 -> 312.4,
305.8, 100.0, -91.1); 2 (61.38, 23.22, -103.12, 19.60 -> 274.7, 275.8, 100.0, -60.3); 3 (61.38, 18.76, -91.20, 27.43 ->
255.0, 255.0, 100.0, -45.0); 4 (61.20, 7.12, -67.20, 37.90 -> 224.7, 198.1, 99.8, -22.2); 5 (61.03, -3.69, -44.92, 47.61
-> 193.3, 137.2, 99.6, -1.0). It ends exactly at the approach point, so the cycle's J move there becomes a no-op, and it
starts at the hover instead of 15 mm below it (the taught route's first waypoint is at z 84.8, below the 100 mm hover, so
the cycle currently dips back toward the stage after lifting). Saving about 3.7 s per traversal, 7 s per cycle.

Not done, worth noting: the column descent (leg 3, 29 sub-moves, 27 s) is the kind of move the cycle already does as a
single linear move elsewhere ("polar clear", 60 mm in 1.0 s at 25 %). If `_waypoints` or the cycle were allowed to do a
pure-z leg as one `L` move, route B would be about 33 s one way with every rule intact; similarly a larger sub-move
(the layout limit is 25, the replay uses 8) would cut the wrist turn from 23 calls to 8. Both are code changes to
`cycle.py`/`robot.py` and were outside this study.

## 7. What must be verified on the real arm before any use

Prerequisites (no plate, nothing moving):
1. Read the controller's joint limits, in particular j4. The route reaches j4 = 183.36 at the turn point; 183.6 is the
   largest value ever logged. If the limit is lower, the turn point must move south (lower j4): `JOINT_ENVELOPE["j4"]`
   in `route_gen.py`, then re-run. The controller refusing waypoint 4 is the symptom.
2. Confirm the plate orientation assumption: with the short-edge grip the plate's long axis runs along the gripper yaw
   (pointing away from the arm). Measure the offset from the j4 axis to the plate centre; if it is not ~0, set
   `Rules.plate_fwd_offset_mm` and re-run (the swept-circle clearance of 18 mm at the turn point is the number at risk).
3. Look at the two volumes the new route uses that no taught route has used: the column above the stage from z 164 (the
   cycle's "polar clear" height) up to 332 at (313-315, 290-305) with the plate aligned with the stage, and the band
   x 186..262, y 325..378 at z 332 (plate height 332 +- plate thickness) east of the microscope where the swing leaves
   the zone box. PLACEMENT.md says the stage-microscope corridor is free at z 300..335; check that it also holds for the
   128 mm plate 35 mm further east than the taught route's sweep (which reached x 228 at y 353).
4. Collision sensitivity at maximum on the pendant (still an outstanding item in the operator list).
5. Decide the elbow-branch question of section 3 at 1 %: lower from the hover to the seat, read the joints, compare
   j3 with -187.2 (taught) and -172.9 (route branch), and watch the elbow against the microscope body.
6. The fixtures moved three times while this study ran (microscope 20:42, polar pose_at_0 20:53, pose_at_90 21:04
   and again 21:22; the intermediate pose_at_90 was 10 mm off pose_at_0, the final one agrees within 1.3 mm and
   0.17 deg). Route A starts from pose_at_90 + hover, route B ends at pose_at_0 + hover; the files on disk match the
   layout as of 21:22. After any further re-teach, re-run `route_gen.py` (it reads `layout.json` at run time, ~70 s)
   before installing anything.

First replay (plate not held, then held):
7. Install under a new name, leaving the taught route untouched:
   `python routes_opt/install_route.py microscope_to_polar_opt.json --name microscope_to_polar_opt --yes`
8. Drive the arm to the microscope hover (within 15 mm of waypoint 1, the replay's start tolerance), then
   `python run.py replay microscope_to_polar_opt --speed 0.10 --step --max-dev 2`. Confirm each waypoint by hand; the
   printed pose deviation must stay under 1 mm (the FK predicts the pose to 0.1 mm, so a larger deviation means the arm
   did not go where the joints say, or the model is wrong for that pose; stop and report). Watch legs 1 and 2 for the
   plate's clearance to the microscope side and leg 3 for anything above the stage.
9. Reverse from the stage hover: `python run.py replay microscope_to_polar_opt --reverse --speed 0.10 --step --max-dev 2`.
   This is the direction that ends at the microscope hover; confirm the arm arrives at hover height with the seat yaw
   and that the subsequent vertical lower is unchanged.
10. Repeat 8-9 with the plate held on its short edges (jaw inside the 0.08 band before and after).
11. Compare the per-waypoint `dt` in `runs/teach/events.jsonl` (replay) and later `runs/<run>/events.jsonl` with the
    model: expect ~0.9 s per sub-move at 50 % (2.8 s for a 24-unit waypoint, 1.85 s for a 16-unit one). A waypoint
    taking much longer than predicted means the controller is doing something the model does not know about.
12. Then the order in ANALYSIS.md section 7: 3 supervised cycles at the fast profile with gates on, 3 at stress, then
    unattended, each gating on zero safe stops and jaw values inside the band. Only then retire the 39-point route.
13. The overlapped variant (`microscope_to_polar_opt_overlap.json`) needs an explicit decision that turning the plate
    while descending above the stage, 173 mm from the microscope seat and 15.7 mm (swept circle) outside the zone box,
    is acceptable; until then it is a number, not a route.
14. Route A (`polar_to_hotel_opt.json`): replay from the stage hover at 90 deg with the plate on its long edges, forward
    and reverse, at 10 %, watching the rack's +y side; the proven-corridor rule only says the plate stays where the taught
    route (and the cycle's connector moves around it) already goes. The one-via point (255, 255) at yaw -45 is 28 mm from
    the microscope zone box (TCP) and its plate corners reach x 309 at y 300, which the taught route's sweep covers.

## 8. Reproduce

```
python routes_opt/fit_fk.py        # fk_model.json + residual table
python routes_opt/speed_model.py   # speed_model.json (reads runs/*/events.jsonl, ~5 s)
python routes_opt/route_gen.py     # candidates + the four route files (~70 s)
```
