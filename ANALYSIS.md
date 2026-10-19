# Analysis of the first unattended run (2026-10-15 03:01 to 2026-10-18 21:23)

Source: `runs/analysis.json` (from `analyze_runs.py`), `runs/forever.log`, `state.json`.

## 1. What happened

| Quantity | Value |
|---|---|
| wall time | 90.4 h |
| cycles completed (microscope -> A1 -> microscope) | 492 |
| median cycle time | 660 s (p10 653, p90 664) |
| moves executed | 61,085 |
| grasps | 1,971, all within tolerance |
| safe stops | 8: seven SDK-induced short stops on route B, one disk-full stop |
| supervisor relaunches | 7, all recovered by resuming on the route |
| final state | plate held on its long edges above the rotating stage, step `place:polar`, arm disabled by the stop; nothing damaged |

No contact event. No grasp failure. The run ended because the video recording filled the disk (54.8 GB to 5.0 GB in 90 h, about 13 GB/day).

## 2. Where the time goes (per cycle, 660 s)

| Family | s | share | note |
|---|---|---|---|
| route B, microscope <-> stage, both directions | 302 | 46 % | 39 waypoints each way, about 4 s per waypoint regardless of distance |
| hotel place at A1 | 56 | 8 % | approach joint move 22 s, slide in 100 mm at 5 % = 21 s |
| route A, stage <-> hotel, both directions | 40 | 6 % | 5 waypoints each way |
| hotel pick at A1 | 34 | 5 % | slide in 21 s |
| stage place + pick | 57 | 9 % | lowers at 5 %: 7.5 s each |
| microscope place + pick | 38 | 6 % | lowers 66 mm at 5 %: 14 s each |
| rotation | 7 | 1 % | |
| remainder | ~125 | 19 % | jaw settling, health checks, vision frames, redundant "hover(corrected)" moves |

Implied joint speed at the 20 % profile: 3 to 5 deg/s. Each SDK move call costs about 1 s of overhead on top of the motion.

## 3. The seven route stops: root cause

Every stop was "abs joint move did not reach": the arm halted 3 to 8 degrees short of a waypoint with enable intact and no controller error. The vendor SDK runs its own position check after each move and compares the gripper axis to a 0.02 tolerance. With a plate held, the gripper reading jitters by up to 0.02 (short-edge grasp sd 0.007), so the check occasionally fails while the arm is still moving, and the SDK sends MotionStop. This is an SDK artifact, not a mechanical event. The jaw readings show the plate never moved in the fingers.

Fix applied: when a move ends short with enable intact, no drag, no controller error and the shortfall under 12 degrees, the same command is re-issued once and logged as `sdk_motionstop`. A real collision drops enable and still stops the run. Long term the motion moves into the controller (Lua), which has no such check.

## 4. Repeatability measured

| Measure | n | mean | sd | min | max |
|---|---|---|---|---|---|
| jaw closed, short edges | 985 | 24.005 | 0.007 | 24.00 | 24.02 |
| jaw closed, long edges | 986 | 2.840 | 0.001 | 2.83 | 2.85 |
| route B waypoint pose deviation (mm) | ~38k | | | p90 0.83 | 1.48 |
| cycle time drift, first 10 vs last 10 cycles | | 654 -> 660 s | | | |

The grasp signature is far tighter than the 0.35 tolerance in use. A 5-sigma band is 0.035 for short edges and 0.006 for long edges, so a plate seated 0.1 off is detectable at the jaw alone.

## 5. Measures still missing

1. Controller collision sensitivity value: never read from the pendant; it fired once (hotel pick) so it is on, but the setting is undocumented in our records.
2. Gripper torque and joint torques: not exposed by the SDK; available in Lua (`GetCJointsTrq`, `GripperEmptyDet`).
3. Rack geometry: front face x, column y extents, shelf z pitch verified only through A1 and A9.
4. Camera reference images for the gates, and ArUco markers for the pose offsets: all gates ran disabled.
5. Plate identity: no barcode on the calibration plate; 492 cycles scanned "none".
6. SDK call latency per move (only per-waypoint dt is logged).
7. Stage angle readback after each rotation (now logged as `polar readback`).
8. Controller error list every health check (now read over BRCAPI).
9. Microscope-side confirmation that the plate is seated (camera references, or a switch on the holder).
10. Time sync: the Pi clock is wrong, so vision-box logs cannot be aligned with ours.

## 6. Error prediction and correction, by failure mode

| Failure mode | Predictor (data already logged) | Correction |
|---|---|---|
| plate shifted in the fingers | jaw closed value outside the 5-sigma band; closing curve shape vs template | re-open, lift, re-seat once; else stop |
| plate not seated | seated-gate correlation drop (needs references) | re-place once; else stop; microscope must not scan |
| mechanical drift of a fixture | per-step pose deviation trend (CUSUM over cycles), ArUco offset trend | apply the measured offset when under 3 mm; alert at 3 mm; stop at 25 mm |
| servo or friction change | per-step duration CUSUM; implied deg/s falling | slow the profile one notch, alert |
| SDK MotionStop | shortfall with enable intact | re-issue once (done) |
| collision | enable dropped, deviation > 1 mm | stop, never resume automatically |
| vision box outage | status latency rising, failures | retry 5 min, then stop |
| disk | free GB trend | ring buffer (done): 0.5 fps, 640x360, 8 GB per run |
| lost enable while stationary | health check | one re-enable per check, counted |

Statistical method for all trend predictors: robust median/MAD baselines per feature (already in `predictor.py`), plus a CUSUM on the standardized residual with a 5-sigma decision threshold, which gives an estimated cycles-to-threshold number instead of a binary alarm. With 492 cycles of baseline data these can be fitted offline this week.

## 7. Speed plan

1. Replace route B with the 13 via points of the same demo (`microscope_to_polar_fast` in layout), validated at 20 % with the operator watching, then at the `fast` profile 40/20/8. Expected: 302 s -> about 60 s per cycle.
2. Staged lowers and slides: approach speed until 10 mm from the seat, contact speed after (done).
3. Drop the redundant corrected-hover move when the correction is zero (done).
4. Hotel approach joint move at travel speed (22 s -> ~6 s).
Target: a cycle under 4 minutes without raising contact speeds.

## 8. Hotel shuffling

Hotel-to-hotel transfers keep the long-edge grip and the yaw, so they do not need the stage: pick slot S1, retreat straight out, climb or descend in a column clear of the shelf fronts, approach S2, slide in, place. The climb column needs the rack's front x. A3 stopped on height alone because the climb happened inside the shelf depth; the retreat-then-climb path removes that.
