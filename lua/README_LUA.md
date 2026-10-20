# nova_cell Lua modules for the Bioyond S4 (BioyondWorks V3.9 template)

## Gripper force probe (`torque_probe/`, 2026-10-20) — load this first

The PC cannot read the gripper servo's torque: the .NET SDK and the JSON port report positions only. The controller
can (manual A.4.18 `GetCJointsTrq`, A.4.19 `GetCJointsNormTrq`, A.10.5 `GripperEmptyDet`). `torque_probe/` is a
stand-alone two-file program that publishes the E1 width and torque through the integer system variables ten times a
second and executes jaw-only requests; the arm never moves. PC side: `python torque_probe.py watch|close|open|burst`.

BioyondWorks 1.1.20251009 is installed on the cell PC (`C:\Bioyond\RobStudio\BR.PC.App.RobStudio.exe`, 2026-10-20).
Default login per manual 3.3: admin / Admin@123. Its `Odin/user/__lif.lua` is the vendor Lua library the controller
runs; function shapes in the probe were checked against it. Its `Odin/config/gripper.conf` shows the shipped gripper
defaults: ctrl_mode 0, trq_thr 0.25, pos_thr 0.0005, default_trq 0.75.

Import (BioyondWorks): open the V3.9 template project, Program library > Import `mod_TorqueProbe.lua`,
overwrite `main.lua` with `torque_probe/main.lua` (keep every other template file), Publish, switch to auto, enable,
Run. On the PC `python torque_probe.py watch` must show the heartbeat counting; if `lua_ready` stays false the program
is not running. Restore the template `main.lua` when done with the probe.

What the manual says about force (Appendix B.1, page 57-58): the gripper module has a config file with
`<ctrl_mode>` (torque mode or position mode), `<default_trq>` 0.7 (70 % clamp force) and `<trq_thr>` 0.35. In position
mode the clamp force is fixed at the default and `SetGripperTorqueLimit` is refused; in torque mode it can be changed
live with `SetGripperTorqueLimit(1, 1, 0..1)` and the close target should be "fully closed". Which mode our unit is in
is read from the pendant (gripper configuration), not from the PC. Also available: `SetExtJTrqLimit(ForceArray)` and
`MoveExtJWithForce(NumArray, V, ForceArray)` (A.10.3-4), a force-limited external-axis move; probe request 23 wraps it
and is marked experimental because the torque sign convention is undocumented.

Measured 2026-10-20 04:00 with the probe published and running (production plate on long edges, free on a table):
- Holding torque after a close is **70 % of rated** (normalised -0.70, raw -186) for every method tried: one command
  to 1.45, one command to 0.8, 0.5 mm steps, through Lua or through the SDK. The commanded target stays below the plate
  and the servo pushes at its cap (gripper.conf default_trq 0.75 on the shipped config). Width at the stall 1.62-1.67.
- With nothing between the fingers the servo holds **+40 to +50 %** at every opening tried (3, 4, 6, 8, 10): a preload
  in the mechanism. "Empty" is not zero torque; `GripperEmptyDet` relies on the position gap as well.
- **SDK motion is refused while any Lua program runs** (jaw commands return False and nothing moves; arm commands
  presumably the same). The SDK connection itself is fine. So the cycle and the probe cannot run together: stop the
  program (remoteStop), move with the SDK, start it again (switchToAuto, remoteEnable, remoteStart, ~3 s) to read.
- BioyondWorks holds the single SDK session while it is open: close it before running anything on port 52525.
- The `SendSysOutVarI` push reaches one client only; with BioyondWorks connected the PC sees none, and
  `torque_probe.py` falls back to the request variable returning to 0.

Requests (`torque_probe.py` writes SYSINVAR0..3, reads the `SendSysOutVarI(1/2)` push): 21 close to W mm then
`GripperEmptyDet` verdict + holding torque; 22 open to W; 23 force close (experimental); 24 set clamp force (torque
mode only); 25 one-second torque burst, peak returned.

## Files
| file | role |
|---|---|
| `data_cell.lua` | all constants and taught targets (`function data_cell()`, called at every job start so targets are fresh copies) |
| `mod_Cell.lua` | dispatcher `mod_Cell()`, reply `mod_CellEnd()`, fixture routines, motion/gripper wrappers, Z floor check |
| `mod_Socket.lua`, `main.lua`, `mod_Init.lua`, `errhandler.lua` | drop-in replacements for the template files (devices 10-12 hooked, index 7 zeroed, gripper detection reset) |

## Import into BioyondWorks
1. Start from the V3.9 clean template project (`V3.9_LUA干净模版S4测试`). Keep `data.lua`, `mod_End.lua`, `mod_EndInvalid.lua` and the `modS_*/modR_*` files as they are.
2. Program library > the three dots next to the project > Import: add `data_cell.lua` and `mod_Cell.lua`; overwrite `mod_Socket.lua`, `main.lua`, `mod_Init.lua`, `errhandler.lua`. The IDE regenerates `__loader.lua` from the file list; `data_cell` is a function, so load order does not matter.
3. If the IDE insists on one function per file, split `mod_Cell.lua` at the `end--functionend` markers (every function is self-contained) and keep the names.
4. Publish, switch to auto, enable, run. `main()` only calls `mod_Socket()`.

## Teaching (all `TODO TEACH` lines in `data_cell.lua`)
A RobTarget whose Z is still 0 counts as untaught and the job is refused with -1/19, so teach in this order:
- `JH` home with the arm clear of everything; every job starts and ends here (the pick/place guard requires it, tolerance `HOME_TOL_DEG`).
- Microscope: `PSCOPE` = plate seated, SHORT-edge grasp; `ZFLOOR_SCOPE` = that Z - 2; `SCOPE_BOX_*` = XY box of the stage; `JSCOPE` = joint point above the stage with the TCP at the hover XY.
- Polar: `PPOLAR0` (E2 = 0, SHORT edges) and `PPOLAR90` (E2 = 90, LONG edges); `JPOLAR` above the stage. If the stage is not E2, set `POLAR_IS_E2=0` and teach both points at the respective physical angles.
- Hotel: row 1 and row 9 seat points per column (`PHOTEL_A1/A9/B1/B9/C1/C9`); rows 3,5,7 are interpolated with `StackConfiguration(9,1,...)`. `JHOTEL_A/B/C` with the TCP at the column pre-position (slot X - 145, slot Y, mid height) so the move to any slot pre-position is vertical.
- Gripper: `GRIP_*_OPEN/CLOSE` (E1 in mm, pendant reading); CLOSE must be at least `GRIP_DET_DIS` below the value where the jaw stalls on a plate. Tune `GRIP_DET_TRQ/DIS` with action 6 + 7 on a real plate and on an empty jaw.
- `CELL_SUPERVISED` picks the speed-scale set (5/2/1 % or 3/2/1 %). Only the per-move V values and these scales exist; nothing is ever faster than the template's own speeds.
layout.json XYZ values are quoted in comments for cross-checking; the RZ convention of the PC readout is unverified, so RZ is left 0 and must be taught.

## Handshake with the PC (`setSysVarI`, indices 0-7 in one call)
| index | meaning |
|---|---|
| 0 | 10 microscope, 11 polar, 12 hotel |
| 1 / 2 | hotel row 1,3,5,7,9 / column 1..3 (A,B,C); 1 otherwise |
| 3 | 1 |
| 4 | 1 pick, 2 place, 3 rotate polar only, 4 go home, 5 open jaw, 6 close jaw slowly, 7 empty-detect report |
| 5 | 1 short edges, 2 long edges (microscope needs 1; polar needs 5 == 6) |
| 6 | polar angle code 1 = 0 deg, 2 = 90 deg (required for device 11 actions 1-3) |
| 7 | hover (10/11) or lift (12) override in mm, 0 = default; clamped to `HOVER_MAX_MM` / `HOTEL_LIFT_MIN..MAX_MM` |

Reply: `SendSysOutVarI(1, code)` is pushed first, then `SendSysOutVarI(2, detail)` when detail is non-zero; SYSINVAR0..7 are zeroed before the push. The PC should treat only `index == 1` as the job code (`brcapi.run_job` currently takes the first push of any index).
Codes: 1 ok, -1 invalid, -2 jaw empty after the pick (jaw reopened, arm left at hover / outside the slot), -3 Z floor violation (no move below the floor was executed, remaining moves skipped).
Detail: 11 bad device/action/SYSINVAR3, 12 bad hotel row/column, 13 bad grip code, 14 polar angle missing or grip/angle mismatch, 15 microscope needs short edges, 16 robot not at JH (send action 4 first), 17 action 3 only for device 11, 18 pick refused while a plate is carried, 19 point/floor/box not taught, 21 jaw empty, 31 target below floor, 32 seat point below floor; for actions 6/7 the detail is the `GripperEmptyDet` result (1 holding, 2 empty).
After -2, -3 or any controller error the arm is not at `JH`: send action 4 (region-aware retreat: up out of microscope/polar, -X out of the hotel, then via the fixture transit point).

## Safety behaviour encoded
`CollSupvOn()` at every job start, never `CollSupvOff()`. Jaw closes in 0.5 mm `MoveExtJ` steps with `DelayMS` between, then `GripperEmptyDet(1,...)`. `EnableGripperDet(1)` after a confirmed grasp (kept on across jobs while carrying), `DisableGripperDet(1)` before every jaw opening, in `errhandler` and in `mod_Init`. Every `cell_MoveL/cell_MoveOffs` target is checked against the active floor before it is queued; a violation latches `CellAbort` so the rest of the job is skipped. Elements 7/8 of every target are rewritten from the job state before each move, so a stale E2 in a taught point can never turn the polar stage.

## API notes
Only functions from the template or manual Appendix A are used. There is no `ReadExtJoint`; E1/E2 are read with `GetElem(GetCJoints(),7/8)`. `GripperEmptyDet` returns 1 holding / 2 empty (from `__lif.lua`). `SetSpeedScale` takes an integer percentage, so 1.5 % is not expressible. `GetCRobT(T0,U0)` provides the current pose for the Z floor and go-home logic.
