# nova_cell Lua modules for the Bioyond S4 (BioyondWorks V3.9 template)

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
