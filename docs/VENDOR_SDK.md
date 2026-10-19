# Vendor .NET SDK (BR.PC.BYSDK 1.14) as observed on the Nova S4, October 2026

Everything below was measured live; the vendor package documents the controller's BRCAPI protocol
(port 56788) but not this SDK (port 52525). Use only through `nova_cell/robot.py`.

## Connection and modes
- `RobotSDKFactory.Create().ConnectAsync(ip, 52525)`; one client at a time. A second client hangs.
- `SetEnableAsync(True)` may need several tries after a power cycle; readouts are stale until the first
  success.
- `EnableDragModeAsync(False)` and `SwitchModeAsync(Manual)` both drop the enable. Re-enable after.
- Entering drag mode drives the gripper to its last held value by itself. Enter drag only with the jaw closed
  or with the plate already gripped. In drag mode the operator can push the fingers open by hand, and the
  rotary platform (E2) is unpowered and drifts.
- `GetMotionInfo()` fields: `IsEnable`, `IsMoving`, `IsInDragMode`, `IsInAutoMode`, `SpeedPercentage`. No error
  field: read `getRobotErr` over BRCAPI instead.

## Poses and joints
- `GetCurPoseAsync()` → x y z (mm) rx ry rz with **rz in degrees** (BRCAPI reports radians and metres).
- `GetCurJointAsync()` → 8 values: J1 to J4 (J1 is the z column, in mm-like units) and four zeros.
- `GetCurExtJointAsync()` → 8 values: index 0 = E1 gripper gap, index 1 = E2 rotary platform (degrees).

## Motion
- `MoveLinearAsync(x,y,z,rx,ry,rz, speed)`, `MoveJointAsync(...)`, `MoveAbsJointAsync(double[8], speed)`,
  `MoveSingleExtJointAsync(index, value)` with a **1-based** external-axis index (1 = E1, 2 = E2). Index 0 is
  rejected with "外部关节索引不合法".
- `MoveAbsJointAsync` needs exactly 8 elements; the controller executes J1 to J4 and ignores the rest.
- Speed is a fraction 0.01 to 1.0; 100 % corresponds to roughly 20 deg/s on the rotary joints and about
  20 mm/s on the column, consistent with the controller's manual-mode cap.
- **The completion flag lies.** After every move the SDK polls the position and compares all axes, including
  the gripper, to 0.02 units. With a plate held, gripper jitter trips it, the SDK returns False and sends
  MotionStop, sometimes while the arm is still moving. Moves also return False when the check times out at
  slow speeds. Verify completion by reading the pose or joints yourself; `robot.py` does, and re-issues a
  stopped move once when enable is intact (a collision drops enable).
- `MoveLinearAsync` occasionally produces no motion for a short move near the microscope hover; the adapter
  falls back to a joint-interpolated move for the same target when the displacement is under 30 mm.
- Known from earlier work: `MoveExtJointAsync(double[8])` faults the controller; use the single-axis call.

## Gripper
- Gap units: fully closed on air reads about −0.04; the 128 mm plate length reads 24.0; the 85 mm width
  2.84. Larger is wider.
- Close in 0.5 steps with 0.15 s pauses; opening can be a single move. A step the servo cannot complete is
  refused, which is the contact signal `measure-grip` uses.

## Rotary platform (E2)
- Read index 1, move index 2. Verify by readback within 0.5 degrees. Zero reference is the reading when the
  stage is parallel to x; this cell uses −90 for the long-edge orientation.

## Logs
- The SDK writes its own log under `site-packages/BioyondSDK/_dotnet/Logs/`, in Chinese. Useful lines:
  `关节运动失败,目标位置` (joint move failed, target position), `位置验证失败` (position verification failed),
  `外部关节索引不合法` (external joint index invalid).
