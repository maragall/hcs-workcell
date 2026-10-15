function data_cell()
  --==================================================================
  -- Plate-transfer cell: constants and taught targets.
  -- Called at the start of every mod_Cell() job so that every target is
  -- a fresh copy (SetElem mutates targets in place, as in the template).
  -- Units: mm, deg. Target layout (12 numbers): X Y Z RX RY RZ E1 E2 0 0 0 0
  --   element 7 = E1 gripper jaw position (mm, pendant reading)
  --   element 8 = E2 rotary platform (deg)   -- see POLAR_IS_E2
  -- Elements 7/8 of every target below are overwritten at run time by
  -- cell_SyncExt(), so only X Y Z RX RY RZ (or J1..J4) need teaching.
  -- A RobTarget whose Z is 0 counts as NOT TAUGHT and the job is refused.
  --==================================================================

  ---------------------------------------------------------------- mode
  CELL_SUPERVISED=1        -- 1 = SUPERVISED speed set, 0 = UNSUPERVISED
  POLAR_IS_E2=1            -- 1 = polar stage is robot external axis E2
                           -- 0 = PC rotates the stage, Lua ignores SYSINVAR6 rotation

  ---------------------------------------------------------------- speeds
  -- SetSpeedScale percentages (integer 1-100). Travel = joint transits
  -- through JH, approach = hover/slide moves, contact = last mm + jaw.
  SCALE_SUP_TRAVEL=5
  SCALE_SUP_APPROACH=2
  SCALE_SUP_CONTACT=1
  SCALE_UNSUP_TRAVEL=3
  SCALE_UNSUP_APPROACH=2
  SCALE_UNSUP_CONTACT=1
  -- per-move V arguments, same magnitudes as the template (VAR1/VAR2/VAR10)
  V_JOINT=100
  V_LINEAR=1000
  V_APPROACH=500
  V_SEAT=200
  V_GRIP=100

  if(CELL_SUPERVISED==1)then
    CellScaleTravel=SCALE_SUP_TRAVEL
    CellScaleApproach=SCALE_SUP_APPROACH
    CellScaleContact=SCALE_SUP_CONTACT
  else
    CellScaleTravel=SCALE_UNSUP_TRAVEL
    CellScaleApproach=SCALE_UNSUP_APPROACH
    CellScaleContact=SCALE_UNSUP_CONTACT
  end

  ---------------------------------------------------------------- gripper
  -- E1 jaw positions in mm as read on the pendant. Template convention:
  -- E1 = (plate opening - 76) / 2. CLOSE targets must sit at least
  -- GRIP_DET_DIS below the value where the jaw stalls on the plate.
  GRIP_SHORT_OPEN=31.0     -- TODO TEACH: jaw open for a SHORT-edge grasp (~138 mm)
  GRIP_SHORT_CLOSE=24.0    -- TODO TEACH: stall on 127.8 mm plate ~25.9 -> target 2 mm lower
  GRIP_LONG_OPEN=10.0      -- TODO TEACH: layout.json gripper.open
  GRIP_LONG_CLOSE=1.0      -- TODO TEACH: layout.json closed_on_plate 2.92 -> target 2 mm lower
  GRIP_STEP_MM=0.5         -- jaw step while closing
  GRIP_STEP_MS=150         -- pause between steps
  GRIP_SETTLE_MS=300       -- pause before GripperEmptyDet
  GRIP_DET_TRQ=0.20        -- TODO TUNE: normalised torque threshold (0-1), ~half the clamp torque
  GRIP_DET_DIS=0.5         -- TODO TUNE: |actual - target| jaw position threshold (same units as E1)

  ---------------------------------------------------------------- heights
  HOVER_MM=25              -- default hover above the seat point (scope, polar)
  HOVER_MAX_MM=60          -- clamp for the SYSINVAR7 override
  SEAT_SLOW_MM=5           -- last mm of every descent run at contact speed
  HOTEL_APPROACH_MM=145    -- pre-position distance in -X in front of the hotel
  HOTEL_LIFT_MM=25         -- default lift inside a hotel slot
  HOTEL_LIFT_MIN_MM=10     -- clamp for the SYSINVAR7 override
  HOTEL_LIFT_MAX_MM=30     -- shelf pitch limit (layout.json max_lift_mm)
  HOME_TOL_DEG=5           -- Check4JointsEqual tolerance for "robot is at JH"

  ---------------------------------------------------------------- Z floors
  -- Nothing is commanded below the active floor; cell_MoveL/cell_MoveOffs
  -- check every target. Scope floor = taught seat Z - 2 (job refused while 0).
  ZFLOOR_SCOPE=0           -- TODO TEACH: PSCOPE Z - 2
  ZFLOOR_POLAR=60          -- layout.json regions.polar.z_floor
  ZFLOOR_HOTEL=55          -- layout.json regions.hotel.z_floor

  ---------------------------------------------------------------- XY region boxes
  -- Used by action 4 (go home) to choose a safe retreat direction.
  SCOPE_BOX_X1=0           -- TODO TEACH: microscope stage XY box (robot base, mm)
  SCOPE_BOX_X2=0
  SCOPE_BOX_Y1=0
  SCOPE_BOX_Y2=0
  POLAR_BOX_X1=100         -- layout.json regions.polar.box
  POLAR_BOX_X2=150
  POLAR_BOX_Y1=90
  POLAR_BOX_Y2=145
  HOTEL_BOX_X1=255         -- layout.json regions.hotel.box
  HOTEL_BOX_X2=330
  HOTEL_BOX_Y1=-170
  HOTEL_BOX_Y2=170

  ---------------------------------------------------------------- polar angles
  POLAR_E2_0=0             -- E2 reading for angle code 1 (short edges to gripper)
  POLAR_E2_90=90           -- E2 reading for angle code 2 (long edges to gripper)

  ---------------------------------------------------------------- joint points
  -- J1 = Z column (mm), J2..J4 = arm joints (deg). Untaught transit points
  -- default to JH so that a forgotten point only costs an extra home move.
  JH=JointTarget(250,0,0,0,0,0,0,0,0,0,0,0)           -- TODO TEACH: home, arm clear of all fixtures
  JSCOPE=JointTarget(250,0,0,0,0,0,0,0,0,0,0,0)       -- TODO TEACH: above microscope, TCP at hover XY
  JPOLAR=JointTarget(250,0,0,0,0,0,0,0,0,0,0,0)       -- TODO TEACH: above polar stage, TCP at hover XY
  -- Hotel transit points: TCP at the column pre-position XY (slot X - 145, slot Y),
  -- Z mid-column (row 5). Then the move to any slot pre-position is vertical.
  JHOTEL_A=JointTarget(250,0,0,0,0,0,0,0,0,0,0,0)     -- TODO TEACH
  JHOTEL_B=JointTarget(250,0,0,0,0,0,0,0,0,0,0,0)     -- TODO TEACH
  JHOTEL_C=JointTarget(250,0,0,0,0,0,0,0,0,0,0,0)     -- TODO TEACH

  ---------------------------------------------------------------- fixture points
  -- MICROSCOPE: plate seated, gripped on SHORT edges, approached from above.
  PSCOPE=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)           -- TODO TEACH (Z 0 = refused)

  -- POLAR: two seat points, one per platform angle (gripper yaw differs by 90).
  -- layout.json pose_at_90 reads 125.421,117.434,67.859; RZ convention unverified.
  PPOLAR0=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)          -- TODO TEACH: E2=0, SHORT edges
  PPOLAR90=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)         -- TODO TEACH: E2=90, LONG edges

  -- HOTEL: row 1 and row 9 seat point per column; rows 3,5,7 are interpolated
  -- by StackConfiguration(9,1,...). layout.json XYZ for reference:
  --   A1 290.98,137.47,73.94  A9 292.98,137.47,373.97
  --   B1 293.99,0.65,72.13    C1 292.98,-129.53,71.95   (RZ there is -58.9 deg, verify)
  PHOTEL_A1=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)        -- TODO TEACH
  PHOTEL_A9=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)        -- TODO TEACH
  PHOTEL_B1=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)        -- TODO TEACH
  PHOTEL_B9=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)        -- TODO TEACH
  PHOTEL_C1=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)        -- TODO TEACH
  PHOTEL_C9=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)        -- TODO TEACH
  PHOTEL=RobTarget(0,0,0,0,0,0,0,0,0,0,0,0)           -- computed slot point, not taught
end--functionend
--Fileinfomation:
