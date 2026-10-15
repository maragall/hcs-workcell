function mod_Cell()
  --==================================================================
  -- Plate-transfer cell dispatcher. Hooked from mod_Socket as
  --     mod_Cell()  mod_CellEnd()
  -- SYSINVAR0 device 10 microscope / 11 polar / 12 hotel
  -- SYSINVAR1 hotel row (1,3,5,7,9)  SYSINVAR2 hotel column 1..3  SYSINVAR3 = 1
  -- SYSINVAR4 1 pick 2 place 3 rotate polar 4 go home 5 open 6 close 7 empty-det report
  -- SYSINVAR5 grip 1 short edges 2 long edges   SYSINVAR6 polar 1 = 0 deg 2 = 90 deg
  -- SYSINVAR7 hover / lift override in mm (0 = default)
  -- Result in CellResult (1, -1, -2, -3), detail in CellDetail (README_LUA.md).
  --==================================================================
  data_cell()
  CellResult=1
  CellDetail=0
  CellAbort=0
  CellFloorZ=-9999
  if(CellDetOn==nil)then
    CellDetOn=0
  end
  CollSupvOn()
  cell_Speed(CellScaleTravel)

  -- Hold the current jaw width and platform angle unless the job changes them.
  -- Every target gets these written into elements 7/8 before it is used, so a
  -- stale E2 in a taught point can never turn the platform by accident.
  local LJ=GetCJoints()
  CellGripW=GetElem(LJ,7)
  CellE2=GetElem(LJ,8)

  if(cell_Validate()==0)then
    return
  end
  cell_SetGrip()
  cell_SetLift()

  if(SYSINVAR4==4)then
    cell_GoHome()
  elseif(SYSINVAR4==5)then
    cell_GripOpen()
  elseif(SYSINVAR4==6)then
    CellDetail=cell_GripCloseSlow()
    if(CellDetail==1 and CellDetOn==0)then
      EnableGripperDet(1)
      CellDetOn=1
    end
  elseif(SYSINVAR4==7)then
    CellDetail=GripperEmptyDet(1,GRIP_DET_TRQ,GRIP_DET_DIS)
  elseif(SYSINVAR4==3)then
    cell_PolarRotate()
  elseif(SYSINVAR0==10)then
    mod_CellScope()
  elseif(SYSINVAR0==11)then
    mod_CellPolar()
  elseif(SYSINVAR0==12)then
    mod_CellHotel()
  end
end--functionend

function mod_CellEnd()
  MOVFINISH()
  ShowInfo("mod_Cell result=" .. CellResult .. " detail=" .. CellDetail)
  --Device ID
  SetSysInVarI(0,0)
  --Pos X
  SetSysInVarI(1,0)
  --Pos Y
  SetSysInVarI(2,0)
  --Pos Z
  SetSysInVarI(3,0)
  --MoveType
  SetSysInVarI(4,0)
  --ObjType
  SetSysInVarI(5,0)
  --ClampMode
  SetSysInVarI(6,0)
  --Lift override
  SetSysInVarI(7,0)
  -- index 1 first: the PC treats the first pushed value as the job code
  SendSysOutVarI(1,CellResult)
  if(CellDetail~=0)then
    SendSysOutVarI(2,CellDetail)
  end
end--functionend

--==================================================================
-- Validation
--==================================================================
function cell_Invalid(code)
  ShowInfo("mod_Cell invalid request, detail=" .. code .. " SYSINVAR0=" .. SYSINVAR0 .. " SYSINVAR4=" .. SYSINVAR4)
  CellResult=-1
  CellDetail=code
  return 0
end--functionend

function cell_Taught(P)
  if(GetElem(P,3)<=0)then
    return 0
  end
  return 1
end--functionend

function cell_Validate()
  if(SYSINVAR0<10 or SYSINVAR0>12 or SYSINVAR3~=1 or SYSINVAR4<1 or SYSINVAR4>7)then
    return cell_Invalid(11)
  end
  if(SYSINVAR4==3 and SYSINVAR0~=11)then
    return cell_Invalid(17)
  end
  if(SYSINVAR4==1 or SYSINVAR4==2 or SYSINVAR4==5 or SYSINVAR4==6)then
    if(SYSINVAR5~=1 and SYSINVAR5~=2)then
      return cell_Invalid(13)
    end
  end
  if(SYSINVAR4==1 and CellDetOn==1)then
    -- a pick would open the jaw on a plate we are still carrying
    return cell_Invalid(18)
  end
  if(SYSINVAR0==10 and SYSINVAR4<=2)then
    if(SYSINVAR5~=1)then
      return cell_Invalid(15)
    end
    if(cell_Taught(PSCOPE)==0 or ZFLOOR_SCOPE<=0 or SCOPE_BOX_X1>=SCOPE_BOX_X2)then
      return cell_Invalid(19)
    end
  end
  if(SYSINVAR0==11 and SYSINVAR4<=3)then
    if(SYSINVAR6~=1 and SYSINVAR6~=2)then
      return cell_Invalid(14)
    end
    if(SYSINVAR4<=2 and SYSINVAR5~=SYSINVAR6)then
      -- short edges only at 0 deg, long edges only at 90 deg
      return cell_Invalid(14)
    end
    if(SYSINVAR4<=2 and SYSINVAR6==1 and cell_Taught(PPOLAR0)==0)then
      return cell_Invalid(19)
    end
    if(SYSINVAR4<=2 and SYSINVAR6==2 and cell_Taught(PPOLAR90)==0)then
      return cell_Invalid(19)
    end
  end
  if(SYSINVAR0==12 and SYSINVAR4<=2)then
    if(SYSINVAR2<1 or SYSINVAR2>3)then
      return cell_Invalid(12)
    end
    if(SYSINVAR1~=1 and SYSINVAR1~=3 and SYSINVAR1~=5 and SYSINVAR1~=7 and SYSINVAR1~=9)then
      return cell_Invalid(12)
    end
  end
  if(SYSINVAR4<=3)then
    -- fixture moves start from JH; after any stop the PC must send action 4 first
    if(Check4JointsEqual(GetCJoints(),JH,HOME_TOL_DEG)==0)then
      return cell_Invalid(16)
    end
  end
  return 1
end--functionend

function cell_SetGrip()
  if(SYSINVAR5==2)then
    CellGripOpen=GRIP_LONG_OPEN
    CellGripClose=GRIP_LONG_CLOSE
  else
    CellGripOpen=GRIP_SHORT_OPEN
    CellGripClose=GRIP_SHORT_CLOSE
  end
end--functionend

function cell_SetLift()
  CellHover=HOVER_MM
  CellLift=HOTEL_LIFT_MM
  if(SYSINVAR7>0)then
    CellHover=SYSINVAR7
    CellLift=SYSINVAR7
  end
  if(CellHover>HOVER_MAX_MM)then
    CellHover=HOVER_MAX_MM
  end
  if(CellLift>HOTEL_LIFT_MAX_MM)then
    CellLift=HOTEL_LIFT_MAX_MM
  end
  if(CellLift<HOTEL_LIFT_MIN_MM)then
    CellLift=HOTEL_LIFT_MIN_MM
  end
end--functionend

--==================================================================
-- Motion wrappers: speed scale, E1/E2 sync, Z floor, abort latch
--==================================================================
function cell_Speed(pct)
  MOVFINISH()
  SetSpeedScale(pct)
end--functionend

function cell_SyncExt(T)
  SetElem(T,7,CellGripW)
  SetElem(T,8,CellE2)
end--functionend

-- Z floor check: P target, dz offset about to be applied. Latches CellAbort
-- so that every later cell_Move* in the job is skipped.
function cell_FloorOk(P,dz)
  local LZ=GetElem(P,3)+dz
  if(LZ<CellFloorZ)then
    ShowInfo("FLOOR VIOLATION z=" .. LZ .. " floor=" .. CellFloorZ)
    CellResult=-3
    CellDetail=31
    CellAbort=1
    return 0
  end
  return 1
end--functionend

function cell_MoveAbsJ(J,V,Z)
  if(CellAbort==1)then
    return
  end
  cell_SyncExt(J)
  MoveAbsJ(J,V,Z)
end--functionend

function cell_MoveL(P,V,Z)
  if(CellAbort==1)then
    return
  end
  if(cell_FloorOk(P,0)==0)then
    return
  end
  cell_SyncExt(P)
  MoveL(P,V,Z,T0,U0)
end--functionend

function cell_MoveOffs(P,dx,dy,dz,V,Z)
  if(CellAbort==1)then
    return
  end
  if(cell_FloorOk(P,dz)==0)then
    return
  end
  cell_SyncExt(P)
  MoveOffs(P,dx,dy,dz,V,Z,T0,U0)
end--functionend

function cell_InBox(x,y,x1,x2,y1,y2)
  if(x>=x1 and x<=x2 and y>=y1 and y<=y2)then
    return 1
  end
  return 0
end--functionend

--==================================================================
-- Gripper (external axis E1). Jaw-only moves use MoveExtJ on a snapshot
-- of the current joints (manual Appendix B idiom), so the arm never moves.
--==================================================================
function cell_GripTo(w)
  if(CellAbort==1)then
    return
  end
  local LJ=GetCJoints()
  SetElem(LJ,7,w)
  CellGripW=w
  MoveExtJ(LJ,V_GRIP,0,T0,U0)
  MOVFINISH()
end--functionend

function cell_GripOpen()
  if(CellDetOn==1)then
    DisableGripperDet(1)
    CellDetOn=0
  end
  cell_GripTo(CellGripOpen)
end--functionend

-- Step the jaw closed, then confirm a plate. Returns 1 holding / 2 empty.
function cell_GripCloseSlow()
  local LW=CellGripW
  while(LW>CellGripClose and CellAbort==0)do
    LW=LW-GRIP_STEP_MM
    if(LW<CellGripClose)then
      LW=CellGripClose
    end
    cell_GripTo(LW)
    DelayMS(GRIP_STEP_MS)
  end
  DelayMS(GRIP_SETTLE_MS)
  return GripperEmptyDet(1,GRIP_DET_TRQ,GRIP_DET_DIS)
end--functionend

--==================================================================
-- Top-access fixtures (microscope, polar): hover, straight down, seat
--==================================================================
function cell_TopPick(P,JT)
  cell_GripOpen()
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JH,V_JOINT,-1)
  cell_MoveAbsJ(JT,V_JOINT,100)
  cell_MoveOffs(P,0,0,CellHover,V_LINEAR,0)
  cell_Speed(CellScaleApproach)
  cell_MoveOffs(P,0,0,SEAT_SLOW_MM,V_APPROACH,0)
  cell_Speed(CellScaleContact)
  cell_MoveL(P,V_SEAT,0)
  MOVFINISH()
  if(CellAbort==1)then
    return
  end
  local LR=cell_GripCloseSlow()
  if(LR~=1)then
    cell_GripOpen()
    cell_MoveOffs(P,0,0,CellHover,V_APPROACH,0)
    CellResult=-2
    CellDetail=21
    return
  end
  EnableGripperDet(1)
  CellDetOn=1
  cell_MoveOffs(P,0,0,CellHover,V_SEAT,0)
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JT,V_JOINT,100)
  cell_MoveAbsJ(JH,V_JOINT,-1)
end--functionend

function cell_TopPlace(P,JT)
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JH,V_JOINT,-1)
  cell_MoveAbsJ(JT,V_JOINT,100)
  cell_MoveOffs(P,0,0,CellHover,V_LINEAR,0)
  cell_Speed(CellScaleApproach)
  cell_MoveOffs(P,0,0,SEAT_SLOW_MM,V_APPROACH,0)
  cell_Speed(CellScaleContact)
  cell_MoveL(P,V_SEAT,0)
  MOVFINISH()
  cell_GripOpen()
  cell_MoveOffs(P,0,0,CellHover,V_APPROACH,0)
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JT,V_JOINT,100)
  cell_MoveAbsJ(JH,V_JOINT,-1)
end--functionend

function mod_CellScope()
  CellFloorZ=ZFLOOR_SCOPE
  if(GetElem(PSCOPE,3)<ZFLOOR_SCOPE)then
    -- seat point taught below the floor: refuse before any motion
    CellResult=-3
    CellDetail=32
    return
  end
  if(SYSINVAR4==1)then
    cell_TopPick(PSCOPE,JSCOPE)
  elseif(SYSINVAR4==2)then
    cell_TopPlace(PSCOPE,JSCOPE)
  end
end--functionend

--==================================================================
-- Polar stage
--==================================================================
function cell_PolarRotate()
  if(POLAR_IS_E2==0)then
    return
  end
  if(SYSINVAR6==2)then
    CellE2=POLAR_E2_90
  else
    CellE2=POLAR_E2_0
  end
  -- arm is already at JH, so this move only turns the platform
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JH,V_JOINT,-1)
  MOVFINISH()
end--functionend

function mod_CellPolar()
  CellFloorZ=ZFLOOR_POLAR
  local LP=PPOLAR0
  if(SYSINVAR6==2)then
    LP=PPOLAR90
  end
  cell_PolarRotate()
  if(SYSINVAR4==1)then
    cell_TopPick(LP,JPOLAR)
  elseif(SYSINVAR4==2)then
    cell_TopPlace(LP,JPOLAR)
  end
end--functionend

--==================================================================
-- Hotel: horizontal access from -X
--==================================================================
function cell_HotelPick(P,JT)
  cell_GripOpen()
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JH,V_JOINT,-1)
  cell_MoveAbsJ(JT,V_JOINT,100)
  cell_MoveOffs(P,-HOTEL_APPROACH_MM,0,0,V_LINEAR,0)
  cell_Speed(CellScaleApproach)
  cell_MoveL(P,V_APPROACH,0)
  MOVFINISH()
  if(CellAbort==1)then
    return
  end
  cell_Speed(CellScaleContact)
  local LR=cell_GripCloseSlow()
  if(LR~=1)then
    cell_GripOpen()
    cell_MoveOffs(P,-HOTEL_APPROACH_MM,0,0,V_APPROACH,0)
    CellResult=-2
    CellDetail=21
    return
  end
  EnableGripperDet(1)
  CellDetOn=1
  cell_MoveOffs(P,0,0,CellLift,V_SEAT,0)
  cell_Speed(CellScaleApproach)
  cell_MoveOffs(P,-HOTEL_APPROACH_MM,0,CellLift,V_APPROACH,0)
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JT,V_JOINT,100)
  cell_MoveAbsJ(JH,V_JOINT,-1)
end--functionend

function cell_HotelPlace(P,JT)
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JH,V_JOINT,-1)
  cell_MoveAbsJ(JT,V_JOINT,100)
  cell_MoveOffs(P,-HOTEL_APPROACH_MM,0,CellLift,V_LINEAR,0)
  cell_Speed(CellScaleApproach)
  cell_MoveOffs(P,0,0,CellLift,V_APPROACH,0)
  cell_Speed(CellScaleContact)
  cell_MoveL(P,V_SEAT,0)
  MOVFINISH()
  cell_GripOpen()
  cell_Speed(CellScaleApproach)
  cell_MoveOffs(P,-HOTEL_APPROACH_MM,0,0,V_APPROACH,0)
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(JT,V_JOINT,100)
  cell_MoveAbsJ(JH,V_JOINT,-1)
end--functionend

function mod_CellHotel()
  CellFloorZ=ZFLOOR_HOTEL
  local LP1=PHOTEL_A1
  local LP9=PHOTEL_A9
  local LJT=JHOTEL_A
  if(SYSINVAR2==2)then
    LP1=PHOTEL_B1
    LP9=PHOTEL_B9
    LJT=JHOTEL_B
  elseif(SYSINVAR2==3)then
    LP1=PHOTEL_C1
    LP9=PHOTEL_C9
    LJT=JHOTEL_C
  end
  if(cell_Taught(LP1)==0 or cell_Taught(LP9)==0)then
    cell_Invalid(19)
    return
  end
  -- rows 1..9 interpolated between the two taught ends of the column
  local LStack=StackConfiguration(9,1,LP1,LP1,LP9,LP9)
  PHOTEL=GetStackTargetPoint(LStack,SYSINVAR1,1)
  if(SYSINVAR4==1)then
    cell_HotelPick(PHOTEL,LJT)
  elseif(SYSINVAR4==2)then
    cell_HotelPlace(PHOTEL,LJT)
  end
end--functionend

--==================================================================
-- Action 4: return to JH from wherever the last job stopped.
-- Retreat direction depends on the XY region: up out of a top-access
-- fixture, -X out of the hotel, then joint move via that fixture's transit.
--==================================================================
function cell_GoHome()
  local LP=GetCRobT(T0,U0)
  local LX=GetElem(LP,1)
  local LY=GetElem(LP,2)
  local LZ=GetElem(LP,3)
  local LJT=JH
  local LUp=0
  cell_Speed(CellScaleApproach)
  if(cell_InBox(LX,LY,SCOPE_BOX_X1,SCOPE_BOX_X2,SCOPE_BOX_Y1,SCOPE_BOX_Y2)==1)then
    CellFloorZ=ZFLOOR_SCOPE
    LUp=GetElem(PSCOPE,3)+CellHover-LZ
    LJT=JSCOPE
  elseif(cell_InBox(LX,LY,POLAR_BOX_X1,POLAR_BOX_X2,POLAR_BOX_Y1,POLAR_BOX_Y2)==1)then
    CellFloorZ=ZFLOOR_POLAR
    LUp=GetElem(PPOLAR0,3)+CellHover-LZ
    if(SYSINVAR6==2)then
      LUp=GetElem(PPOLAR90,3)+CellHover-LZ
    end
    LJT=JPOLAR
  elseif(cell_InBox(LX,LY,HOTEL_BOX_X1,HOTEL_BOX_X2,HOTEL_BOX_Y1,HOTEL_BOX_Y2)==1)then
    CellFloorZ=ZFLOOR_HOTEL
    cell_MoveOffs(LP,-HOTEL_APPROACH_MM,0,0,V_APPROACH,0)
    LJT=JHOTEL_B
    if(LY>HOTEL_BOX_Y2/3)then
      LJT=JHOTEL_A
    elseif(LY<HOTEL_BOX_Y1/3)then
      LJT=JHOTEL_C
    end
  end
  if(LUp>0)then
    cell_MoveOffs(LP,0,0,LUp,V_APPROACH,0)
  end
  cell_Speed(CellScaleTravel)
  cell_MoveAbsJ(LJT,V_JOINT,100)
  cell_MoveAbsJ(JH,V_JOINT,-1)
end--functionend
--Fileinfomation:
