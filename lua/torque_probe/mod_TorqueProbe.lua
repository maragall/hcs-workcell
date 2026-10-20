function mod_TorqueProbe()
  --==================================================================
  -- Gripper (E1) force probe, 2026-10-20. Runs on the controller and
  -- publishes the jaw width and the servo torque through the integer
  -- system variables that the PC polls with getSysVarI (JSON port 56788).
  -- Function shapes checked against the vendor library shipped with
  -- BioyondWorks 1.1.20251009 (Odin/user/__lif.lua): GetCJoints(),
  -- GetCJointsTrq(), GetCJointsNormTrq() return JOINT objects with
  -- fields _j1.._j8 and _ext1.._ext8; GripperEmptyDet(idx, trqtol, distol)
  -- returns 1 holding when |norm torque ext1| >= trqtol AND
  -- |actual - commanded| >= distol, else 2; MoveExtJWithForce(extj, spd,
  -- force) takes DefArray objects (ext targets, torque limits) and calls
  -- SetExtJTrqLimit then MoveExtJ.
  --
  -- Published every loop (about 10 Hz):
  --   SYSINVAR4 = E1 raw torque   x1000 (GetCJointsTrq()._ext1)
  --   SYSINVAR5 = E1 width        x100  (GetCJoints()._ext1, pendant units)
  --   SYSINVAR6 = E1 norm. torque x1000 (GetCJointsNormTrq()._ext1, 1000 = 100 %)
  --   SYSINVAR7 = heartbeat 0..9999
  --   SYSINVAR8 = E1 commanded width x100 (GetCRefJoints()._ext1)
  -- Requests (PC writes SYSINVAR0..3; the probe zeroes SYSINVAR0..3 when
  -- done and answers SendSysOutVarI(1, code) then SendSysOutVarI(2, detail)):
  --   SYSINVAR0=21 close: MoveExtJ to SYSINVAR1/100 at speed SYSINVAR3 (1..100),
  --                settle, detail = GripperEmptyDet(1, SYSINVAR2/1000, PROBE_DET_DIS)
  --                (1 holding, 2 empty); holding torque then in SYSINVAR6.
  --   SYSINVAR0=22 open:  MoveExtJ to SYSINVAR1/100 at speed SYSINVAR3.
  --   SYSINVAR0=23 force-limited close (first try with the operator at the e-stop):
  --                MoveExtJWithForce(DefArray(SYSINVAR1/100, E2 now, 0,0,0,0),
  --                SYSINVAR3, DefArray(SYSINVAR2/1000, 1, 1, 1, 1, 1)).
  --                The stage (E2) is always given its current reading so it never moves.
  --   SYSINVAR0=24 set clamp force (torque control mode only, manual B.1.2):
  --                SetGripperTorqueLimit(1, 1, SYSINVAR2/1000).
  --   SYSINVAR0=25 torque sample burst: 50 samples of |E1 norm torque| at 20 ms
  --                (PC side pulls on the plate meanwhile); peak x1000 in detail.
  -- Codes: 1 done, -1 bad request (detail 11 width, 12 torque).
  -- The gripper.conf shipped with the IDE: ctrl_mode 0, trq_thr 0.25,
  -- pos_thr 0.0005, default_trq 0.75. Read the real unit's values on the pendant.
  --==================================================================
  SetLuaAppSta(0)
  SetSysInVarI(0,0)
  SetSysInVarI(1,0)
  SetSysInVarI(2,0)
  SetSysInVarI(3,0)
  ResetSysOutVarI()
  DisableGripperDet(1)
  PROBE_DET_DIS=0.1
  PROBE_SETTLE_MS=300
  PROBE_V_DEFAULT=50
  SetLuaAppSta(1)
  local Beat=0
  while(1==1)do
    GetSysVarI()
    probe_Publish()
    SetSysInVarI(7,Beat)
    Beat=Beat+1
    if(Beat>9999)then
      Beat=0
    end
    if(SYSINVAR0>=21 and SYSINVAR0<=25)then
      probe_Request()
    elseif(SYSINVAR0~=0)then
      ShowInfo("torque probe: unknown request " .. SYSINVAR0)
      probe_Reply(-1,0)
    end
    DelayMS(100)
  end
end--functionend

function probe_Round(x)
  if(x>=0)then
    return (x+0.5)-((x+0.5)%1)
  end
  return -((0.5-x)-((0.5-x)%1))
end--functionend

function probe_Publish()
  local LJ=GetCJoints()
  local LR=GetCRefJoints()
  local LT=GetCJointsTrq()
  local LN=GetCJointsNormTrq()
  SetSysInVarI(4,probe_Round(LT._ext1*1000))
  SetSysInVarI(5,probe_Round(LJ._ext1*100))
  SetSysInVarI(6,probe_Round(LN._ext1*1000))
  SetSysInVarI(8,probe_Round(LR._ext1*100))
end--functionend

function probe_Reply(code,detail)
  SetSysInVarI(0,0)
  SetSysInVarI(1,0)
  SetSysInVarI(2,0)
  SetSysInVarI(3,0)
  ShowInfo("torque probe: code=" .. code .. " detail=" .. detail)
  SendSysOutVarI(1,code)
  SendSysOutVarI(2,detail)
end--functionend

function probe_GripTo(w,v)
  -- jaw-only move on a snapshot of the current joints (manual Appendix B idiom): E2 keeps its reading
  local LJ=GetCJoints()
  LJ._ext1=w
  MoveExtJ(LJ,v,0,T0,U0)
  MOVFINISH()
end--functionend

function probe_Request()
  local LReq=SYSINVAR0
  local LW=SYSINVAR1/100
  local LTrq=SYSINVAR2/1000
  local LV=SYSINVAR3
  if(LV<1 or LV>100)then
    LV=PROBE_V_DEFAULT
  end
  ShowInfo("torque probe: request " .. LReq .. " w=" .. LW .. " trq=" .. LTrq .. " v=" .. LV)
  if(LReq==21)then
    if(LW<0 or LW>40)then
      probe_Reply(-1,11)
      return
    end
    probe_GripTo(LW,LV)
    DelayMS(PROBE_SETTLE_MS)
    local LDet=GripperEmptyDet(1,LTrq,PROBE_DET_DIS)
    probe_Publish()
    probe_Reply(1,LDet)
  elseif(LReq==22)then
    if(LW<0 or LW>40)then
      probe_Reply(-1,11)
      return
    end
    probe_GripTo(LW,LV)
    probe_Reply(1,0)
  elseif(LReq==23)then
    if(LW<0 or LW>40)then
      probe_Reply(-1,11)
      return
    end
    if(LTrq<=0 or LTrq>0.5)then
      probe_Reply(-1,12)
      return
    end
    local LJ=GetCJoints()
    local LTarget=DefArray(LW,LJ._ext2,0,0,0,0)
    local LLimit=DefArray(LTrq,1,1,1,1,1)
    MoveExtJWithForce(LTarget,LV,LLimit)
    MOVFINISH()
    DelayMS(PROBE_SETTLE_MS)
    local LN=GetCJointsNormTrq()
    probe_Publish()
    probe_Reply(1,probe_Round(LN._ext1*1000))
  elseif(LReq==24)then
    if(LTrq<=0 or LTrq>1)then
      probe_Reply(-1,12)
      return
    end
    SetGripperTorqueLimit(1,1,LTrq)
    probe_Reply(1,0)
  elseif(LReq==25)then
    local LPeak=0
    local i=0
    while(i<50)do
      local LN=GetCJointsNormTrq()
      local LT=LN._ext1
      if(LT<0)then
        LT=-LT
      end
      if(LT>LPeak)then
        LPeak=LT
      end
      DelayMS(20)
      i=i+1
    end
    probe_Reply(1,probe_Round(LPeak*1000))
  end
end--functionend
--Fileinfomation:
