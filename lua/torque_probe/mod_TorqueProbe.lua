function mod_TorqueProbe()
  --==================================================================
  -- Gripper (E1) force probe, 2026-10-20. Runs on the controller and
  -- publishes the jaw width and the servo torque through the integer
  -- system variables that the PC polls with getSysVarI (JSON port 56788).
  -- Manual refs: A.4.15 GetCJoints, A.4.18 GetCJointsTrq, A.4.19
  -- GetCJointsNormTrq, A.8 SetSysInVarI/SendSysOutVarI, A.10 gripper.
  --
  -- Published every loop (about 10 Hz):
  --   SYSINVAR4 = E1 raw torque   x1000 (GetCJointsTrq element 7)
  --   SYSINVAR5 = E1 width        x100  (GetCJoints element 7, pendant units = mm)
  --   SYSINVAR6 = E1 norm. torque x1000 (GetCJointsNormTrq element 7, 0..1000 = 0..100 %)
  --   SYSINVAR7 = heartbeat 0..9999
  -- Requests (PC writes SYSINVAR0..3; the probe zeroes SYSINVAR0 when done
  -- and answers SendSysOutVarI(1, code) then SendSysOutVarI(2, detail)):
  --   SYSINVAR0=21 close: MoveExtJ to SYSINVAR1/100 mm at speed SYSINVAR3 (1..100),
  --                settle, detail = GripperEmptyDet(1, SYSINVAR2/1000, PROBE_DET_DIS)
  --                (1 holding, 2 empty); the holding torque is then in SYSINVAR6.
  --   SYSINVAR0=22 open:  MoveExtJ to SYSINVAR1/100 mm at speed SYSINVAR3.
  --   SYSINVAR0=23 force close (EXPERIMENTAL, first try with the operator at the
  --                e-stop): MoveExtJWithForce({1}, SYSINVAR3, {SYSINVAR2/1000}).
  --                The sign convention of the torque is not documented; start with
  --                a small value (100 = 10 %) and watch which way the jaw goes.
  --   SYSINVAR0=24 set clamp force (torque control mode only, manual B.1.2):
  --                SetGripperTorqueLimit(1, 1, SYSINVAR2/1000).
  --   SYSINVAR0=25 torque sample burst: 50 samples of E1 norm torque at 20 ms
  --                while the PC does whatever it wants (e.g. pulls the plate);
  --                the peak goes to detail (x1000).
  -- Codes: 1 done, -1 bad request, -2 refused.
  --==================================================================
  SetLuaAppSta(0)
  SetSysInVarI(0,0)
  SetSysInVarI(1,0)
  SetSysInVarI(2,0)
  SetSysInVarI(3,0)
  ResetSysOutVarI()
  DisableGripperDet(1)
  PROBE_DET_DIS=0.5
  PROBE_SETTLE_MS=300
  PROBE_V_DEFAULT=50
  SetLuaAppSta(1)
  local Beat=0
  while(1==1)do
    GetSysVarI()
    local LJ=GetCJoints()
    local LT=GetCJointsTrq()
    local LN=GetCJointsNormTrq()
    SetSysInVarI(4,probe_Round(GetElem(LT,7)*1000))
    SetSysInVarI(5,probe_Round(GetElem(LJ,7)*100))
    SetSysInVarI(6,probe_Round(GetElem(LN,7)*1000))
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
  -- jaw-only move on a snapshot of the current joints (manual Appendix B idiom)
  local LJ=GetCJoints()
  SetElem(LJ,7,w)
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
    local LN=GetCJointsNormTrq()
    SetSysInVarI(6,probe_Round(GetElem(LN,7)*1000))
    probe_Reply(1,LDet)
  elseif(LReq==22)then
    if(LW<0 or LW>40)then
      probe_Reply(-1,11)
      return
    end
    probe_GripTo(LW,LV)
    probe_Reply(1,0)
  elseif(LReq==23)then
    if(LTrq<=0 or LTrq>0.5)then
      probe_Reply(-1,12)
      return
    end
    -- A.10.4 MoveExtJWithForce(NumArray, V, ForceArray): external axis motion under a torque setpoint
    MoveExtJWithForce({1},LV,{LTrq})
    MOVFINISH()
    DelayMS(PROBE_SETTLE_MS)
    local LN=GetCJointsNormTrq()
    probe_Reply(1,probe_Round(GetElem(LN,7)*1000))
  elseif(LReq==24)then
    if(LTrq<=0 or LTrq>1)then
      probe_Reply(-1,12)
      return
    end
    -- B.1.2 SetGripperTorqueLimit(N, DofIndex, Torque): torque control mode only, lost at power off
    SetGripperTorqueLimit(1,1,LTrq)
    probe_Reply(1,0)
  elseif(LReq==25)then
    local LPeak=0
    local i=0
    while(i<50)do
      local LN=GetCJointsNormTrq()
      local LT=GetElem(LN,7)
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
