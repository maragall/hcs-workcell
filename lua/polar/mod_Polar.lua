function mod_Polar()
  --==================================================================
  -- Polar: picks, places and module turns run on the controller.
  -- The numbers (places, seats, grips, routes) are the table POLAR in
  -- data.lua, written by run/lua.py from layout.json and
  -- preset/routes.json. Nothing here is taught or edited by hand.
  --
  -- The loop is the torque probe's (mod_TorqueProbe.lua): it publishes
  -- the jaw and motor torques ten times a second and serves the jaw
  -- requests 21..25, so the PC's force close (devices/clamp.py) works
  -- unchanged. Added requests, written by the PC to SYSINVAR0..3:
  --   31 pick   SYSINVAR1 = place id, SYSINVAR2 = grip (module only)
  --   32 place  SYSINVAR1 = place id, SYSINVAR2 = grip in hand (module only)
  --   33 turn the rotation module to SYSINVAR1 degrees (0 or 90)
  --   34 speed test: SYSINVAR1 = class (1 travel, 2 approach, 3 contact),
  --      SYSINVAR2 = millimetres straight up and back (1..50)
  --   SYSINVAR3 = speed profile (index into POLAR.profiles, 0 = POLAR.profile)
  -- Place id: 1 the rotation module, 2.. instruments, column*10+row a slot.
  -- Grip: 1 short edges, 2 long edges.
  -- Every job starts and ends at the module's hover.
  -- Answer: SendSysOutVarI(1, code), SendSysOutVarI(2, detail).
  --   code  1 done; -1 refused, nothing moved; -2 stopped part way, the
  --         arm is left where it stopped.
  --   detail 11 unknown place or value, 12 unknown grip, 13 no stored
  --         route, 16 the arm is not at the module's hover, 21 nothing
  --         clamped in two closes, 22 the held width left its band,
  --         23 the plate sits crooked in the jaw, 24 the claw is not
  --         square to the slot, 25 the jaw did not open, 26 the module
  --         did not reach its angle.
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
  POLAR_CODE=1
  POLAR_DETAIL=0
  POLAR_CLASS=0
  POLAR_E1=0
  POLAR_E2=0
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
    elseif(SYSINVAR0>=31 and SYSINVAR0<=34)then
      polar_Job()
    elseif(SYSINVAR0~=0)then
      ShowInfo("polar: unknown request " .. SYSINVAR0)
      probe_Reply(-1,0)
    end
    DelayMS(100)
  end
end--functionend

function polar_Job()
  local LReq=SYSINVAR0
  local LA=SYSINVAR1
  local LB=SYSINVAR2
  local LProfile=POLAR.profiles[SYSINVAR3]
  if(LProfile==nil)then
    LProfile=POLAR.profiles[POLAR.profile]
  end
  POLAR_SCALE=LProfile
  POLAR_CODE=1
  POLAR_DETAIL=0
  POLAR_CLASS=0
  ShowInfo("polar: request " .. LReq .. " a=" .. LA .. " b=" .. LB)
  MOVFINISH()
  polar_Sync()
  if(LReq==31 or LReq==32)then
    local LPlace=POLAR.place[LA]
    if(LPlace==nil)then
      probe_Reply(-1,11)
      return
    end
    if(polar_AtHover()==0)then
      probe_Reply(-1,16)
      return
    end
    if(LPlace.kind==0)then
      local LSeat=POLAR.module.seat[LB]
      local LGrip=POLAR.grip[LB]
      if(LSeat==nil or LGrip==nil)then
        probe_Reply(-1,12)
        return
      end
      if(LReq==31)then
        polar_SeatPick(LSeat.pose,LGrip,POLAR.module.hover,LSeat.open,LSeat.deg)
      else
        polar_SeatPlace(LSeat.pose,LGrip,POLAR.module.hover,LSeat.open,LSeat.lift,LSeat.deg)
      end
    elseif(LPlace.kind==1)then
      if(LPlace.go_empty==nil or LPlace.back_empty==nil or LPlace.go_plate==nil or LPlace.back_plate==nil)then
        probe_Reply(-1,13)
        return
      end
      polar_L(POLAR.module.hover90,2,-1)
      if(LReq==31)then
        polar_SlotPick(LPlace)
      else
        polar_SlotPlace(LPlace)
      end
    else
      if(LPlace.go_empty==nil or LPlace.back_empty==nil or LPlace.go_plate==nil or LPlace.back_plate==nil)then
        probe_Reply(-1,13)
        return
      end
      local LGrip=POLAR.grip[1]
      polar_L(POLAR.module.hover0,2,-1)
      if(LReq==31)then
        polar_Run(LPlace.go_empty)
        polar_SeatPick(LPlace.seat,LGrip,LPlace.hover,LPlace.open,nil)
        polar_Run(LPlace.back_plate)
      else
        polar_Run(LPlace.go_plate)
        polar_SeatPlace(LPlace.seat,LGrip,LPlace.hover,LPlace.open,LPlace.lift,nil)
        polar_Run(LPlace.back_empty)
      end
    end
  elseif(LReq==33)then
    if(POLAR.module.reading[LA]==nil)then
      probe_Reply(-1,11)
      return
    end
    if(polar_AtHover()==0)then
      probe_Reply(-1,16)
      return
    end
    local LHere=GetCRobT(T0,U0)
    polar_L({LHere._x,LHere._y,POLAR.module.hover90[3]+POLAR.module.clear,LHere._rx,LHere._ry,LHere._rz},2,-1)
    polar_Turn(LA)
  else
    if(LA<1 or LA>3 or LB<1 or LB>50)then
      probe_Reply(-1,11)
      return
    end
    local LHere=GetCRobT(T0,U0)
    polar_L({LHere._x,LHere._y,LHere._z+LB,LHere._rx,LHere._ry,LHere._rz},LA,-1)
    MOVFINISH()
    polar_L({LHere._x,LHere._y,LHere._z,LHere._rx,LHere._ry,LHere._rz},LA,-1)
  end
  MOVFINISH()
  probe_Reply(POLAR_CODE,POLAR_DETAIL)
end--functionend

function polar_Fail(code,detail)
  -- the first failure of a job is the one reported; every later step of the job does nothing
  if(POLAR_CODE==1)then
    POLAR_CODE=code
    POLAR_DETAIL=detail
    ShowInfo("polar: stopped, code=" .. code .. " detail=" .. detail)
  end
end--functionend

function polar_Abs(x)
  if(x<0)then
    return -x
  end
  return x
end--functionend

function polar_Sync()
  -- the jaw and the module as last commanded: every arm move carries these, so neither moves with the arm
  local LRef=GetCRefJoints()
  POLAR_E1=LRef._ext1
  POLAR_E2=LRef._ext2
end--functionend

function polar_Speed(class)
  -- the speed scale changes only with the arm at rest
  if(class~=POLAR_CLASS)then
    MOVFINISH()
    SetSpeedScale(POLAR_SCALE[class])
    POLAR_CLASS=class
  end
end--functionend

function polar_L(p,class,zone)
  -- one straight move to x y z rx ry rz
  if(POLAR_CODE~=1)then
    return
  end
  polar_Speed(class)
  MoveL(RobTarget(p[1],p[2],p[3],p[4],p[5],p[6],POLAR_E1,POLAR_E2,0,0,0,0),POLAR.v_linear,zone,T0,U0)
end--functionend

function polar_J(j,class,zone)
  -- one joint move to the planner's own joints
  if(POLAR_CODE~=1)then
    return
  end
  polar_Speed(class)
  MoveAbsJ(JointTarget(j[1],j[2],j[3],j[4],0,0,POLAR_E1,POLAR_E2,0,0,0,0),POLAR.v_joint,zone)
end--functionend

function polar_Run(route)
  -- a stored route: each step is {1, x, y, z, rx, ry, rz} (straight) or {2, j1, j2, j3, j4} (joints).
  -- Two straight steps in a row are blended by POLAR.zone; a joint step and the last step end at rest.
  if(POLAR_CODE~=1)then
    return
  end
  local n=#route
  for i=1,n do
    local s=route[i]
    local z=-1
    if(i<n and s[1]==1 and route[i+1][1]==1)then
      z=POLAR.zone
    end
    if(s[1]==2)then
      polar_J({s[2],s[3],s[4],s[5]},1,z)
    else
      polar_L({s[2],s[3],s[4],s[5],s[6],s[7]},1,z)
    end
  end
  MOVFINISH()
end--functionend

function polar_Contact(from,to)
  -- to a seat or into a slot: approach speed until slow_mm before the end, contact speed for the rest
  if(POLAR_CODE~=1)then
    return
  end
  local dx=to[1]-from[1]
  local dy=to[2]-from[2]
  local dz=to[3]-from[3]
  local d=polar_Abs(dx)
  if(polar_Abs(dy)>d)then
    d=polar_Abs(dy)
  end
  if(polar_Abs(dz)>d)then
    d=polar_Abs(dz)
  end
  if(d>POLAR.slow_mm+1.0)then
    local f=(d-POLAR.slow_mm)/d
    polar_L({from[1]+dx*f,from[2]+dy*f,from[3]+dz*f,to[4],to[5],to[6]},2,-1)
  end
  polar_L(to,3,-1)
  MOVFINISH()
end--functionend

function polar_Ext(e1,e2,v)
  -- the jaw and the module only, on a snapshot of the commanded joints: the arm does not move
  MOVFINISH()
  local LJ=GetCRefJoints()
  LJ._ext1=e1
  LJ._ext2=e2
  MoveExtJ(LJ,v,0,T0,U0)
  MOVFINISH()
  polar_Sync()
end--functionend

function polar_Read()
  -- the jaw's width and the size of its torque, five readings 100 ms apart
  MOVFINISH()
  local w=0
  local t=0
  for i=1,5 do
    DelayMS(100)
    local LJ=GetCJoints()
    local LN=GetCJointsNormTrq()
    w=w+LJ._ext1
    t=t+polar_Abs(LN._ext1)
  end
  return w/5,t/5
end--functionend

function polar_Open(width)
  if(POLAR_CODE~=1)then
    return
  end
  polar_Ext(width,POLAR_E2,POLAR.jaw_v)
  DelayMS(500)
  local LJ=GetCJoints()
  if(LJ._ext1<width-1.0)then
    polar_Fail(-2,25)
  end
end--functionend

function polar_Close(grip,open)
  -- force close: toward the command under the torque limit; a plate stops the jaw above the command with the torque at the limit
  if(POLAR_CODE~=1)then
    return
  end
  local try=1
  while(try<=2)do
    MOVFINISH()
    local LJ=GetCJoints()
    MoveExtJWithForce(DefArray(grip.closed,LJ._ext2,0,0,0,0),POLAR.clamp_v,DefArray(POLAR.clamp_limit,1,1,1,1,1))
    MOVFINISH()
    DelayMS(300)
    polar_Sync()
    local w,t=polar_Read()
    if(t>=0.9*POLAR.clamp_limit and w>=grip.closed+0.3 and polar_Abs(w-grip.width)<=1.0)then
      return
    end
    ShowInfo("polar: close " .. try .. " did not hold, width=" .. w .. " torque=" .. t)
    polar_Ext(open,POLAR_E2,POLAR.jaw_v)
    try=try+1
  end
  polar_Fail(-2,21)
end--functionend

function polar_Carry(grip)
  -- the plate is still between the fingers: the jaw reads inside the grip's band
  if(POLAR_CODE~=1)then
    return
  end
  local w,t=polar_Read()
  if(w<grip.band[1] or w>grip.band[2])then
    polar_Fail(-2,22)
  end
end--functionend

function polar_Crooked(grip)
  -- before a slide into a slot: a plate turned in the jaw reads wider
  if(POLAR_CODE~=1)then
    return
  end
  local w,t=polar_Read()
  if(w-grip.width>POLAR.crooked_mm)then
    polar_Fail(-2,23)
  end
end--functionend

function polar_YawOff(p)
  MOVFINISH()
  local LHere=GetCRobT(T0,U0)
  local d=LHere._rz-p[6]
  while(d>180)do
    d=d-360
  end
  while(d<-180)do
    d=d+360
  end
  local dxy=polar_Abs(LHere._x-p[1])
  if(polar_Abs(LHere._y-p[2])>dxy)then
    dxy=polar_Abs(LHere._y-p[2])
  end
  return polar_Abs(d),dxy
end--functionend

function polar_Square(p)
  -- before a slide into a slot: the claw square to it and on its line, else one slow straight move to the exact pose
  if(POLAR_CODE~=1)then
    return
  end
  local dyaw,dxy=polar_YawOff(p)
  if(dyaw>0.1 or dxy>0.5)then
    polar_L(p,3,-1)
    dyaw,dxy=polar_YawOff(p)
  end
  if(dyaw>0.25)then
    polar_Fail(-2,24)
  end
end--functionend

function polar_AtHover()
  -- 1 when the wrist is over the module: within 15 mm of its hover sideways, not below it, turned as it is there
  local LHere=GetCRobT(T0,U0)
  local h=POLAR.module.hover90
  local d=LHere._rz-h[6]
  while(d>180)do
    d=d-360
  end
  while(d<-180)do
    d=d+360
  end
  if(polar_Abs(LHere._x-h[1])>15 or polar_Abs(LHere._y-h[2])>15 or LHere._z<h[3]-3 or polar_Abs(d)>5)then
    return 0
  end
  return 1
end--functionend

function polar_Turn(deg)
  -- the module to an angle, by the axis reading for it; asked once more when it stops more than 0.1 deg off
  if(POLAR_CODE~=1)then
    return
  end
  local want=POLAR.module.reading[deg]
  local try=1
  local off=0
  while(try<=2)do
    polar_Ext(POLAR_E1,want,POLAR.module_v)
    DelayMS(POLAR.module_settle_ms)
    local LJ=GetCJoints()
    off=polar_Abs(LJ._ext2-want)
    if(off<=0.1)then
      return
    end
    try=try+1
  end
  if(off>0.5)then
    polar_Fail(-2,26)
  end
end--functionend

function polar_Ensure(deg,seat,hover)
  -- at the module's hover, before the lower: the module at the angle this seat is for, read from the axis
  if(POLAR_CODE~=1)then
    return
  end
  MOVFINISH()
  local LJ=GetCJoints()
  if(polar_Abs(LJ._ext2-POLAR.module.reading[deg])<=0.3)then
    return
  end
  polar_L({seat[1],seat[2],seat[3]+POLAR.module.hover+POLAR.module.clear,seat[4],seat[5],seat[6]},2,-1)
  polar_Turn(deg)
  polar_L(hover,2,-1)
end--functionend

function polar_SeatPick(seat,grip,hover_mm,open,deg)
  local LHover={seat[1],seat[2],seat[3]+hover_mm,seat[4],seat[5],seat[6]}
  polar_L(LHover,2,-1)
  if(deg~=nil)then
    polar_Ensure(deg,seat,LHover)
  end
  polar_Open(open)
  polar_Contact(LHover,seat)
  polar_Close(grip,open)
  polar_L(LHover,2,-1)
  polar_Carry(grip)
end--functionend

function polar_SeatPlace(seat,grip,hover_mm,open,lift,deg)
  -- the plate is let go `lift` above the height a pick lowers to
  local LHover={seat[1],seat[2],seat[3]+hover_mm,seat[4],seat[5],seat[6]}
  polar_L(LHover,2,-1)
  if(deg~=nil)then
    polar_Ensure(deg,seat,LHover)
  end
  polar_Carry(grip)
  polar_Contact(LHover,{seat[1],seat[2],seat[3]+lift,seat[4],seat[5],seat[6]})
  polar_Open(open)
  polar_L(LHover,2,-1)
end--functionend

function polar_SlotPick(slot)
  -- empty claw from the module to the slot's entry point, in, close, out, and back to the module with the plate
  local LGrip=POLAR.grip[2]
  polar_Run(slot.go_empty)
  polar_Square(slot.approach)
  polar_Open(LGrip.open)
  polar_Contact(slot.approach,slot.carry)
  polar_Contact(slot.carry,slot.seat)
  polar_Close(LGrip,LGrip.open)
  polar_L(slot.carry,2,-1)
  polar_Carry(LGrip)
  polar_L(slot.approach,2,-1)
  polar_L(slot.entry,2,-1)
  polar_Run(slot.back_plate)
end--functionend

function polar_SlotPlace(slot)
  -- the plate from the module to the slot's entry point, in, open, out, and back to the module empty
  local LGrip=POLAR.grip[2]
  polar_Run(slot.go_plate)
  polar_L(slot.approach,2,-1)
  polar_Square(slot.approach)
  polar_Carry(LGrip)
  polar_Crooked(LGrip)
  polar_Contact(slot.approach,slot.carry)
  polar_Contact(slot.carry,slot.seat)
  polar_Open(LGrip.open)
  polar_L(slot.carry,2,-1)
  polar_L(slot.approach,2,-1)
  polar_Run(slot.back_empty)
end--functionend
--Fileinfomation:
