function mod_Init()
    --lua program status init
    SetLuaAppSta(0)
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
    --Cell lift override
    SetSysInVarI(7,0)
    --Cell plate type
    SetSysInVarI(8,0)
    --Reset system output variables
    ResetSysOutVarI()
    --a program restart loses the carry state: start with detection off
    DisableGripperDet(1)
    CellDetOn=0
    SetLuaAppSta(1)
end --functionend
--Fileinfomation:
