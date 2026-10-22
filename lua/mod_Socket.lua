function mod_Socket()
  mod_Init()
  while(1==1)do
    ActionProcessedVar=0
    GetSysVarI()
    if(SYSINVAR0~=0)then
        ShowInfo("SYSINVAR0=" .. SYSINVAR0 .. " 1=" .. SYSINVAR1 .. " 2=" .. SYSINVAR2 .. " 3=" .. SYSINVAR3 .. " 4=" .. SYSINVAR4 .. " 5=" .. SYSINVAR5 .. " 6=" .. SYSINVAR6 .. " 7=" .. SYSINVAR7 .. " 8=" .. SYSINVAR8)
    end

    -- template devices (delete these two branches if mod_Stack/modScan are not in the project)
    if(SYSINVAR0==1 and SYSINVAR1>=1 and SYSINVAR1<=7 and SYSINVAR2>=1 and SYSINVAR2<=3)then
        mod_Stack()
        mod_End()
        ActionProcessedVar=1
    elseif(SYSINVAR0==2)then
        modScan()
        mod_End()
        ActionProcessedVar=1
    -- plate-transfer cell: 10 microscope, 11 polar, 12 hotel.
    -- mod_Cell validates the rest of the request itself and mod_CellEnd
    -- reports 1 / -1 / -2 / -3 and zeroes SYSINVAR0..8, so mod_End is not used.
    elseif(SYSINVAR0>=10 and SYSINVAR0<=12)then
        mod_Cell()
        mod_CellEnd()
        ActionProcessedVar=1
    end
    DelayMS(100)
    -- unknown device id: the template's -1 reply (threshold raised past 12)
    if(SYSINVAR0>12 and ActionProcessedVar==0)then
        mod_EndInvalid()
        DelayMS(500)
    end
  end
end--functionend
--Fileinfomation:
