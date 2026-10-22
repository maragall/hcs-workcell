function data_plate()
  --==================================================================
  -- Plate types: jaw positions and grab height per plate, chosen by
  -- SYSINVAR8 (0 = default = type 1). Called by mod_Cell() right after
  -- data_cell(). Same idea as the template's mod_GetObjTypeProperty
  -- (SYSINVAR5 there, VAR3/VAR4 written into element 7 of every target,
  -- VAR7 added to the seat Z). Values here are E1 pendant readings, as
  -- everywhere else in data_cell.lua; the template's own formula is
  -- E1 = (plate opening - 76) / 2.
  -- Unknown type: PlateKnown=0 and the job is refused with -1/20.
  -- layout.json plates.<name> must hold the same numbers (id = type).
  --==================================================================
  PlateKnown=1
  PlateType=SYSINVAR8
  if(PlateType==0)then
    PlateType=1
  end
  if(PlateType==1)then
    -- production plate, layout.json plates.production (127.46 x 85.14 mm)
    PLATE_SHORT_OPEN=31.0    -- TODO TEACH (was GRIP_SHORT_OPEN, ~138 mm opening)
    PLATE_SHORT_CLOSE=24.0   -- TODO TEACH (stall on 127.8 mm plate ~25.9 -> 2 mm lower)
    PLATE_LONG_OPEN=10.0     -- TODO TEACH (layout.json gripper.open)
    PLATE_LONG_CLOSE=1.0     -- TODO TEACH (layout.json closed_on_plate 2.92 -> 2 mm lower)
    PLATE_GRAB_MM=0          -- added to the seat point Z before every pick/place
    PLATE_HEIGHT_MM=0        -- TODO MEASURE (layout.json plate.height)
  else
    -- next plate type: copy the block above as elseif(PlateType==2), teach, add it to layout.json
    PlateKnown=0
  end
end--functionend
--Fileinfomation:
