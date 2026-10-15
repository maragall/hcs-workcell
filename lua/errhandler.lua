function errhandler()
  -- EnableGripperDet/DisableGripperDet must always pair (manual B.1.2)
  DisableGripperDet(1)
  CellDetOn=0
  -- a job interrupted by an error must not resume its queued moves
  CellAbort=1
  ShowInfo("errhandler: gripper detection off, cell job aborted")
end--functionend
--Fileinfomation:
