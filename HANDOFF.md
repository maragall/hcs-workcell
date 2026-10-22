# Handoff to the cell PC (the computer with the Polar arm, the vision box and Squid)

Read this first, then follow `docs/manual/hcs-workcell-manual.pdf` page by page. The PDF is for the operator; this
file is for the person or the AI session setting the PC up.

## 1. Get the code

```
git clone https://github.com/maragall/hcs-workcell.git   # or: git pull, if it is already there
cd hcs-workcell
pip install -r requirements.txt
pip install <the Bioyond SDK wheel, bioyondsdk-0.1.1>      # needs the .NET 8 runtime
python -m pytest tests -q                                   # 60+ tests, no hardware, under a minute
```

## 2. The marker is the key to calibrating fast

The vision box reads an ArUco marker (the 40 mm tab from the kit, drawing `aruco标定板`: two countersunk holes
30 mm apart). The tab screws onto the practice plate (drawing `视觉标定板带aruco v3.0`). Once the box knows how the
gripper sits relative to that marker, every position is taught by putting the practice plate where a plate will
rest and hovering the camera over it: the box computes the grip pose. No precise seating by hand.

The cameras have no focus ring. The camera check is the box reading the marker on the practice plate. Calibration
(the plate captures, the hand captures, the upload, the check, the teach-free relation for both grips) is done in our
setup program's Cameras tab, not with the vendor tool.

Our side: `cli.py record-pose NAME` stores a position (and the box's teach point), `cli.py record-trigger` stores
the marker view for it, `cli.py marker-check NAME` reads how far a position has drifted. The box is addressed with
`aruco_marker_id 1` in `layout.json` (the tab decodes as DICT_ARUCO_ORIGINAL id 682 in OpenCV; the box numbers
markers its own way and 1 is what works on this cell).

## 3. In this order on the cell

1. `python cli.py forget-taught` first: every pose, route, grip width and learned correction in `layout.json` was
   taught on the practice bench and is wrong for yours. Be blind to them. Then `python cli.py status` and
   `python cli.py preflight`: preflight refuses until the numbers below exist; that is the list of what to do.
2. New fingers: `python cli.py measure-grip short_edges` and `long_edges`; the held-plate photos are retaken on the
   first runs. Re-teach the `PLATE_*` values in `lua/data_plate.lua` only if the controller's Lua path is used.
3. The 30-slot hotel: measure `plate.height` and the finger stand-off, set `fixtures.hotel.lift_mm` below the 17 mm
   headroom, re-teach A1, A10, C1, C10. `python cli.py positions` shows every position and what is NOT TAUGHT.
4. `routes.safe_z_mm`: a height that clears the hotel, the stage and the holder.
5. The microscope: `python cli.py record-pose microscope`, its region box, the route to the stage.
6. One supervised cycle: `python cli.py run --mode supervised --slot A1 --cycles 1`.
7. `python exercise.py --mode supervised`: every slot, the stage, the trip on purpose, then the plates in turn.
8. The setup program: `python gui.py setup` (Setup, Teach, Test), the PDF's pages. Then `python gui.py run`.

## 4. Squid

`devices/squid.py` calls Squid's `Microscope.home_xyz`, `MultiPointController.set_base_path`, `start_new_experiment`,
`run_acquisition`, and hears the end through `signal_acquisition_finished` (confirmed against Cephla-Lab/Squid commit
489bc836). Build Squid's `Microscope` and `MultiPointController` where Squid runs and pass them to `Squid(...)`;
`layout.json squid.<name>.base_path` is where datasets go. Until then the fake microscope runs the loop.

## 5. When something stops

The arm holds still and one line says why. Send that line and the run folder's `events.jsonl` back. The usual three:
a slot that looks taken (the arm already backs out and uses another), a plate that does not look seated, a position
that drifted (teach it again).

## 6. What was never run on hardware

Everything after the 2026-10-22 refactor: the four packages, the asyncio loop, the occupancy detector, the front view,
the stage check, the planner, the hub recording, the plate order, the speed hacks, the robustness fixes, the GUI.
All of it passes offline on the fakes. The first real cycle (step 6) is the test of the refactor.
