# Replicating the Nova plate-transfer cell

This is the complete procedure to rebuild the cell from hardware to an unattended loop, as done in
Shanghai in October 2026. Follow it in order. Every number that depends on your installation lives in
`layout.json`; nothing in the code is specific to one site.

## 1. Hardware

| Item | Used here | Notes |
|---|---|---|
| Robot | Bioyond Nova / LabGenius S4 SCARA (S4-1.5-0.6-Z0.8), firmware 1.14 | 4 joints + 2 external axes: E1 gripper, E2 rotary platform |
| Gripper | vendor servo gripper on external axis E1 | position controlled, no force sensing through the SDK |
| Rotating stage | vendor rotary platform on external axis E2 | 0 / −90 degrees in this cell |
| Vision box | Raspberry Pi 5, VIBVision service build 20250929 | two IMX219 cameras on the gripper: `auto_teach_camera` looks down, 90 mm behind the fingertips; `scan_camera` looks forward |
| Plate hotel | 3 columns A B C, 10 rows at 37.5 mm pitch, plates on odd rows | slots are approached from −x |
| Microscope stage | holder accepting an SBS plate on its short edges | |
| PC | Windows 11, Python 3.12, .NET 8 runtime | one PC runs everything |
| Network | robot controller 192.168.1.220, vision box 192.168.3.100 | two subnets on this PC's adapters |

Calibration plate for commissioning: any SBS plate. Production plate widths are re-measured in two minutes
(section 6).

## 2. Software on the PC

```
git clone <this repo> nova_cell
cd nova_cell
pip install -r requirements.txt
pip install pythonnet
pip install --no-deps "<vendor>/Nova SDK/Python/bioyondsdk-0.1.1-py3-none-any.whl"
dotnet --list-runtimes          # needs Microsoft.NETCore.App 8.x
python tests/test_sim.py        # 22 offline tests, no hardware needed
```

The vendor Python wheel is only used by `devices/arm.py`. The vision box is driven over its documented TCP
text protocol by `devices/vision_box.py`, no vendor code.

Optional: `winget install Gyan.FFmpeg` to turn recorded frames into mp4.

## 3. Interfaces you must know

- **Robot SDK** (port 52525): see `docs/VENDOR_SDK.md`. Key facts: external-axis moves are 1-based, readouts
  0-based; every move's completion flag is unreliable while a plate is held, so the adapter verifies by
  readback; drag mode snaps the gripper to its last held value.
- **Controller protocol BRCAPI** (port 56788, JSON lines): read-only use today (state, errors, pose). The
  motion-in-Lua path is prepared in `lua/` and `devices/controller.py` but not yet deployed.
- **Vision box** (port 9000, text lines ending in `\r`): `get_frame`, `get_aruco_pose`, `get_pose_offset`,
  `run_live_detection`, `scan`, LED control, NG events, and the no-teach workspace (teach points, trigger points,
  relations). Its calibration handlers are broken in build 20250929 (`'str' object has no attribute 'sendall'`);
  `nova_vision_proxy.py` in the parent folder works around that for the vendor GUI only.

## 4. Safety configuration before any motion

1. On the pendant software (BioyondWorks): collision detection enabled on every joint, lowest trigger
   speed, highest sensitivity. Only editable while disabled. Record the values in `layout.json` under a note.
2. Payload set to the gripper plus a plate.
3. In `layout.json`: `workspace_box`, per-region `z_floor`, and the microscope region marked `no_swing`.
   The code refuses any target outside the box or below a floor, and any joint move or yaw change inside a
   no-swing region.
4. Speeds start at the `supervised` profile (20/10/5 %). Raise only after the validation ladder in section 8.

## 5. Teaching poses (operator at the cell, 30 minutes)

Drag mode is for hovers and seats only. Commands are read-only except where stated.

```
python cli.py status
python cli.py record-pose microscope            # plate seated on the holder, SHORT edges in the fingers
python cli.py record-pose microscope.hover --local-only   # straight up, 60 to 70 mm
python cli.py polar jog +5 / polar zero          # turn the stage until parallel to x, then zero it
python cli.py record-pose polar.pose_at_0        # plate seated on the stage at 0, short edges
python cli.py record-pose polar.hover --local-only
python cli.py polar goto 90                      # stage to the long-edge orientation
python cli.py record-pose polar.pose_at_90       # regrip on the LONG edges, seated
```

Hotel: teach the bottom and top slot of each column by hand (`fixtures.hotel.taught`); the rest are
interpolated. Verify the first slot with `place-test` stage by stage.

Each `record-pose` also stores the pose on the vision box as a vendor teach point (`microscope-short`,
`polar0-short`, `polar90-wide`), which is what the vendor's offset functions use later.

## 6. Gripper widths (two minutes)

```
python cli.py measure-grip short_edges      # closes in steps until the servo refuses: contact
python cli.py measure-grip long_edges
```
Values are the finger gap in the gripper's units; larger is wider. Measured here: 24.0 for the plate's 128 mm
length, 2.84 for its 85 mm width, spread over a thousand grasps under 0.01.

## 7. Routes (operator drags, code normalises)

The controller refuses long direct moves between regions, so two routes are hand-taught:

```
python cli.py trace microscope_to_polar     # drag from the microscope hover to the stage hover; stop: create runs/trace.stop
python cli.py normalise microscope_to_polar
python cli.py trace polar_to_hotel          # from above the stage seat to in front of the hotel
python cli.py normalise polar_to_hotel
```
Rules for the drag: lift before moving sideways, do the 180 degree claw turn away from fixtures, arrive at
the microscope hover from straight above. The normaliser keeps joints and poses aligned, thins to a waypoint
list, and refuses joint steps above 25 degrees. Replay with `python cli.py replay <name> --step`.

## 8. Validation ladder

1. `python cli.py place-test A1 --stage 1..4`: route, pre-approach, slide, lower, with a look between stages.
2. `python cli.py hotel-cycle-test A1`: pick, place, pick at the slot, return, place on the stage.
3. `python cli.py run --mode supervised --slot A1 --cycles 1 --no-gates --no-vision-correction --allow-no-tty --yes --set-state microscope --record`
4. Three cycles at `--profile fast`, then three at `--profile stress`.
5. Unattended: `python run_forever.py A1 --first-state microscope` (see section 10).

Every stage must end with zero safe stops and grasp values inside `gripper.tolerance`.

## 9. What the loop does each cycle

microscope pick (short edges) → route B → place on the stage at 0 → stage to −90 → pick on the long edges →
route A → hotel slot place → hotel pick → route A reversed → place on the stage at −90 → stage to 0 → pick on
the short edges → route B reversed → place on the microscope. State is written before and after every step;
an interrupted carry resumes from the nearest route waypoint (`--resume`).

## 10. Unattended operation

`run_forever.py` relaunches the cycle only after recoverable stops (vision outage, lost enable while
stationary, SDK connection drop), at most 3 per hour and 10 per day. Contact, refused moves, grasp failures,
drag mode, low disk, or an unknown plate location leave the arm in place and write `runs/ALERT_SUPERVISOR.txt`.
Stop cleanly by creating `runs/STOP`. Recording is a ring buffer (`--record-max-gb`).

Results of the first unattended run (90 h, 492 cycles, no contact) are in `ANALYSIS.md`.

## 11. Data

`runs/<timestamp>/events.jsonl` has one JSON record per move, jaw action, vision call, health check, fault
and cycle, with the feature keys listed in `DESIGN.md` section 5. `images/` holds per-step frames, `video/`
the continuous recording. `tools/analyze_runs.py` aggregates; `tools/build_refs_from_runs.py` labels frames for the gates.

## 12. Known limits at the time of writing

- The vision gates are implemented but blind from the taught hovers (camera geometry). A fixed external camera
  on the microscope holder, or inspection poses, is required before "vision-guided" is true.
- Hotel rows above the first need a retreat-then-climb approach; the rack's front-face x must be measured.
- Hand-eye calibration on the vision box carries an unexplained error figure; ArUco offsets are unverified.
- The vendor SDK stops a move when gripper jitter trips its own check; the adapter re-issues once when enable
  is intact. Moving motion into the controller's Lua program removes this class of fault.
