# Polar workcell

Setup script for the Bioyond Polar arm with a plate hotel and a rotation module.

## Prerequisites

- BioyondWorks, installed on the PC.
- The Bioyond Python package (a .whl file). Contact us or Bioyond to get it.
- Python 3.10 or newer.
- Open PowerShell and `cd` into this folder, the one that holds `cli.py`. Every command below is run from there.
- Run `Set-ExecutionPolicy -Scope Process Bypass`. Windows blocks scripts by default; this allows them in this PowerShell window only. Run it again in every new window.
- Run `.\setup\install.ps1 -SdkWheel <path to the .whl file>`, then `.\.venv\Scripts\Activate.ps1`.
- The camera box login, in a file `nova_ssh.local.json` in the top folder: `{"user": "...", "password": "..."}`.
- Close BioyondWorks.

## Hardware

1. Connect the power cable and the emergency stop to the arm. Turn on the main switch at the back. Wait until the status light is steady.
2. Connect the PC to port 1 of the controller with a network cable.
3. Screw the hotel onto the platform. The slots face the arm. Leave it empty.
4. Place the ArUco plate on the rotation module.

## Setup

Run `python cli.py setup`. The script ends with `READY`.

The script asks you to show the arm eight places with the ArUco plate: the rotation module twice (short edges, then long edges), then slots A1, A9, B1, B9, C1 and C9.

Each time:

1. The arm goes loose.
2. Fit the gripper onto the plate until the finger pads sit in the plate's grooves.
3. Press Enter.

<img src="docs/img/teach_slot_front_camera.jpg" width="400" alt="gripper camera image of the plate seating in slot A9">

Gripper camera image of the plate seating in slot A9.

Then load the plates into the hotel. Start at A1.

![hotel](docs/img/hotel_matrix.png)

## Setup script in a nutshell

- User shows the arm eight places with the ArUco plate:
    - rotation module seats, short edges and long edges; the arm then finds each seat's angle by force
    - six hotel slots; the hotel is fitted through them
- Planner: routes from each slot ↔ rotation module
- Upload to the arm:
    - hotel slot positions
    - rotation module seat positions
    - long-edge grip, short-edge grip, and grip torque probe procedure
    - hotel slot exit/entry procedure
    - routes from each slot ↔ rotation module
- Calibrate cameras:
    - Run aruco intrinsic and extrinsic calibration
    - Upload to the camera box

# Teaching

## Hardware

![workcell with microscope](docs/img/cell_model_microscope.png)

1. Place the microscope next to the platform, at least 20 cm from the arm's base. The plate holder must be within 39 cm of the arm's axis.
2. Start Squid. Tick Settings > Enable MCP Control Server and Settings > Enable MCP Python Exec.
3. Place a real plate in the microscope's plate holder.

## Teaching

1. Run `python cli.py teach`. The script sends the microscope's stage to Squid's loading position, then the arm goes loose.
2. Move the arm by hand until the gripper grabs the plate by its short edges.
3. Press Enter. The script ends with `READY`.

## Teaching and route planning script in a nutshell

- User teaches microscope seat
    - gripper grabbing a real plate by short edges
- Planner: microscope seat ↔ rotation module routes
- Upload routes and microscope seat to the arm

# Running jobs

## Demo

Run `python demo.py`. The arm takes the plate from A1 to the microscope and back.

## Your own script

```python
from polar import Polar

arm = Polar(plate_at="A1")

arm.move("A1", "microscope")
arm.move("microscope", "A1")
```

- `arm.move(source, destination)` picks the plate at the source and places it at the destination.
- `arm.pick(place)` and `arm.place(place)` do one half each.
- `plate_at` says where the plate is when the script starts. Leave it out and the arm remembers from the last run.
- A place is a hotel slot (`"A1"` to `"B9"`), `"module"`, or the name of an instrument you taught. Column C does not work yet.
- Each call returns when the arm is done. It raises an error with the reason if it cannot do it.

## With a running microscope

In Squid, tick Settings > Enable MCP Control Server and Settings > Enable MCP Python Exec.

```python
from polar import Polar, Microscope

arm = Polar(plate_at="microscope")
scope = Microscope()

arm.move("microscope", "A1")
arm.move("A3", "microscope")
scope.acquire("acquisition.yaml")
```

- Before the arm goes to the microscope, it waits for the acquisition to finish and sends the stage to Squid's loading position.
- After the arm has placed a plate there, the stage goes back to where the acquisition ended.
- `scope.acquire(path)` starts an acquisition saved by Squid (`acquisition.yaml`).
