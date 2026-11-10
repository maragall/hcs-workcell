# Setup

## Prerequisites

- BioyondWorks, installed on the PC.
- The Bioyond Python package (a .whl file). Contact us or Bioyond to get it.
- Python 3.10 or newer.
- Run `.\install.ps1 -SdkWheel <path to the .whl file>`.
- Close BioyondWorks.

## Hardware

![workcell](img/cell_model.png)

1. Connect the power cable and the emergency stop to the arm. Turn on the main switch at the back. Wait until the status light is steady.
2. Connect the PC to port 1 of the controller with a network cable.
3. Screw the hotel onto the platform. The slots face the arm.
4. Load the plates into the hotel. Start at A1.

![hotel](img/hotel_matrix.png)

5. Place the ArUco plate on the rotation module.

## Setup

Run `python cli.py setup`. The script ends with `READY`.

## Setup script in a nutshell

- Upload some preset knowledge to the arm:
    - hotel slot positions
    - rotation module seat position
    - long-edge grip, short-edge grip, and grip torque probe procedure
    - routes from each slot ↔ rotation module
    - slot-exit procedure
- Calibrate cameras:
    - Run aruco intrinsic and extrinsic calibration
    - Upload to the camera box
