"""
Fit the hand-taught hotel seats (fixtures.hotel.taught_detail, the raw records) to one regular grid and write it as the seat poses:
every seat has the same x and yaw 0 (the claw is square to the hotel), columns are `pitch` apart in y (B in the middle), rows are the
STEP's 38 mm tray pitch apart in z. The fit is robust (soft L1), so a seat taught off-centre does not drag the grid with it; the
residual of every seat is printed. `--apply` writes fixtures.hotel.taught (the raw records stay in taught_detail), fixtures.hotel.grid
and pose_source "grid".

    python tools/fit_hotel_grid.py [--apply]
"""
import os
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402

COL = {"A": 1, "B": 0, "C": -1}                  # column A is +y, as in the STEP (CAD x +134 = A)
ROW_PITCH = 38.0                                 # the STEP: ten trays 38 mm apart, the hotel uses every second one by the row numbers


def main(apply=False):
    layout = R.load_layout()
    h = layout["fixtures"]["hotel"]
    raw = {k: v["pose"] for k, v in h["taught_detail"].items() if k[0] in COL and k[1:].isdigit()}
    slots = sorted(raw)
    cols = np.array([COL[s[0]] for s in slots])
    rows = np.array([int(s[1:]) for s in slots])
    seen = np.array([raw[s][:3] for s in slots])

    def model(p):
        x0, y0, pitch, z0 = p
        return np.c_[np.full(len(slots), x0), y0 + pitch * cols, z0 + ROW_PITCH * (rows - 1)]

    fit = least_squares(lambda p: (model(p) - seen).ravel(), [291.0, 0.0, 134.0, 78.0], loss="soft_l1", f_scale=2.0)
    x0, y0, pitch, z0 = fit.x
    err = seen - model(fit.x)
    print(f"{len(slots)} seats: x {x0:.1f}, B at y {y0:.1f}, column pitch {pitch:.1f}, row 1 at z {z0:.1f}, row pitch {ROW_PITCH}, yaw 0")
    for s, e in zip(slots, err):
        print(f"  {s}: taught minus grid  dx {e[0]:+5.1f}  dy {e[1]:+5.1f}  dz {e[2]:+5.1f}" + ("   <-- far from the grid" if np.abs(e).max() > 5 else ""))
    if apply:
        grid = {"x": round(x0, 3), "y_B": round(y0, 3), "column_pitch": round(pitch, 3), "z_row1": round(z0, 3), "row_pitch": ROW_PITCH, "yaw": 0.0}
        p = model(fit.x)
        over = h.get("grid", {}).get("overrides", {})          # seats a person saw off the grid: [dx, dy, dz] added to the grid pose
        for s, v in zip(slots, p):
            d = over.get(s, [0, 0, 0])
            h["taught"][s] = [round(float(v[0] + d[0]), 3), round(float(v[1] + d[1]), 3), round(float(v[2] + d[2]), 3), 0.0, -0.0, 0.0]
        grid["overrides"] = over
        h["grid"] = grid
        h["pose_source"] = "grid"
        R.save_layout(layout)
        print("written: fixtures.hotel.taught (15 seats), fixtures.hotel.grid, pose_source grid")


if __name__ == "__main__":
    main("--apply" in sys.argv)
