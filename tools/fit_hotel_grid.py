"""
Fit the hand-taught hotel seats (fixtures.hotel.taught_detail, the raw records) to one regular grid and write it as the seat poses:
every seat has yaw 0 and the same x up to a lean along the height (the claw is square to the hotel), columns are `pitch` apart in y (B in the middle), rows are the
STEP's 38 mm tray pitch apart in z. The wrist pose is not the plate: the plate sits `pivot` mm ahead of the wrist along the claw, so a
taught yaw of a few degrees moves the plate sideways by pivot * sin(yaw) (5.5 degrees = 18 mm). Each raw record is therefore turned to
yaw 0 about the PLATE, not about the wrist, and the pivot is fitted with the grid (2026-11-01: 189 mm, the CAD says 185; all three columns
then land within 1 mm). Forcing the yaw to 0 and keeping the wrist y, as the first grid did, bent column B by 18 mm. The residual of
every seat is printed. `--apply` writes fixtures.hotel.taught (the raw records stay in taught_detail), fixtures.hotel.grid
and pose_source "grid".

    python tools/fit_hotel_grid.py [--apply]
"""
import os
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from run import positions  # noqa: E402

COL = {"A": 1, "B": 0, "C": -1}                  # column A is +y, as in the STEP (CAD x +134 = A)
COL_SLOTS = ()
ROW_PITCH = 38.0                                 # the STEP: ten trays 38 mm apart, the hotel uses every second one by the row numbers


def main(apply=False):
    layout = positions.load()
    h = layout["fixtures"]["hotel"]
    raw = {k: v["pose"] for k, v in h["taught_detail"].items() if k[0] in COL and k[1:].isdigit()}
    slots = sorted(raw)
    cols = np.array([COL[s[0]] for s in slots])
    rows = np.array([int(s[1:]) for s in slots])
    yaw = np.radians([h["taught_detail"][s].get("raw_yaw_deg", raw[s][5]) for s in slots])
    xyz = np.array([raw[s][:3] for s in slots])

    def seen_at(pivot):                              # the wrist pose with the claw square, the plate where it was taught
        return np.c_[xyz[:, 0] + pivot * (np.cos(yaw) - 1), xyz[:, 1] + pivot * np.sin(yaw), xyz[:, 2]]

    def model(p):
        x0, y0, pitch, z0, lean, xa, xc = p[0], p[1], p[2], p[3], p[5], p[6], p[7]
        z = z0 + ROW_PITCH * (rows - 1)
        colx = np.where(cols > 0, xa, np.where(cols < 0, xc, 0.0))   # each column's own depth along the slot, B the reference (C read 1 mm deeper, 2026-11-08)
        return np.c_[x0 + colx + lean * (z - z0), y0 + pitch * cols, z]       # lean: the seats' x per mm of height (the rack is not plumb to the
                                                                              # arm's z: 2026-11-08 row 1 read 2 mm deeper than row 9, on repeat)

    fit = least_squares(lambda p: (model(p) - seen_at(p[4])).ravel(), [291.0, 0.0, 134.0, 78.0, 185.0, 0.0, 0.0, 0.0], loss="soft_l1", f_scale=1.0)
    x0, y0, pitch, z0, pivot, lean, xa, xc = fit.x
    err = seen_at(pivot) - model(fit.x)
    print(f"{len(slots)} seats: x {x0:.1f} at B row 1 (A {xa:+.1f}, C {xc:+.1f}), lean {lean * 100:+.2f} mm per 100 mm of height ({np.degrees(np.arctan(lean)):+.2f} deg), "
          f"B at y {y0:.1f}, column pitch {pitch:.1f}, row 1 at z {z0:.1f}, row pitch {ROW_PITCH}, yaw 0, pivot {pivot:.1f}")
    for s, e in zip(slots, err):
        print(f"  {s}: taught minus grid  dx {e[0]:+5.1f}  dy {e[1]:+5.1f}  dz {e[2]:+5.1f}" + ("   <-- far from the grid" if np.abs(e).max() > 5 else ""))
    if apply:
        grid = {"x": round(x0, 3), "x_column_mm": {"A": round(float(xa), 3), "B": 0.0, "C": round(float(xc), 3)}, "x_lean_per_mm": round(float(lean), 5), "y_B": round(y0, 3), "column_pitch": round(pitch, 3), "z_row1": round(z0, 3),
                "row_pitch": ROW_PITCH, "yaw": 0.0, "pivot_mm": round(pivot, 2)}
        p = model(fit.x)
        over = {k: v for k, v in h.get("grid", {}).get("overrides", {}).items() if k in COL_SLOTS}   # [dx, dy, dz] a person saw off the grid
        trim = float(h.get("grid", {}).get("x_trim_mm", 0.0))                  # along the slot, seen by a person: added to every seat's x
        trim_note = h.get("grid", {}).get("_x_trim_note")
        for s, v in zip(slots, p):
            d = over.get(s, [0, 0, 0])
            h["taught"][s] = [round(float(v[0] + d[0] + trim), 3), round(float(v[1] + d[1]), 3), round(float(v[2] + d[2]), 3), 0.0, -0.0, 0.0]
        grid["overrides"] = over
        grid["x_trim_mm"] = trim
        if trim_note:
            grid["_x_trim_note"] = trim_note
        h["grid"] = grid
        h["pose_source"] = "grid"
        positions.save(layout)
        print("written: fixtures.hotel.taught (15 seats), fixtures.hotel.grid, pose_source grid")


if __name__ == "__main__":
    main("--apply" in sys.argv)
