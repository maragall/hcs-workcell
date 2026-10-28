"""
Do the claw's fingers fit the hotel's slot gaps? An exact check on the manufacturer's CAD (the STEP assembly is not in the repo):
the two fingers and their jaw heads are slid into a slot along its axis, at a lift above the seat, and intersected with the
trays, side panels and base plates of the rack by boolean common. Needs cadquery-ocp and the STEP file.

    python -m bringup.rackcheck PATH/polar.step            the table for a few rows and lifts

Frame: the CAD's (x across the platform, y up, z along its length); the robot's x is the CAD's z + 228.1.
Printed: per row and lift, the shallowest slide position at which fingers and rack share material, or "clear".
"""
from __future__ import annotations

import sys
from typing import Dict, List, Optional

TRAY_TOP_ROW1_Y = 124.0          # CAD heights of the top of the row-1 tray, and the pitch
PITCH = 38.0
PLATE_CENTRE_Z = 253.5           # the plate centre across the rack's depth
COLUMN_X = {"A": 134.0, "B": 0.0, "C": -134.0}
FINGER_JAW_X = 1.2               # where the jaw's centre line is in the CAD's stowed claw
FINGER_TIP_Z = 114.8
HEAD_MID_Y = 484.0
TIP_BEHIND_PLATE_CENTRE = 14.9   # the fingertips grip 14.9 mm short of the plate's centre (wrist to plate 219.9, to fingertip 205)
PLATE_MID = 7.21


def _load(path: str):
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_LabelSequence, TDF_Label
    from OCP.TDataStd import TDataStd_Name
    from OCP.TopLoc import TopLoc_Location
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    doc = TDocStd_Document(TCollection_ExtendedString("d"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.ReadFile(path)
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    found: List[tuple] = []

    def name(label):
        n = TDataStd_Name()
        return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ""

    def walk(label, loc):
        if tool.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            tool.GetComponents_s(label, comps)
            for i in range(1, comps.Length() + 1):
                c = comps.Value(i)
                ref = TDF_Label()
                tool.GetReferredShape_s(c, ref)
                walk(ref, loc.Multiplied(tool.GetLocation_s(c)))
        else:
            shape = tool.GetShape_s(label).Located(loc)
            box = Bnd_Box()
            BRepBndLib.AddOptimal_s(shape, box, False, False)
            if not box.IsVoid():
                found.append((name(label), shape, box.Get()))

    free = TDF_LabelSequence()
    tool.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location())
    return found


def _near(b, x, y, z, tol=1.5):
    return all(abs(a - e) < tol for a, e in zip(b, (x[0], y[0], z[0], x[1], y[1], z[1])))


def parts(path: str) -> Dict[str, list]:
    """fingers, heads, trays (by column and row), panels, bases: the CAD solids, by their boxes."""
    out: Dict[str, list] = {"fingers": [], "heads": [], "trays": [], "panels": [], "bases": []}
    for nm, shape, b in _load(path):
        xmin, ymin, zmin, xmax, ymax, zmax = b
        if abs(ymax - ymin - 43.9) < 1.0 and abs(zmax - zmin - 134.0) < 1.0 and xmax - xmin < 8:
            out["fingers"].append(shape)
        elif abs(ymax - ymin - 10.0) < 0.5 and abs(zmax - zmin - 38.0) < 0.5 and xmax - xmin < 9:
            out["heads"].append(shape)
        elif abs(xmax - xmin - 91.0) < 1.0 and abs(ymax - ymin - 8.0) < 0.6 and abs(zmax - zmin - 138.0) < 1.0:
            out["trays"].append(((xmin + xmax) / 2, ymin, shape))
        elif abs(xmax - xmin - 10.0) < 0.6 and abs(zmax - zmin - 40.0) < 1.0 and ymax - ymin > 300:
            out["panels"].append(shape)
        elif abs(xmax - xmin - 91.0) < 1.0 and abs(ymax - ymin - 10.0) < 0.6 and abs(zmax - zmin - 138.0) < 1.0:
            out["bases"].append(shape)
    return out


def _volume(shape) -> float:
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    return p.Mass()


def _moved(shape, dx: float, dy: float, dz: float):
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    t = gp_Trsf()
    t.SetTranslation(gp_Vec(dx, dy, dz))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def shared_volume(a, b) -> float:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    c = BRepAlgoAPI_Common(a, b)
    return _volume(c.Shape()) if c.IsDone() else 0.0


def slide(p: Dict[str, list], column: str, row: int, lift_mm: float, depth_mm: float) -> float:
    """Cubic mm shared by the fingers and the rack with the fingertips `depth_mm` short of the seated position, the plate raised
    `lift_mm` above its seat. depth 0 is the seat."""
    dx = COLUMN_X[column] - FINGER_JAW_X
    dy = (TRAY_TOP_ROW1_Y + PITCH * (row - 1) + PLATE_MID + lift_mm) - HEAD_MID_Y
    dz = (PLATE_CENTRE_Z - TIP_BEHIND_PLATE_CENTRE - depth_mm) - FINGER_TIP_Z
    claw = [_moved(s, dx, dy, dz) for s in p["fingers"] + p["heads"]]
    rack = [s for cx, ymin, s in p["trays"] if abs(cx - COLUMN_X[column]) < 60 and abs(ymin - (TRAY_TOP_ROW1_Y - 8 + PITCH * (row - 1))) < PITCH * 1.6]
    rack += p["panels"] + p["bases"]
    return sum(shared_volume(c, r) for c in claw for r in rack)


def table(path: str, column: str = "A", rows=(1, 5, 10), lifts=(0.0, 5.0, 10.0, 14.0, 25.0), depths=range(0, 141, 10)) -> List[str]:
    p = parts(path)
    lines = [f"CAD parts found: {len(p['fingers'])} fingers, {len(p['heads'])} heads, {len(p['trays'])} trays, {len(p['panels'])} panels, {len(p['bases'])} bases"]
    for row in rows:
        for lift in lifts:
            hit: Optional[int] = None
            for d in depths:
                if slide(p, column, row, lift, float(d)) > 1.0:
                    hit = d
                    break
            lines.append(f"column {column} row {row:2d} lift {lift:4.1f} mm: " + ("clear all the way in" if hit is None else
                         f"fingers meet the rack when the tips are {hit} mm short of the seat"))
    return lines


if __name__ == "__main__":
    for line in table(sys.argv[1] if len(sys.argv) > 1 else "polar.step"):
        print(line)
