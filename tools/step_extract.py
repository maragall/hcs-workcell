"""
Walk the manufacturer's STEP assembly and write every part: its path in the tree, name, instance count, bounding box in the assembly frame
(CAD units, mm) and size. The text of the STEP says little; the assembly tree with each instance's placement is the information.

    python tools/step_extract.py "C:/Users/ASUS/Downloads/Polar-R400-Z400 模型图.STEP" [out.json]
"""
import json
import os
import sys
import time

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def name_of(label) -> str:
    n = TDataStd_Name()
    return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ""


def main(path, out):
    t0 = time.time()
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.InitDocument(doc)
    rd = STEPCAFControl_Reader()
    rd.SetNameMode(True)
    rd.SetColorMode(False)
    if rd.ReadFile(path) != IFSelect_RetDone:
        sys.exit("could not read the STEP file")
    print(f"read the file ({time.time() - t0:.0f} s); transferring", flush=True)
    rd.Transfer(doc)
    print(f"transferred ({time.time() - t0:.0f} s)", flush=True)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    rows = []

    def walk(label, loc, trail):
        ref = TDF_Label()
        if st.IsReference_s(label):
            st.GetReferredShape_s(label, ref)
        else:
            ref = label
        here = trail + [name_of(label) or name_of(ref)]
        if st.IsAssembly_s(ref):
            comps = TDF_LabelSequence()
            st.GetComponents_s(ref, comps)
            for i in range(1, comps.Length() + 1):
                c = comps.Value(i)
                walk(c, loc.Multiplied(st.GetLocation_s(c)), here)
        else:
            shape = st.GetShape_s(ref).Located(loc)
            box = Bnd_Box()
            BRepBndLib.AddOptimal_s(shape, box, False, False)
            if box.IsVoid():
                return
            x0, y0, z0, x1, y1, z1 = box.Get()
            rows.append({"path": " / ".join(here), "name": here[-1], "min": [x0, y0, z0], "max": [x1, y1, z1],
                         "size": [x1 - x0, y1 - y0, z1 - z0]})

    free = TDF_LabelSequence()
    st.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location(), [])
        if len(rows) % 200 == 0:
            print(f"{len(rows)} parts ({time.time() - t0:.0f} s)", flush=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)
    print(f"{len(rows)} parts written to {out} ({time.time() - t0:.0f} s)", flush=True)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "docs", "cad_parts.json"))
