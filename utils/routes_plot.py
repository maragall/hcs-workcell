"""docs/routes_3d.html: every stored route (run/preset/routes.json) drawn in the hotel model, for a look by eye. Offline, no hardware.
The wrist path as stored, with joint legs traced through the arc the arm really drives, and the plate's centre for routes that carry one.
    python utils/routes_plot.py"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
os.chdir(HERE)
from run.planner import joints as jt  # noqa: E402
from run.planner import fk  # noqa: E402
from utils.hotel_model_html import scene_boxes  # noqa: E402

L = json.load(open("layout.json", encoding="utf-8"))
R = json.load(open("run/preset/routes.json", encoding="utf-8"))
LIM = jt.limits(L)
HELD = L["volumes"]["held"]


def trace(key, v):
    name, holding = key.split("|")
    a, b = name.split("->")
    fwd = a == "module"
    other = b if fwd else a
    hotel = len(other) in (2, 3) and other[0] in "ABC" and other[1:].isdigit()
    if fwd:
        here = list(v.get("start") or R["start"])
        cj = list(v["start_joints"]) if v.get("start_joints") else fk.ik(here, elbow=v.get("branch", 1))
    else:
        here = list(v["entry"])
        cj = list(v["start_joints"])
    pts = []
    for i, p in enumerate(v["poses"]):
        target = v["joint_targets"].get(str(i))
        if target is not None:                                    # a joint leg: the arc the arm really drives
            n = max(1, int(math.ceil(max(abs(target[k] - cj[k]) for k in range(4)) / 2.0)))
            for s in range(n + 1):
                j = [cj[k] + s / n * (target[k] - cj[k]) for k in range(4)]
                q = fk.fk(j)
                pts.append([q[0], q[1], q[2], j[1] + j[2] + j[3], 1])
            cj = list(target)
        else:
            branch = fk.elbow_sign(cj)
            ja, jb, turn = jt.leg_joints(here, p, branch, LIM, cj)
            n = max(1, int(math.dist(here[:3], p[:3]) / 10.0))
            for s in range(n + 1):
                t = s / n
                pts.append([here[k] + t * (p[k] - here[k]) for k in range(3)] + [here[5] + t * turn, 0])
            cj = jb
        here = list(p)
    grip = holding if holding in HELD and isinstance(HELD[holding], dict) else "long_edges"
    return {"key": key, "group": other[0] if hotel else "m", "fwd": fwd, "holding": holding, "legs": len(v["poses"]),
            "ahead": sum(HELD[grip]["x"]) / 2, "pts": [[round(x, 1) for x in q] for q in pts]}


routes, skipped = [], []
for k, v in R["routes"].items():
    if not (isinstance(v, dict) and "poses" in v and "->" in k):
        continue
    try:
        routes.append(trace(k, v))
    except Exception as e:
        skipped.append(f"{k}: {type(e).__name__}: {e}")
obst = L["volumes"]["obstacles"]
boxes = [b for b in scene_boxes(L) if b["k"] in ("tray", "post", "hotel")]
boxes += [{"k": "pillar", "x": obst["pillar"]["x"], "y": obst["pillar"]["y"], "z": obst["pillar"]["z"]},
          {"k": "platform", "x": [-95, 554], "y": [-200, 200], "z": [-47, 48]}]
if "microscope" in obst:
    boxes.append({"k": "microscope", "x": obst["microscope"]["x"], "y": obst["microscope"]["y"], "z": obst["microscope"]["z"]})
page = open(os.path.join(HERE, "utils", "routes_plot_template.html"), encoding="utf-8").read()
page = page.replace("__DATA__", json.dumps({"routes": routes, "boxes": boxes, "seat": L["fixtures"]["polar"]["pose_at_90"]["pose"],
                                            "planned_at": R.get("planned_at")}))
out = os.path.join(HERE, "docs", "routes_3d.html")
open(out, "w", encoding="utf-8").write(page)
print(out, len(routes), "routes drawn")
for s in skipped:
    print("skipped", s)
