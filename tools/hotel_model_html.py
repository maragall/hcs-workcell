"""Writes docs/hotel_model.html: the STEP's parts (docs/cad_parts.json, tools/step_extract.py) as boxes in the robot frame, with the
claw and the plate in it, the hotel's slot length along x and a pose slider. Robot frame: x = CAD z + 228.1, y = CAD x, z = CAD y + k."""
import json, os
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
K = -57.4
parts = json.load(open(os.path.join(HERE, "docs", "cad_parts.json"), encoding="utf-8"))
lay = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))
boxes = []
for p in parts:
    if p["size"][0] > 1e50:
        continue
    (x0, y0, z0), (x1, y1, z1) = p["min"], p["max"]
    b = dict(n=p["name"], x=[round(z0 + 228.1, 1), round(z1 + 228.1, 1)], y=[round(x0, 1), round(x1, 1)], z=[round(y0 + K, 1), round(y1 + K, 1)])
    hx = b["x"][0] >= 405 and b["x"][1] <= 560 and b["y"][0] >= -200 and b["y"][1] <= 200 and b["z"][0] > 40
    kind = "tray" if hx and abs((b["z"][1] - b["z"][0]) - 8) < 0.6 and (b["x"][1] - b["x"][0]) > 100 else "post" if hx and (b["z"][1] - b["z"][0]) > 300 else "hotel" if hx else "other"
    b["k"] = kind
    boxes.append(b)
v = lay["volumes"]
seats = {s: lay["fixtures"]["hotel"]["taught"][s][:3] for s in lay["fixtures"]["hotel"]["taught"] if s[1:].isdigit()}
data = dict(boxes=boxes, claw=v["claw"], plate=v["held"]["long_edges"], seats=seats, front_allow=v["held"]["hotel_front_allowance_mm"],
            margin=v["margin_mm"], lift=lay["fixtures"]["hotel"]["lift_mm"], approach=lay["fixtures"]["hotel"]["approach"]["mm"],
            axis=[126.0, 0.0], plate_dims=[127.46, 85.14, 14.42])
html = open(os.path.join(HERE, "tools", "hotel_model_template.html"), encoding="utf-8").read().replace("__DATA__", json.dumps(data))
open(os.path.join(HERE, "docs", "hotel_model.html"), "w", encoding="utf-8").write(html)
trays = [b for b in boxes if b["k"] == "tray"]
print(len(boxes), "parts;", len(trays), "trays; tray x", sorted({tuple(b["x"]) for b in trays})[:4], "posts", sum(b["k"] == "post" for b in boxes))
