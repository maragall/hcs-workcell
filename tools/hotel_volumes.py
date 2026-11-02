"""Writes layout.volumes.obstacles.hotel.parts: the hotel as the STEP has it (docs/cad_parts.json), not one box. Every tray is a box
(x 412.6..550.6, 91 wide, 8 thick) with the operator's 3 mm lip on top, the six rear posts, the roof and the base are their own boxes.
The envelope (x, y, z) stays, as the quick test before the parts are looked at.

    python tools/hotel_volumes.py
"""
import json, os, sys
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402

K = -57.4                 # volumes.z_offset_k: robot z = CAD y + k
LIP_MM = 3.0              # the slot lip, operator 2026-11-01
parts = json.load(open(os.path.join(HERE, "docs", "cad_parts.json"), encoding="utf-8"))
out, trays = [], 0
for p in parts:
    if p["size"][0] > 1e50:
        continue
    (cx0, cy0, cz0), (cx1, cy1, cz1) = p["min"], p["max"]
    b = {"x": [round(cz0 + 228.1, 1), round(cz1 + 228.1, 1)], "y": [round(cx0, 1), round(cx1, 1)], "z": [round(cy0 + K, 1), round(cy1 + K, 1)]}
    if not (b["x"][0] >= 405 and b["x"][1] <= 560 and b["y"][0] >= -200 and b["y"][1] <= 200 and b["z"][0] > 40):
        continue
    if abs(b["z"][1] - b["z"][0] - 8) < 0.6 and b["x"][1] - b["x"][0] > 100 and abs(b["y"][1] - b["y"][0] - 91) < 1:
        b["z"][1] = round(b["z"][1] + LIP_MM, 1)          # the lip stands 3 mm above the tray
        b["kind"], b["margin_mm"] = "tray", 1.0
        trays += 1
    elif b["z"][1] - b["z"][0] > 300:
        b["kind"], b["margin_mm"] = "post", 0.5
    else:
        b["kind"], b["margin_mm"] = "part", 1.0
    out.append(b)
L = R.load_layout()
rows = L["fixtures"]["hotel"]["rows"]                     # the trays that are there: the plate slid in 25 mm above a seat for a day, so the
keep = []                                                  # trays between the rows in use are absent (a 30 mm gap would have been hit)
for b in out:
    if b["kind"] == "tray":
        idx = round((b["z"][0] - 58.1) / 38.0) + 1
        if idx not in rows:
            continue
    keep.append(b)
out = keep
trays = sum(b["kind"] == "tray" for b in out)
h = L["volumes"]["obstacles"]["hotel"]
h["parts"] = out
h.pop("trays_z", None), h.pop("_trays_note", None)
h["_parts_note"] = ("2026-11-01: the hotel from the STEP (tools/hotel_volumes.py): each tray with a 3 mm lip, the rear posts, roof and base. "
                    "Inside the envelope only the parts count; the plate may be in the gap between two trays and nowhere else.")
L["volumes"]["held"]["hotel_front_allowance_mm"] = 0.0
R.save_layout(L)
print(len(out), "parts,", trays, "trays")
