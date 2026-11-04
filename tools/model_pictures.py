"""Pictures of the cell model for the manual (docs/manual/model_*.png): the window's own Model (gui.py) drawn with PIL instead of a tk
canvas, at poses the setup goes through. Nothing touches the arm. `python -X utf8 tools/model_pictures.py`."""
from __future__ import annotations

import json
import math
import os
import sys
import time

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import gui  # noqa: E402

OUT = os.path.join(HERE, "docs", "manual")


class Recorder:
    """What gui.Model asks of a tk canvas, drawn into a PIL image (items in the order they are created, as tk does)."""

    def __init__(self, parent=None, width=560, height=480, bg="#000000", **_):
        self.w, self.h, self.bg = width, height, bg
        self.img = Image.new("RGB", (width, height), bg)
        self.d = ImageDraw.Draw(self.img)
        try:
            self.font = ImageFont.truetype("consola.ttf", 13)
            self.font_ui = ImageFont.truetype("segoeui.ttf", 13)
        except OSError:
            self.font = self.font_ui = ImageFont.load_default()

    def pack(self, *a, **k):
        pass

    def delete(self, tag):
        pass

    def find_withtag(self, tag):
        return []

    def create_line(self, x0, y0, x1, y1, fill="#fff", width=1, tags=None, **_):
        self.d.line([(x0, y0), (x1, y1)], fill=fill, width=int(width))

    def create_polygon(self, *coords, fill="", outline="", tags=None, **_):
        pts = list(zip(coords[0::2], coords[1::2]))
        self.d.polygon(pts, fill=fill or None, outline=outline or None)

    def create_oval(self, x0, y0, x1, y1, outline="#fff", fill="", tags=None, **_):
        self.d.ellipse([x0, y0, x1, y1], fill=fill or None, outline=outline or None)

    def create_text(self, x, y, text="", fill="#fff", anchor="nw", font=None, tags=None, **_):
        f = self.font_ui if font and font[0] == "Segoe UI" else self.font
        box = self.d.textbbox((0, 0), text, font=f)
        tw, th = box[2] - box[0], box[3] - box[1]
        ax = {"n": -tw / 2, "s": -tw / 2, "e": -tw, "w": 0, "c": -tw / 2}
        px = x + (ax["e"] if "e" in anchor else ax["w"] if "w" in anchor else -tw / 2)
        py = y + (-th if "s" in anchor else 0 if "n" in anchor else -th / 2)
        self.d.text((px, py), text, fill=fill, font=f)


class Poll:
    def __init__(self, pose, jaw, module_raw, joints):
        self.pose, self.jaw, self.module, self.joints, self.at, self.error = pose, jaw, module_raw, joints, time.time(), ""


def picture(name, lay, pose, jaw, module_raw, joints, size=(900, 720), zoom=None, route=None, caption=None, view=None):
    gui.tk.Canvas = Recorder                                                  # the Model draws into PIL
    if view:                                                                  # (azimuth, elevation) in degrees; (0, 90) is the plan view
        gui.Model.AZ, gui.Model.EL = math.radians(view[0]), math.radians(view[1])
    m = gui.Model(None, lay, size[0], size[1])
    if zoom:                                                                  # (centre x, y, z, mm across the picture)
        cx, cy, cz, span = zoom
        u, v, _ = m._rot(cx, cy, cz)
        m.scale = size[0] / span
        m.ox, m.oy = size[0] / 2 - u * m.scale, size[1] / 2 + v * m.scale
        m.canvas = Recorder(None, size[0], size[1])
        m.draw_static()
    if route:
        m.set_route(route)
    m.draw(Poll(pose, jaw, module_raw, joints))
    if caption:
        m.canvas.d.text((10, size[1] - 40), caption, fill="#e6e8eb", font=m.canvas.font_ui)
    path = os.path.join(OUT, f"model_{name}.png")
    m.canvas.img.save(path)
    print("wrote", path)


def main():
    os.makedirs(OUT, exist_ok=True)
    lay = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))
    F = lay["fixtures"]
    from routes_opt import fk
    seat = F["microscope"]["pose"]
    nest = lay["routes"]["nest"]["pose"]
    P0 = F["polar"]["pose_at_0"]["pose"]
    zero = lay["polar"]["ext_axis"]["zero_reading"]
    j_seat = fk.ik(seat, elbow=1)
    j_nest = fk.ik(nest, elbow=1)
    j_p0 = fk.ik(P0, elbow=1)
    band = lay["gripper"]["accept_band"]
    short = sum(band["short_edges"]) / 2
    # 1. the whole cell, the arm at its nest, jaw open
    picture("cell", lay, [nest[0], nest[1], nest[2], nest[5]], 29.0, zero, j_nest,
            caption="the cell as the window draws it: hotel (blue trays), pillar, module (green), microscope box (red), the arm")
    # 2. the microscope seat: the fingers around the plate in the holder (what you teach by hand)
    picture("seat", lay, [seat[0], seat[1], seat[2], seat[5]], short, zero, j_seat,
            zoom=(seat[0] + 186.5 * math.cos(math.radians(seat[5])), seat[1] + 186.5 * math.sin(math.radians(seat[5])), seat[2], 700),
            caption="the microscope seat: the fingers around the plate's short edges, the claw square to the instrument")
    # 2b. the same from above: where the plate and the fingers are, in plan
    picture("seat_top", lay, [seat[0], seat[1], seat[2], seat[5]], short, zero, j_seat, view=(0.0, 89.9),
            zoom=(seat[0] + 186.5 * math.cos(math.radians(seat[5])), seat[1] + 186.5 * math.sin(math.radians(seat[5])), seat[2], 700),
            caption="the seat from above: the plate in the holder, the fingers on its short edges, the arm behind")
    picture("cell_top", lay, [nest[0], nest[1], nest[2], nest[5]], 29.0, zero, j_nest, view=(0.0, 89.9),
            caption="the cell from above: arm base at the dot, hotel, module, microscope box")
    gui.Model.AZ, gui.Model.EL = math.radians(-38.0), math.radians(27.0)
    # 3. at the module, the plate gripped by the short edges, module at 0
    picture("module", lay, [P0[0], P0[1], P0[2] + 34.4, P0[5]], short, zero, fk.ik([P0[0], P0[1], P0[2] + 34.4, 0, 0, P0[5]], elbow=1),
            zoom=(312.6, 118.0, P0[2], 700), caption="above the rotation module (0 deg): the plate held by its short edges")
    # 4. a planned route drawn, nest to the module hover
    hover = [P0[0], P0[1], P0[2] + 34.4]
    route = {"to": "polar hover", "name": "lift-travel-lower", "taught": False, "planned_s": 0.0,
             "poses": [[nest[0], nest[1], hover[2] + 150, 0, 0, nest[5]], [hover[0], hover[1], hover[2] + 150, 0, 0, P0[5]], hover + [0, 0, P0[5]]],
             "joint_legs": [1]}
    picture("route", lay, [nest[0], nest[1], nest[2], nest[5]], 8.2, zero, j_nest, route=route,
            caption="a route as the window shows it while the arm drives it: blue = straight legs, orange = joint leg")


if __name__ == "__main__":
    main()
