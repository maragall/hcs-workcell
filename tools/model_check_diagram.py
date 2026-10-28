"""docs/model_check.png: the cell from above as hardware.json says it is, for the operator to check against the machine.
Black and white, one scale, robot base frame (x right, y up, arm along +x, hotel slots open toward -x). Where the
measures disagree with each other or with the taught poses the figure shows both, marked with a number.
    python tools/model_check_diagram.py
"""
import json
import math
import os

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HW = json.load(open(os.path.join(HERE, "hardware.json"), encoding="utf-8"))
LAY = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))
R = HW["reading_in_robot_frame"]
J1 = (126.0, 0.0)
PLAT_X, PLAT_Y = R["platform_x"], (-HW["platform"]["y_mm"] / 2, HW["platform"]["y_mm"] / 2)     # y placement not measured: centred on the arm axis
BASE_X, BASE_Y = R["base_box_x"], (-HW["arm_base"]["y_mm"] / 2, HW["arm_base"]["y_mm"] / 2)
SLOT_X, END_X = R["slot_line_x"], R["hotel_end_x"]
HOTEL_Y = (-HW["hotel"]["y_mm"] / 2, HW["hotel"]["y_mm"] / 2)
DEPTH = HW["hotel"]["x_mm"]
COLS = HW["hotel"]["columns"]
MODULE = tuple(LAY["fixtures"]["polar"]["pose_at_0"]["pose"][:2])                    # taught seat at 0 degrees
SEAT = tuple(LAY["fixtures"]["microscope"]["pose"][:2])
DY = HW["rotation_module"]["to_column_a_mm"]["dy"]

S, SS = 1.35, 2
X0, X1, Y0, Y1 = -130, 610, -250, 470
TOP, FOOT = 80, 250
W, H = int((X1 - X0) * S) + 40, int((Y1 - Y0) * S) + TOP + FOOT
FONT, BOLD = "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/segoeuib.ttf"
BLACK, GREY, LIGHT = (0, 0, 0), (120, 120, 120), (232, 232, 232)
img = Image.new("RGB", (W * SS, H * SS), "white")
d = ImageDraw.Draw(img)


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size * SS)


def P(p):
    return ((20 + (p[0] - X0) * S) * SS, (TOP + (Y1 - p[1]) * S) * SS)


def label(p, s, size=14, bold=False, anchor="mm", dx=0, dy=0):
    x, y = P(p)[0] + dx * SS, P(p)[1] + dy * SS
    b = d.textbbox((x, y), s, font=font(size, bold), anchor=anchor)
    pad = 3 * SS
    d.rectangle([b[0] - pad, b[1] - pad, b[2] + pad, b[3] + pad], fill="white")
    d.text((x, y), s, fill=BLACK, font=font(size, bold), anchor=anchor)


def line(a, b, w=2, color=BLACK):
    d.line([P(a), P(b)], fill=color, width=w * SS)


def dashed(a, b, w=2, color=GREY, dash=9, gap=6):
    (x0, y0), (x1, y1) = P(a), P(b)
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / ((dash + gap) * SS)))
    for i in range(n):
        t0, t1 = i / n, (i + dash / (dash + gap)) / n
        d.line([(x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0), (x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1)], fill=color, width=w * SS)


def rect(x, y, fill=None, dash=False, w=2, color=BLACK):
    pts = [(x[0], y[0]), (x[1], y[0]), (x[1], y[1]), (x[0], y[1]), (x[0], y[0])]
    if fill:
        d.polygon([P(p) for p in pts], fill=fill)
    for a, b in zip(pts, pts[1:]):
        dashed(a, b, w, color) if dash else line(a, b, w, color)


def disc(c, r, fill="white", w=3):
    (cx, cy), rr = P(c), r * S * SS
    d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=fill, outline=BLACK, width=w * SS)


def arrow(p, q, size=8):
    (x0, y0), (x1, y1) = P(p), P(q)
    a = math.atan2(y1 - y0, x1 - x0)
    s = size * SS
    d.polygon([(x1, y1), (x1 - s * math.cos(a - 0.35), y1 - s * math.sin(a - 0.35)),
               (x1 - s * math.cos(a + 0.35), y1 - s * math.sin(a + 0.35))], fill=BLACK)


def dim(a, b, text, size=13, dy=0, dx=0):
    line(a, b, 1)
    arrow(a, b)
    arrow(b, a)
    label(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), text, size, dx=dx, dy=dy)


def tick(p, up=True):
    dashed(p, (p[0], p[1] + (14 if up else -14)), 1, GREY, 3, 3)


d.text((20 * SS, 12 * SS), "The cell as hardware.json describes it", fill=BLACK, font=font(26, True))
d.text((20 * SS, 46 * SS), "Top view, robot base frame, mm, one scale. Numbers in circles are disagreements, listed below.", fill=GREY, font=font(14))

# platform, base box
rect(PLAT_X, PLAT_Y, dash=True, color=GREY)
label((PLAT_X[1], PLAT_Y[1]), "platform 650 x 400 (y position: not measured)", 13, anchor="ra", dx=-6, dy=14)
rect(BASE_X, BASE_Y, fill=LIGHT)
label(((BASE_X[0] + BASE_X[1]) / 2, 0), "base\n150 x 140", 13)
# arm: j1, j2 (links 200)
J2 = (J1[0] + 200, 0)
line(J1, J2, 7)
line(J1, J2, 3, "white")
disc(J1, 12, LIGHT)
disc(J2, 10)
label((J1[0], 0), "j1", 12, True, dy=-24)
label((J2[0], 0), "j2", 12, True, dy=-24)
label(((J1[0] + J2[0]) / 2, 0), "link 200", 12, dy=18)

# hotel: slot line, columns, the two readings of its depth
rect((SLOT_X, END_X), HOTEL_Y, fill=LIGHT)
dashed((END_X - DEPTH, HOTEL_Y[0]), (END_X - DEPTH, HOTEL_Y[1]), 2, BLACK)
line((SLOT_X, HOTEL_Y[0]), (SLOT_X, HOTEL_Y[1]), 4)
for name, y in COLS.items():
    disc((SLOT_X, y), 9)
    d.text(P((SLOT_X, y)), name, fill=BLACK, font=font(13, True), anchor="mm")
label((END_X - 70, -20), "plate hotel\n376 x 139.29", 14, True)
label((SLOT_X - 4, HOTEL_Y[1] + 4), "slot line", 12, anchor="rb", dy=-4)

# module (taught seat) and column A offset
disc(MODULE, 7, "black")
label(MODULE, "rotation module seat", 13, True, anchor="lm", dx=14, dy=-14)
label(MODULE, f"taught ({MODULE[0]:.0f}, {MODULE[1]:.0f})", 12, anchor="lm", dx=14, dy=6)
dim((SLOT_X + 34, COLS["A"]), (SLOT_X + 34, COLS["A"] + DY), f"{DY}", dx=0)
dashed((SLOT_X, COLS["A"]), (SLOT_X + 40, COLS["A"]), 1, GREY, 3, 3)
dashed((MODULE[0], MODULE[1]), (SLOT_X + 40, MODULE[1]), 1, GREY, 3, 3)

# microscope seat
disc(SEAT, 20)
line((SEAT[0] - 6, SEAT[1]), (SEAT[0] + 6, SEAT[1]), 2)
line((SEAT[0], SEAT[1] - 6), (SEAT[0], SEAT[1] + 6), 2)
label(SEAT, f"microscope seat\ntaught ({SEAT[0]:.0f}, {SEAT[1]:.0f})", 13, True, anchor="lm", dx=34)

# measures along the bottom
yb = PLAT_Y[0] - 22
for a, b, t, off in ((BASE_X[1], SLOT_X, "250 base to slot", 0), (BASE_X[1], END_X, "490 base to hotel end", 26), (PLAT_X[0], PLAT_X[1], "650 platform", 52)):
    y = yb - off
    ax = PLAT_X[0] if t.endswith("platform") else a
    dashed((ax, PLAT_Y[0]), (ax, y), 1, GREY, 3, 3)
    dashed((b, PLAT_Y[0] if off == 0 else y + 26 if False else PLAT_Y[0]), (b, y), 1, GREY, 3, 3)
    dim((ax, y), (b, y), t, 13)
dim((BASE_X[1], -95), (J1[0], -95), "60 base to j1", 12, dy=14)
dashed((BASE_X[1], BASE_Y[0]), (BASE_X[1], -95), 1, GREY, 3, 3)

# axes, scale bar
ox, oy = 40, H - 36
for a, b in (((ox, oy), (ox + 60, oy)), ((ox, oy), (ox, oy - 60))):
    d.line([(a[0] * SS, a[1] * SS), (b[0] * SS, b[1] * SS)], fill=BLACK, width=3 * SS)
d.text(((ox + 68) * SS, oy * SS), "x", fill=BLACK, font=font(15, True), anchor="lm")
d.text((ox * SS, (oy - 68) * SS), "y", fill=BLACK, font=font(15, True), anchor="mb")
bx = ox + 150
d.line([(bx * SS, oy * SS), ((bx + 100 * S) * SS, oy * SS)], fill=BLACK, width=3 * SS)
d.text(((bx + 50 * S) * SS, (oy - 8) * SS), "100 mm", fill=BLACK, font=font(13), anchor="mb")

# disagreements
notes = ["1  Hotel depth: the dashed line is 139.29 mm from the hotel end (operator); the slot line, 240 mm from it, would give a deeper hotel.",
         "2  The module seat (taught, y 305) lies outside a 400 mm platform centred on the arm axis; the arm axis position is not measured.",
         "3  Slot line x 316 (operator, 250 mm from the base edge) against 293 from the taught hotel poses."]
for i, n in enumerate(notes):
    d.text((20 * SS, (H - FOOT + 74 + i * 20) * SS), n, fill=BLACK, font=font(13))
for num, p, dxy in ((1, (END_X - DEPTH, HOTEL_Y[1]), (0, -14)), (2, (PLAT_X[1], PLAT_Y[1]), (-16, 28)), (3, (SLOT_X, HOTEL_Y[0]), (14, 14))):
    c = P(p)
    cx, cy = c[0] + dxy[0] * SS, c[1] + dxy[1] * SS
    d.ellipse([cx - 11 * SS, cy - 11 * SS, cx + 11 * SS, cy + 11 * SS], fill="white", outline=BLACK, width=2 * SS)
    d.text((cx, cy), str(num), fill=BLACK, font=font(13, True), anchor="mm")

out = os.path.join(HERE, "docs", "model_check.png")
img.resize((W, H), Image.LANCZOS).save(out)
print(out, W, H)
