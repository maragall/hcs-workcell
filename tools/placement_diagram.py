"""docs/placement.png: the cell from above, from the manufacturer CAD (layout.json `volumes`, hardware.json `cad`).
Black and white, one scale, robot base frame in mm (x right, y up). The claw is drawn where it really is: its
fingers are 205 mm ahead of the wrist pose the controller reports.
    python tools/placement_diagram.py
"""
import json
import math
import os

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAY = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))
CAD = json.load(open(os.path.join(HERE, "hardware.json"), encoding="utf-8"))["cad"]
CLAW = LAY["volumes"]["claw"]
PILLAR = LAY["volumes"]["obstacles"]["pillar"]
HOTEL_X, HOTEL_Y = CAD["hotel"]["x_r"], CAD["hotel"]["side_panels_y_r"]
COLS, COL_W = CAD["hotel"]["columns_y_r"], CAD["hotel"]["column_width_mm"]
PLAT_X, PLAT_Y = CAD["platform"]["x_r"], CAD["platform"]["y_r"]
MOD_AXIS, MOD_X, MOD_Y = CAD["rotation_module"]["axis_r"], CAD["rotation_module"]["plate_seat_x_r"], CAD["rotation_module"]["plate_seat_y_r"]
J1 = (126.0, 0.0)
SEAT = tuple(json.load(open(os.path.join(HERE, "hardware.json"), encoding="utf-8"))["microscope"]["placement"]["seat_xy_robot_mm"])
MODULE_WRIST = LAY["fixtures"]["polar"]["pose_at_0"]["pose"]
CRASH = (199.3, 121.0, 9.0)                       # wrist x, y, yaw where the arm stopped on 2026-10-27

S, SS = 1.25, 2
X0, X1, Y0, Y1 = -130, 600, -230, 480
TOP, FOOT = 86, 60
W, H = int((X1 - X0) * S) + 40, int((Y1 - Y0) * S) + TOP + FOOT
FONT, BOLD = "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/segoeuib.ttf"
BLACK, GREY, LIGHT = (0, 0, 0), (120, 120, 120), (232, 232, 232)
img = Image.new("RGB", (W * SS, H * SS), "white")
d = ImageDraw.Draw(img)


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size * SS)


def P(p):
    return ((20 + (p[0] - X0) * S) * SS, (TOP + (Y1 - p[1]) * S) * SS)


def label(p, s, size=13, bold=False, anchor="mm", dx=0, dy=0):
    x, y = P(p)[0] + dx * SS, P(p)[1] + dy * SS
    b = d.textbbox((x, y), s, font=font(size, bold), anchor=anchor)
    pad = 3 * SS
    d.rectangle([b[0] - pad, b[1] - pad, b[2] + pad, b[3] + pad], fill="white")
    d.text((x, y), s, fill=BLACK, font=font(size, bold), anchor=anchor)


def poly(pts, fill=None, w=2):
    xy = [P(p) for p in pts]
    if fill:
        d.polygon(xy, fill=fill)
    d.line(xy + [xy[0]], fill=BLACK, width=w * SS)


def rect(x, y, **kw):
    poly([(x[0], y[0]), (x[1], y[0]), (x[1], y[1]), (x[0], y[1])], **kw)


def claw(wx, wy, yaw, fill=None):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    for part in ("body", "fingers"):
        (x0, x1), (y0, y1) = CLAW[part]["x"], CLAW[part]["y"]
        poly([(wx + u * c - v * s, wy + u * s + v * c) for u, v in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))], fill=fill)


def disc(c, r, fill="white", w=3):
    (cx, cy), rr = P(c), r * S * SS
    d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=fill, outline=BLACK, width=w * SS)


def arrow(p, q, size=8):
    (x0, y0), (x1, y1) = P(p), P(q)
    a = math.atan2(y1 - y0, x1 - x0)
    k = size * SS
    d.polygon([(x1, y1), (x1 - k * math.cos(a - 0.35), y1 - k * math.sin(a - 0.35)),
               (x1 - k * math.cos(a + 0.35), y1 - k * math.sin(a + 0.35))], fill=BLACK)


def dim(a, b, text, dy=0):
    d.line([P(a), P(b)], fill=BLACK, width=SS)
    arrow(a, b)
    arrow(b, a)
    label(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), text, 12, dy=dy)


d.text((20 * SS, 12 * SS), "The cell from above", fill=BLACK, font=font(28, True))
d.text((20 * SS, 50 * SS), "From the manufacturer CAD. Robot base frame, mm, one scale. The claw is drawn where it really is: "
       "its fingertips are 205 mm ahead of the wrist pose.", fill=GREY, font=font(14))

rect(PLAT_X, PLAT_Y, fill=(246, 246, 246))
label((PLAT_X[1], PLAT_Y[0]), f"platform {PLAT_X[1] - PLAT_X[0]:.0f} x {PLAT_Y[1] - PLAT_Y[0]:.0f}", 13, anchor="ra", dx=-6, dy=-12)
rect(PILLAR["x"], PILLAR["y"], fill=LIGHT)
label(((PILLAR["x"][0] + PILLAR["x"][1]) / 2, 0), "arm\npillar", 13)
disc(J1, 12, "white")
label(J1, "j1", 12, True, dy=-22)

rect(HOTEL_X, HOTEL_Y, fill=LIGHT)
for name, y in COLS.items():
    rect(HOTEL_X, (y - COL_W / 2, y + COL_W / 2), fill="white", w=1)
    label((HOTEL_X[0] + 24, y), name, 14, True)
label((HOTEL_X[1] - 46, -150), f"plate hotel\n{CAD['hotel']['depth_mm']:.0f} x {HOTEL_Y[1] * 2:.0f}", 13, True)

claw(MODULE_WRIST[0], MODULE_WRIST[1], MODULE_WRIST[5])
disc(MODULE_WRIST[:2], 5, "black")
label(MODULE_WRIST[:2], "wrist at the module seat (taught 313, 305);\nthe claw points -y over the module", 12, anchor="lm", dx=12, dy=-26)
claw(CRASH[0], CRASH[1], CRASH[2])
disc(CRASH[:2], 5, "black")
label(CRASH[:2], "wrist where the arm stopped (199, 121);\nthe claw points +x, fingertips 10 mm from the hotel", 12, anchor="rm", dx=-14, dy=-44)

rect(MOD_X, MOD_Y, fill=(205, 205, 205), w=4)             # the module is on the platform: drawn last, heavy, labelled
disc(MOD_AXIS, 7, "black")
label((MOD_AXIS[0], MOD_Y[0]), f"ROTATION MODULE\naxis ({MOD_AXIS[0]:.0f}, {MOD_AXIS[1]:.0f}), on the platform", 13, True, anchor="ma", dy=8)

disc(SEAT, 20)
label(SEAT, f"microscope: holder centre ({SEAT[0]:.0f}, {SEAT[1]:.0f}),\n14 cm beyond the long edge, 22 cm from the left end", 13, True, anchor="lm", dx=30)

dim((PILLAR["x"][1], -100), (J1[0], -100), "66  pillar edge to j1", dy=-14)
dim((PILLAR["x"][1], -150), (HOTEL_X[1], -150), "490  pillar edge to hotel end", dy=-14)
dim((HOTEL_X[0], 210), (HOTEL_X[1], 210), "138", dy=-12)

ox, oy = 40, H - 26
for a, b in (((ox, oy), (ox + 50, oy)), ((ox, oy), (ox, oy - 50))):
    d.line([(a[0] * SS, a[1] * SS), (b[0] * SS, b[1] * SS)], fill=BLACK, width=3 * SS)
d.text(((ox + 58) * SS, oy * SS), "x", fill=BLACK, font=font(15, True), anchor="lm")
d.text((ox * SS, (oy - 58) * SS), "y", fill=BLACK, font=font(15, True), anchor="mb")
bx = ox + 140
d.line([(bx * SS, oy * SS), ((bx + 100 * S) * SS, oy * SS)], fill=BLACK, width=3 * SS)
d.text(((bx + 50 * S) * SS, (oy - 8) * SS), "100 mm", fill=BLACK, font=font(13), anchor="mb")

out = os.path.join(HERE, "docs", "placement.png")
tmp = out + ".tmp.png"
img.resize((W, H), Image.LANCZOS).save(tmp)
os.replace(tmp, out)
print(out, W, H)
