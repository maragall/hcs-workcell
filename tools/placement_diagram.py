"""docs/placement.png: where the microscope goes, top view in the robot base frame (mm). Numbers from layout.json's
regions and fixtures, written out here because the figure is for people who do not have the layout file.
    python tools/placement_diagram.py
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

BASE, SEAT, MODULE = (126, 0), (123, 400), (317, 305)
HOTEL_X, HOTEL_Y = 293, (-137.5, 137.5)
REACH, INNER, TOL = 420, 240, 20
S, SS = 1.0, 2                              # px per mm, supersampling
X0, X1, Y0, Y1 = -330, 600, -445, 445       # world window, mm
HEAD = 190
W, H = int((X1 - X0) * S) + 40, int((Y1 - Y0) * S) + HEAD + 40
INK, GREY, ORANGE, BLUE, GREEN = (24, 24, 24), (120, 130, 130), (196, 82, 24), (30, 85, 165), (40, 125, 45)
FONT = "C:/Windows/Fonts/segoeui.ttf"
BOLD = "C:/Windows/Fonts/segoeuib.ttf"


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size * SS)


img = Image.new("RGB", (W * SS, H * SS), "white")
d = ImageDraw.Draw(img)


def P(x, y):
    return ((20 + (x - X0) * S) * SS, (HEAD + (Y1 - y) * S) * SS)


def PT(x, y):
    return tuple(v / SS for v in P(x, y))


def text(xy, s, color=INK, size=22, bold=False, anchor="la"):
    f = font(size, bold)
    box = d.textbbox((xy[0] * SS, xy[1] * SS), s, font=f, anchor=anchor)
    pad = 5 * SS
    d.rectangle([box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad], fill="white")     # a ring never runs through a word
    d.text((xy[0] * SS, xy[1] * SS), s, fill=color, font=f, anchor=anchor)


def dashed_circle(c, r, color, width=3, dash=14.0, gap=9.0):
    step = dash + gap
    n = int(2 * math.pi * r / step)
    for i in range(n):
        a0, a1 = i * 360.0 / n, (i + dash / step) * 360.0 / n
        (cx, cy), (rx, ry) = P(*c), (r * S * SS, r * S * SS)
        d.arc([cx - rx, cy - ry, cx + rx, cy + ry], -a1, -a0, fill=color, width=width * SS)


def disc(c, r_mm, fill, edge, label, label_color):
    (cx, cy), r = P(*c), r_mm * S * SS
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill, outline=edge, width=4 * SS)
    d.text((cx, cy), label, fill=label_color, font=font(22, True), anchor="mm")


text((40, 28), "Where to put the microscope", INK, 40, True)
text((40, 84), "Top view of the cell from above, robot base frame, millimetres.", GREY, 21)
text((40, 112), "Put the microscope so its plate holder is inside the orange circle (20 mm).", GREY, 21)
text((40, 140), "The arm reaches 240 to 420 mm from its base. Numbers are from the Shanghai cell.", GREY, 21)

dashed_circle(BASE, REACH, (150, 165, 165))
dashed_circle(BASE, INNER, (150, 165, 165))
text(PT(BASE[0] - REACH * 0.71 - 8, BASE[1] + REACH * 0.71 + 8), "reach limit 420", GREY, 20, anchor="ra")
text(PT(BASE[0] - INNER * 0.71 + 12, BASE[1] - INNER * 0.71 - 22), "inner limit 240", GREY, 20, anchor="la")

# route, scope to module
d.line([P(*SEAT), P(*MODULE)], fill=(190, 190, 190), width=5 * SS)
mid = ((SEAT[0] + MODULE[0]) / 2, (SEAT[1] + MODULE[1]) / 2)
text((PT(*mid)[0] + 14, PT(*mid)[1] - 30), "route: keep it clear", GREY, 20)

# hotel
d.line([P(HOTEL_X, HOTEL_Y[0]), P(HOTEL_X, HOTEL_Y[1])], fill=GREEN, width=12 * SS)
for y, name in ((HOTEL_Y[1], "A"), (HOTEL_Y[0], "C")):
    cx, cy = P(HOTEL_X, y)
    d.ellipse([cx - 16 * SS, cy - 16 * SS, cx + 16 * SS, cy + 16 * SS], fill="white", outline=GREEN, width=4 * SS)
    d.text((cx, cy), name, fill=GREEN, font=font(20, True), anchor="mm")
ax, ay = P(HOTEL_X - 6, 0)
d.line([(ax, ay), (ax - 70 * SS, ay)], fill=GREEN, width=4 * SS)
d.polygon([(ax - 70 * SS, ay), (ax - 54 * SS, ay - 9 * SS), (ax - 54 * SS, ay + 9 * SS)], fill=GREEN)
text((P(HOTEL_X, 0)[0] / SS + 24, P(HOTEL_X, 0)[1] / SS - 28), "plate hotel", GREEN, 22, True)
text((P(HOTEL_X, 0)[0] / SS + 24, P(HOTEL_X, 0)[1] / SS + 2), "columns A to C", GREEN, 20)
text((PT(HOTEL_X, 0)[0] - 14, PT(HOTEL_X, 0)[1] - 44), "slots open toward -x", GREEN, 19, anchor="ra")

# arm, module, microscope
disc(BASE, 20, (242, 242, 242), INK, "", INK)
text((P(*BASE)[0] / SS, P(*BASE)[1] / SS + 30), "arm base  (126, 0)", INK, 22, True, "ma")
disc(MODULE, 20, (230, 238, 250), BLUE, "M", BLUE)
text((P(*MODULE)[0] / SS, P(*MODULE)[1] / SS + 28), "rotation module  (317, 305)", BLUE, 22, True, "ma")
disc(SEAT, TOL, (252, 238, 228), ORANGE, "S", ORANGE)
text((PT(*SEAT)[0] - 34, PT(*SEAT)[1] - 14), "microscope seat  (123, 400)", ORANGE, 24, True, "rm")
text((PT(*SEAT)[0] - 34, PT(*SEAT)[1] + 16), "within 20 mm, clear air above it", ORANGE, 20, anchor="rm")

# axes
ox, oy = 50, H - 40
d.line([(ox * SS, oy * SS), ((ox + 90) * SS, oy * SS)], fill=(70, 70, 70), width=4 * SS)
d.line([(ox * SS, oy * SS), (ox * SS, (oy - 90) * SS)], fill=(70, 70, 70), width=4 * SS)
text((ox + 98, oy), "x", GREY, 22, anchor="lm")
text((ox, oy - 100), "y", GREY, 22, anchor="mb")

out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "placement.png")
img.resize((W, H), Image.LANCZOS).save(out)
print(out, W, H)
