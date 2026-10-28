"""docs/placement.png: the cell from above in the robot base frame, black and white, one scale (mm), every distance on
the figure. Positions and boxes come from layout.json, so the figure follows the file.
    python tools/placement_diagram.py
"""
import json
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from routes_opt import fk  # noqa: E402

L = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))
F, R = L["fixtures"], L["regions"]
BASE, SEAT, MODULE = (126.0, 0.0), (123.0, 400.0), (317.0, 305.0)    # the nominal placement; the README distance table uses the same
HOTEL_X = 293.0                                    # slot centre x; columns A and C at y = +-137.5
A, C = (HOTEL_X, 137.5), (HOTEL_X, -137.5)
ELBOW = fk.elbow_xy(L["trajectories"]["microscope_to_polar"]["joints"][-1])     # the elbow on the taught approach to the module
TOL = 20
WB = L["workspace_box"]

S, SS = 1.6, 2                                     # px per mm, supersampling
X0, X1, Y0, Y1 = -60, 430, -215, 480               # world window
TOP = 90
W, H = int((X1 - X0) * S) + 40, int((Y1 - Y0) * S) + TOP + 50
FONT, BOLD = "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/segoeuib.ttf"
BLACK, GREY, LIGHT = (0, 0, 0), (110, 110, 110), (232, 232, 232)

img = Image.new("RGB", (W * SS, H * SS), "white")
d = ImageDraw.Draw(img)


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size * SS)


def P(p):
    return ((20 + (p[0] - X0) * S) * SS, (TOP + (Y1 - p[1]) * S) * SS)


def label(xy, s, size=16, bold=False, anchor="mm", color=BLACK):
    f = font(size, bold)
    x, y = xy[0] * SS, xy[1] * SS
    b = d.textbbox((x, y), s, font=f, anchor=anchor)
    pad = 3 * SS
    d.rectangle([b[0] - pad, b[1] - pad, b[2] + pad, b[3] + pad], fill="white")
    d.text((x, y), s, fill=color, font=f, anchor=anchor)


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
        (dashed(a, b, w, color) if dash else line(a, b, w, color))


def disc(c, r_mm, fill="white", w=3):
    (cx, cy), r = P(c), r_mm * S * SS
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill, outline=BLACK, width=w * SS)


def arrow(p, q, size=9):
    (x0, y0), (x1, y1) = P(p), P(q)
    a = math.atan2(y1 - y0, x1 - x0)
    s = size * SS
    d.polygon([(x1, y1), (x1 - s * math.cos(a - 0.35), y1 - s * math.sin(a - 0.35)),
               (x1 - s * math.cos(a + 0.35), y1 - s * math.sin(a + 0.35))], fill=BLACK)


def dimension(p, q, text, off=0.0, side=1, size=16):
    """A measure: a line offset sideways from p-q by `off` mm, arrows at both ends, the value on it."""
    ux, uy = q[0] - p[0], q[1] - p[1]
    n = math.hypot(ux, uy)
    nx, ny = -uy / n * side, ux / n * side
    a, b = (p[0] + nx * off, p[1] + ny * off), (q[0] + nx * off, q[1] + ny * off)
    if off:
        dashed(p, a, 1, GREY, 4, 3)
        dashed(q, b, 1, GREY, 4, 3)
    line(a, b, 1)
    arrow(b, a)
    arrow(a, b)
    m = P(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
    label((m[0] / SS, m[1] / SS), text, size)


d.text((20 * SS, 14 * SS), "The cell from above", fill=BLACK, font=font(30, True))
d.text((20 * SS, 52 * SS), "Robot base frame, millimetres, one scale: 100 mm is the bar at lower left. Coordinates are x, y.",
       fill=GREY, font=font(15))

# workspace and zones
rect(WB["x"], WB["y"], dash=True, w=2, color=GREY)
label((P((WB["x"][1], WB["y"][1]))[0] / SS - 4, P((WB["x"][1], WB["y"][1]))[1] / SS + 12), "arm workspace", 14, anchor="rm", color=GREY)
mz = R["microscope"]["box"]
rect(mz["x"], mz["y"], fill=LIGHT, dash=True)
label((P((mz["x"][0], mz["y"][1]))[0] / SS + 6, P((mz["x"][0], mz["y"][1]))[1] / SS + 12), "microscope zone (arm moves straight down only)", 13, anchor="lm")
pz = R["polar"]["box"]
rect(pz["x"], pz["y"], fill=LIGHT)
hz = R["hotel"]["box"]
rect(hz["x"], hz["y"], fill=LIGHT)

# hotel columns
line((HOTEL_X, -150), (HOTEL_X, 150), 4)
for name, c in (("A", A), ("C", C)):
    disc(c, 9, "white", 3)
    d.text(P(c), name, fill=BLACK, font=font(15, True), anchor="mm")
label((P((HOTEL_X + 40, 0))[0] / SS, P((HOTEL_X + 40, 0))[1] / SS), "plate hotel\nslots open toward -x", 15, bold=False)

# the arm reaching the module: base, link 1, elbow, link 2
line(BASE, ELBOW, 7)
line(ELBOW, MODULE, 7)
line(BASE, ELBOW, 3, "white")
line(ELBOW, MODULE, 3, "white")
disc(BASE, 14, LIGHT)
disc(ELBOW, 10, "white")
label((P(BASE)[0] / SS - 24, P(BASE)[1] / SS + 40), "arm base\n(126, 0)", 15, True, "rm")
mid1 = ((BASE[0] + ELBOW[0]) / 2, (BASE[1] + ELBOW[1]) / 2)
mid2 = ((ELBOW[0] + MODULE[0]) / 2, (ELBOW[1] + MODULE[1]) / 2)
label((P(mid1)[0] / SS + 22, P(mid1)[1] / SS + 10), "link 200", 14)
label((P(mid2)[0] / SS - 34, P(mid2)[1] / SS), "link 200", 14)

# module and microscope seat
disc(MODULE, 7, "black")
label((P(MODULE)[0] / SS + 52, P(MODULE)[1] / SS - 30), f"rotation module ({MODULE[0]:.0f}, {MODULE[1]:.0f})", 15, True, "mm")
disc(SEAT, TOL, "white", 3)
d.line([(P(SEAT)[0] - 6 * SS, P(SEAT)[1]), (P(SEAT)[0] + 6 * SS, P(SEAT)[1])], fill=BLACK, width=2 * SS)
d.line([(P(SEAT)[0], P(SEAT)[1] - 6 * SS), (P(SEAT)[0], P(SEAT)[1] + 6 * SS)], fill=BLACK, width=2 * SS)
label((P(SEAT)[0] / SS + 14, P(SEAT)[1] / SS + 58), f"microscope seat ({SEAT[0]:.0f}, {SEAT[1]:.0f})\nplace within {TOL} mm", 15, True, "lm")

# measures
dist = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1])
dimension(BASE, SEAT, f"{dist(BASE, SEAT):.1f}", off=26, side=1)
dimension(SEAT, MODULE, f"{dist(SEAT, MODULE):.1f}", off=26, side=-1)
dimension(BASE, A, f"{dist(BASE, A):.1f}", off=22, side=1)
dimension(BASE, C, f"{dist(BASE, C):.1f}", off=22, side=-1)
dimension(MODULE, A, f"{dist(MODULE, A):.1f}", off=26, side=1)
dimension(A, C, f"{dist(A, C):.1f}", off=48, side=-1)

# axes and scale bar
ox, oy = 40, H - 30
line_px = lambda a, b: d.line([(a[0] * SS, a[1] * SS), (b[0] * SS, b[1] * SS)], fill=BLACK, width=3 * SS)
line_px((ox, oy), (ox + 60, oy))
line_px((ox, oy), (ox, oy - 60))
d.text(((ox + 68) * SS, oy * SS), "x", fill=BLACK, font=font(16, True), anchor="lm")
d.text((ox * SS, (oy - 68) * SS), "y", fill=BLACK, font=font(16, True), anchor="mb")
bx = ox + 150
line_px((bx, oy), (bx + 100 * S, oy))
line_px((bx, oy - 6), (bx, oy + 6))
line_px((bx + 100 * S, oy - 6), (bx + 100 * S, oy + 6))
d.text(((bx + 50 * S) * SS, (oy - 10) * SS), "100 mm", fill=BLACK, font=font(14), anchor="mb")

out = os.path.join(HERE, "docs", "placement.png")
img.resize((W, H), Image.LANCZOS).save(out)
print(out, W, H, "elbow", [round(v, 1) for v in ELBOW])
