"""docs/img/cell_model.png: the workcell as a line drawing, white faces, black edges. python utils/cell_figure.py [--flip]"""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from PIL import Image, ImageDraw
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AZ, EL = math.radians(float(next((a[5:] for a in sys.argv if a.startswith("--az=")), -38.0))), math.radians(27.0)
def rot(x, y, z):
    y = -y
    ca, sa, ce, se = math.cos(AZ), math.sin(AZ), math.cos(EL), math.sin(EL)
    xr, yr = x * ca - y * sa, x * sa + y * ca
    return xr, z * ce - yr * se, yr * ce + z * se
GREY = set()                                                     # the slots: drawn grey
def boxes():
    L = json.load(open(os.path.join(HERE, "run", "layout.json"), encoding="utf-8"))
    from utils.hotel_model_html import scene_boxes
    out = [((-95, 554), (-200, 200), (-47, 48)),            # platform
           ((-80, 60), (-74, 74), (33, 703))]               # arm column
    for b in scene_boxes(L):
        if b["k"] == "post":
            out.append((tuple(b["x"]), tuple(b["y"]), tuple(b["z"])))
        elif b["k"] in ("tray", "hotel") and b["x"][0] > 400 and (b["x"][1] - b["x"][0]) > 20:
            box = ((b["x"][0], min(b["x"][1], 511.0)), tuple(b["y"]), tuple(b["z"]))   # stop at the posts: no interpenetration
            out.append(box)
            if b["k"] == "tray":
                GREY.add(id(box))
    P0 = L["fixtures"]["polar"]["pose_at_0"]["pose"]
    h = json.load(open(os.path.join(HERE, "run", "hardware.json"), encoding="utf-8"))["cad"]["rotation_module"]
    out.append((tuple(h["plate_seat_x_r"]), tuple(h["plate_seat_y_r"]), (48.0, P0[2] - 12.0)))   # module pad, from the platform top
    if "--microscope" in sys.argv:                                   # the instrument's box, as the GUI draws it
        m = L["volumes"]["obstacles"]["microscope"]
        out.append((tuple(m["x"]), tuple(m["y"]), (max(m["z"][0], -47.0), m["z"][1])))
    return out
def prism(a, b, w, h, z):
    """A box from point a to point b (x, y), w wide, from z to z + h: eight corners, the axis-aligned box's corner order."""
    ax, ay = a; bx, by = b
    L = math.hypot(bx - ax, by - ay); ux, uy = (bx - ax) / L, (by - ay) / L; nx, ny = -uy * w / 2, ux * w / 2
    return {(i, j, k): ((ax, bx)[i] + (-nx, nx)[j], (ay, by)[i] + (-ny, ny)[j], (z, z + h)[k]) for i in (0, 1) for j in (0, 1) for k in (0, 1)}
def arm(L):
    """Carriage on the column, shoulder, two 200 mm links to a wrist above the rotation module, the claw ahead of it."""
    h = json.load(open(os.path.join(HERE, "run", "hardware.json"), encoding="utf-8"))["cad"]["rotation_module"]
    S = (126.0, 0.0); W = (h["axis_r"][0], h["axis_r"][1]); zc = 520.0
    d = math.hypot(W[0] - S[0], W[1] - S[1]); a = math.atan2(W[1] - S[1], W[0] - S[0]); b = math.acos(min(1.0, d / 400.0))
    E = (S[0] + 200.0 * math.cos(a - b), S[1] + 200.0 * math.sin(a - b))
    out = [((60.0, 156.0), (-45.0, 45.0), (zc - 40.0, zc + 40.0))]       # shoulder block on the front of the column
    out.append(prism(S, E, 70.0, 40.0, zc - 20.0))                       # link 1
    out.append(prism(E, W, 60.0, 30.0, zc - 60.0))                       # link 2
    out.append(prism(W, (W[0] + 60.0 * (W[0] - E[0]) / 200.0, W[1] + 60.0 * (W[1] - E[1]) / 200.0), 50.0, 70.0, zc - 130.0))   # wrist and claw
    return out
def corners(b):
    if isinstance(b, dict):
        return b
    (x0, x1), (y0, y1), (z0, z1) = b
    return {(i, j, k): ((x0, x1)[i], (y0, y1)[j], (z0, z1)[k]) for i in (0, 1) for j in (0, 1) for k in (0, 1)}
def faces(b):
    c = corners(b)
    q = lambda *ks: [c[k] for k in ks]
    return [q((0,0,0),(1,0,0),(1,1,0),(0,1,0)), q((0,0,1),(1,0,1),(1,1,1),(0,1,1)),
            q((0,0,0),(1,0,0),(1,0,1),(0,0,1)), q((0,1,0),(1,1,0),(1,1,1),(0,1,1)),
            q((0,0,0),(0,1,0),(0,1,1),(0,0,1)), q((1,0,0),(1,1,0),(1,1,1),(1,0,1))]
def main(flip=False, W=1400, H=1300):
    bs = boxes()
    bs += arm(None)
    pts = [rot(*q) for b in bs for q in corners(b).values()]
    u0, u1 = min(p[0] for p in pts), max(p[0] for p in pts); v0, v1 = min(p[1] for p in pts), max(p[1] for p in pts)
    s = 0.94 * min(W / (u1 - u0), H / (v1 - v0)); ox, oy = W / 2 - s * (u0 + u1) / 2, H / 2 + s * (v0 + v1) / 2
    im = Image.new("RGB", (W, H), "white"); d = ImageDraw.Draw(im)
    depth = lambda p: rot(*p)[2] * (-1 if flip else 1)
    centre = lambda b: depth(tuple(sum(q[i] for q in corners(b).values()) / 8 for i in range(3)))
    for b in [bs[0]] + sorted(bs[2:], key=centre) + [bs[1]]:        # platform first, far to near, the column in front of everything                                   # far boxes first, then each box back face to front face
        for _, f in sorted(((sum(depth(p) for p in f) / 4, f) for f in faces(b)), key=lambda t: t[0]):
            poly = [(ox + rot(*p)[0] * s, oy - rot(*p)[1] * s) for p in f]
            d.polygon(poly, fill="#d0d0d0" if id(b) in GREY else "white", outline="black")
    im = im.resize((W // 2, H // 2), Image.LANCZOS)
    out = next((a[6:] for a in sys.argv if a.startswith("--out=")), os.path.join(HERE, "docs", "img", "cell_model.png")); im.save(out); print(out)
if __name__ == "__main__":
    main(flip="--flip" in sys.argv)
