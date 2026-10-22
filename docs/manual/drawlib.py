"""Shared shapes for the manual drawings: plain shapes, the owner's palette, one thing to do in red."""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle  # noqa: E402

PNG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "png")
C_VISION, C_POLAR, C_SQUID = "#9DCBE8", "#5FA9D6", "#1F3F5F"
C_GREY, C_INK, C_RED, C_OK, C_MUTE = "#C9CED6", "#1f2933", "#b23a3a", "#2e8b57", "#555555"
DPI = 200
TYPE = 1.4                      # drawings are scaled down on the page; type is drawn larger to survive it
HOTEL_COLS, HOTEL_ROWS = "ABC", 10


def figure(w, h, xlim, ylim):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def save(fig, name):
    os.makedirs(PNG, exist_ok=True)
    fig.tight_layout(pad=0.4)
    path = os.path.join(PNG, name + ".png")
    fig.savefig(path, dpi=DPI, facecolor="white")
    plt.close(fig)
    return path


def box(ax, x, y, w, h, fc="white", ec=C_INK, lw=1.2, ls="-", z=2):
    ax.add_patch(Rectangle((x, y), w, h, fc=fc, ec=ec, lw=lw, ls=ls, zorder=z))


def rbox(ax, x, y, w, h, fc="white", ec=C_INK, lw=1.2, r=0.15, z=2):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}", fc=fc, ec=ec, lw=lw, zorder=z))


def circle(ax, x, y, r, fc="white", ec=C_INK, lw=1.2, z=3):
    ax.add_patch(Circle((x, y), r, fc=fc, ec=ec, lw=lw, zorder=z))


def text(ax, x, y, s, size=11, color=C_INK, ha="center", va="center", bold=False, z=6):
    ax.text(x, y, s, ha=ha, va=va, fontsize=size * TYPE, color=color, fontweight="bold" if bold else "normal",
            zorder=z)


def mark(ax, x, y, n, r=0.32, size=11):
    """A numbered red ring: the thing the caption line with that number talks about."""
    circle(ax, x, y, r, fc="white", ec=C_RED, lw=2.2, z=8)
    text(ax, x, y, str(n), size=size, color=C_RED, bold=True, z=9)


def arrow(ax, a, b, color=C_RED, dashed=False, rad=0.0, lw=2.0, both=False, z=7):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="<|-|>" if both else "-|>", mutation_scale=16, lw=lw, color=color,
                                 linestyle=(0, (5, 4)) if dashed else "-", connectionstyle=f"arc3,rad={rad}",
                                 zorder=z))


def line(ax, a, b, color=C_INK, lw=1.2, dashed=False, z=4):
    ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=lw, ls=(0, (5, 4)) if dashed else "-", zorder=z,
            solid_capstyle="round")


def captions(ax, lines, x, y0, dy=0.42, size=9.5):
    """The short list under a drawing: one line per numbered mark."""
    for k, s in enumerate(lines):
        text(ax, x, y0 - k * dy, s, size=size, ha="left")


def cross(ax, x, y, r=0.25, lw=2.4):
    """A red cross: do not touch."""
    line(ax, (x - r, y - r), (x + r, y + r), color=C_RED, lw=lw, z=9)
    line(ax, (x - r, y + r), (x + r, y - r), color=C_RED, lw=lw, z=9)


def button(ax, x, y, w, h, label, size=10, **_ignored):
    """A button on screen: a plain white rectangle, thin dark outline, dark text. Nothing else."""
    rbox(ax, x, y, w, h, fc="white", ec=C_INK, lw=1.0, r=0.08)
    text(ax, x + w / 2, y + h / 2, label, size=size, color=C_INK, bold=True)


# ------------------------------------------------------------------ the bench from above
def bench(ax, scopes=2, hotel_plates=("A1",)):
    """The cell from above: arm base at the bottom, stage in front of it, microscopes to the left, hotel to the right.
    Returns the centres the marks go on."""
    box(ax, 0.3, 0.3, 11.4, 7.4, ec=C_GREY, fc="white")
    circle(ax, 6, 1.3, 0.9, fc=C_POLAR)
    text(ax, 6, 1.3, "arm", color="white", bold=True)
    circle(ax, 6, 4.2, 1.1)
    box(ax, 5.35, 3.8, 1.3, 0.8, lw=1.0)
    text(ax, 6, 5.6, "stage")
    pts = {"stage": (6.0, 4.2)}
    ys = [4.4, 1.0] if scopes >= 2 else [3.3]
    for i, y in enumerate(ys[:scopes]):
        box(ax, 1.0, y, 2.6, 2.0, fc=C_SQUID)
        box(ax, 1.9, y + 0.6, 1.3, 0.8, lw=1.0)
        text(ax, 2.3, y + 2.35, f"microscope {i + 1}" if scopes > 1 else "microscope")
        pts[f"microscope {i + 1}" if scopes > 1 else "microscope"] = (2.55, y + 1.0)
    box(ax, 9.0, 1.2, 2.2, 5.6)
    for i, col in enumerate(HOTEL_COLS):
        x = 9.2 + i * 0.7
        for r in range(HOTEL_ROWS):
            slot = f"{col}{r + 1}"
            box(ax, x, 1.45 + r * 0.52, 0.55, 0.36, fc=C_POLAR if slot in hotel_plates else C_GREY, ec="#888", lw=0.5)
            pts[slot] = (x + 0.27, 1.63 + r * 0.52)
        text(ax, x + 0.27, 7.05, col, size=10)
    text(ax, 8.75, 1.63, "1", size=9, color=C_MUTE, ha="right")
    text(ax, 8.75, 6.31, "10", size=9, color=C_MUTE, ha="right")
    text(ax, 10.1, 0.75, "hotel")
    return pts


# ------------------------------------------------------------------ side views
def hand_side(ax, x, y, open_w=2.0, cam=True, z=5):
    """The arm's hand from the side, finger tips at height y, centred on x. Returns the down camera lens point."""
    box(ax, x - 0.35, y + 2.2, 0.7, 1.2, fc=C_GREY, z=z)                         # wrist
    box(ax, x - open_w / 2 - 0.45, y + 1.5, open_w + 0.9, 0.7, fc="white", z=z)  # hand body
    for sx in (-1, 1):
        box(ax, x + sx * open_w / 2 - 0.12 + (0.12 if sx < 0 else -0.12), y, 0.24, 1.5, fc=C_GREY, z=z)
    if cam:
        box(ax, x - 0.3, y + 1.0, 0.6, 0.5, fc=C_VISION, z=z)                       # down camera
        circle(ax, x, y + 1.0, 0.14, fc=C_INK, z=z + 1)
        return (x, y + 0.86)
    return (x, y + 1.0)


def plate_side(ax, x, y, w=2.6, h=0.28, fc=C_POLAR, tilt=0.0, z=4):
    """A plate seen edge on, bottom-left at (x, y). tilt raises the right end."""
    pts = [(x, y), (x + w, y + tilt), (x + w, y + tilt + h), (x, y + h)]
    ax.add_patch(Polygon(pts, closed=True, fc=fc, ec=C_INK, lw=1.2, zorder=z))


def holder_side(ax, x, y, w=3.2, h=0.6, fc=C_SQUID, z=3):
    """A plate holder seen from the side: a block with a shallow recess the plate sits in."""
    box(ax, x, y, w, h, fc=fc, z=z)
    box(ax, x + 0.25, y + h - 0.18, w - 0.5, 0.18, fc="white", lw=0.8, z=z + 1)
    return (x + 0.3, y + h - 0.18)


def hand_glyph(ax, x, y, h=1.6, color=C_INK):
    """A flat hand, fingers pointing left, as tall as the gap it measures (a hand's width)."""
    palm_w, palm_h = h * 0.45, h * 0.82
    rbox(ax, x, y + (h - palm_h) / 2, palm_w, palm_h, fc="white", ec=color, r=0.12)
    fw = palm_h / 4.6
    for k in range(4):
        fy = y + (h - palm_h) / 2 + 0.06 + k * (fw + 0.07)
        rbox(ax, x - h * 0.42, fy, h * 0.44, fw, fc="white", ec=color, r=0.06)
    rbox(ax, x + palm_w * 0.25, y - 0.02, fw, h * 0.2, fc="white", ec=color, r=0.06)


# ------------------------------------------------------------------ the hotel matrix (the screen's grid)
def matrix(ax, x0, y0, cell=0.6, gap=0.08, plates=(), learned=(), match=(), miss=(), off=(), size=9):
    """Columns A B C, row 10 on top, row 1 at the bottom. Returns {slot: (cx, cy)}."""
    pts = {}
    for i, col in enumerate(HOTEL_COLS):
        text(ax, x0 + i * (cell + gap) + cell / 2, y0 + HOTEL_ROWS * (cell + gap) + 0.15, col, size=size + 1,
             color=C_MUTE, bold=True)
    for r in range(HOTEL_ROWS):
        y = y0 + r * (cell + gap)
        text(ax, x0 - 0.2, y + cell / 2, str(r + 1), size=size, color=C_MUTE, ha="right")
        for i, col in enumerate(HOTEL_COLS):
            slot = f"{col}{r + 1}"
            x = x0 + i * (cell + gap)
            fc = C_POLAR if slot in plates else "#e5e8ec" if slot in off else "#f3f5f8"
            rbox(ax, x, y, cell, cell, fc=fc, ec="#9aa3ae" if slot not in off else "#d0d4d9", r=0.06)
            text(ax, x + cell / 2, y + cell / 2, slot, size=size, color="white" if slot in plates else
                 "#b0b7c0" if slot in off else C_INK, bold=slot in plates)
            if slot in learned:
                circle(ax, x + cell - 0.1, y + cell - 0.1, 0.055, fc=C_OK, ec=C_OK, z=7)
            if slot in match or slot in miss:
                rbox(ax, x - 0.04, y - 0.04, cell + 0.08, cell + 0.08, fc="none",
                     ec=C_OK if slot in match else C_RED, lw=2.6, r=0.08, z=6)
            pts[slot] = (x + cell / 2, y + cell / 2)
    return pts


# ------------------------------------------------------------------ the practice plate and the marker tab
MARKER_BITS = ["10110", "01001", "11010", "00101", "10011"]


def aruco(ax, x, y, s, z=6):
    """A square marker, bottom-left at (x, y), side s: black border, a fixed pattern inside."""
    box(ax, x, y, s, s, fc="black", ec="black", z=z)
    n, pad = len(MARKER_BITS), s / 7
    c = (s - 2 * pad) / n
    for r, row in enumerate(MARKER_BITS):
        for k, bit in enumerate(row):
            if bit == "1":
                box(ax, x + pad + k * c, y + pad + (n - 1 - r) * c, c, c, fc="white", ec="white", lw=0, z=z + 1)


def marker_tab(ax, x, y, scale=1.0, z=5):
    """The steel tab from the kit: 42 x 60 mm, a 40 mm marker, two screw holes 30 mm apart. Bottom-left at (x, y)."""
    w, h = 4.2 * scale, 6.0 * scale
    rbox(ax, x, y, w, h, fc="#e9ebee", ec=C_INK, r=0.1, z=z)
    aruco(ax, x + 0.1 * scale, y + 1.6 * scale, 4.0 * scale, z=z + 1)
    for hx in (w / 2 - 1.5 * scale, w / 2 + 1.5 * scale):
        circle(ax, x + hx, y + 0.8 * scale, 0.22 * scale, fc="white", z=z + 2)
    return (x + w / 2, y + h / 2)


def plate_top(ax, x, y, scale=1.0, fc=C_POLAR, z=4):
    """The practice plate from above: 127 x 85 mm, one corner cut. Bottom-left at (x, y)."""
    L, W, c = 12.7 * scale, 8.5 * scale, 0.8 * scale
    pts = [(x, y), (x + L - c, y), (x + L, y + c), (x + L, y + W), (x, y + W)]
    ax.add_patch(Polygon(pts, closed=True, fc=fc, ec=C_INK, lw=1.2, zorder=z))
    return (x + L / 2, y + W / 2)
