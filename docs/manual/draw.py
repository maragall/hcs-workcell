"""The manual's drawings, one per step. `python3 draw.py` writes every PNG into png/."""
from drawlib import (C_GREY, C_INK, C_MUTE, C_OK, C_POLAR, C_RED, C_SQUID, C_VISION, aruco, arrow, bench, box,
                     button, captions, circle, cross, figure, hand_glyph, hand_side, holder_side, line, mark,
                     marker_tab, matrix, plate_side, plate_top, rbox, save, text)

LOADED = ("A1", "C2", "A3", "C4", "A5", "C6", "A7", "C8", "A9", "C10")


# ------------------------------------------------------------------ 0 cables
def fig_cables():
    fig, ax = figure(9, 6.2, (0, 12), (-1.9, 7.2))
    box(ax, 4.6, 0.6, 2.8, 6.0, fc="#eef0f3")                                     # back of the PC
    text(ax, 6.0, 6.25, "PC, back", bold=True)
    for k, (y, lab) in enumerate(((4.6, "LAN"), (3.7, "LAN"), (2.2, "USB"))):
        box(ax, 5.2, y, 0.9, 0.55, fc="white")
        text(ax, 6.5, y + 0.27, lab, size=10, color=C_MUTE, ha="left")
    box(ax, 0.4, 3.6, 2.9, 2.6, fc=C_POLAR)                                      # arm controller
    text(ax, 1.85, 5.7, "arm", color="white", bold=True)
    box(ax, 2.5, 4.0, 0.55, 0.45, fc="white")
    text(ax, 2.77, 3.75, "LAN 1", size=7, color="white")
    circle(ax, 1.2, 4.4, 0.42, fc="white")                                        # key switch
    line(ax, (1.2, 4.4), (1.42, 4.76), color=C_RED, lw=3)
    text(ax, 0.7, 5.0, "Manual", size=6.5, color="white")
    text(ax, 1.8, 5.0, "Auto", size=7.5, color="white", bold=True)
    box(ax, 0.6, 4.0, 0.3, 0.3, fc=C_OK, ec=C_OK)                                 # power
    text(ax, 0.75, 3.75, "power", size=6.5, color="white")
    box(ax, 8.9, 4.4, 2.6, 1.4, fc=C_VISION)                                       # vision box
    text(ax, 10.2, 5.1, "vision box", bold=True)
    box(ax, 8.6, 1.6, 3.3, 1.4, fc=C_SQUID)                                        # squid controller
    text(ax, 10.25, 2.3, "Squid controller", size=9, color="white", bold=True)
    arrow(ax, (3.05, 4.22), (5.2, 4.87), rad=-0.15, lw=2.4)                       # 1 arm LAN 1 -> PC
    arrow(ax, (8.9, 4.7), (6.1, 3.97), rad=-0.15, lw=2.4)                         # 2 vision box -> PC
    arrow(ax, (8.9, 2.2), (6.1, 2.47), rad=0.1, lw=2.4)                           # 3 squid -> PC
    mark(ax, 4.0, 5.0, 1)
    mark(ax, 7.8, 4.7, 2)
    mark(ax, 7.8, 2.0, 3)
    mark(ax, 1.85, 3.1, 4)
    captions(ax, ("1  arm, port LAN 1, to the PC", "2  vision box to the PC", "3  Squid controller USB to the PC",
                  "4  arm power on, key at Auto"), 0.5, -0.5)
    save(fig, "0_cables")


# ------------------------------------------------------------------ 1 cameras
def fig_cameras():
    fig, ax = figure(9, 5.6, (0, 12), (-1.7, 6.4))
    lens = hand_side(ax, 4.0, 1.0, open_w=2.4)
    box(ax, 5.05, 2.0, 0.5, 0.6, fc=C_VISION)                                     # front camera on the hand
    circle(ax, 5.55, 2.3, 0.13, fc=C_INK)
    circle(ax, lens[0], lens[1] - 0.05, 0.3, fc="none", ec=C_INK, lw=2.0)        # focus ring, down camera
    circle(ax, 5.62, 2.3, 0.3, fc="none", ec=C_INK, lw=2.0)                      # focus ring, front camera
    cross(ax, lens[0], lens[1] - 0.05, r=0.38)
    cross(ax, 5.62, 2.3, r=0.38)
    text(ax, 4.0, 4.6, "the arm's hand, from the side", bold=True)
    text(ax, 2.0, 1.7, "down camera", size=10, color=C_MUTE, ha="right")
    line(ax, (2.05, 1.7), (3.6, 1.85), color=C_MUTE, lw=0.8)
    text(ax, 5.2, 0.2, "front camera", size=10, color=C_MUTE, ha="left")
    line(ax, (5.9, 0.45), (5.7, 1.95), color=C_MUTE, lw=0.8)
    mark(ax, 2.6, 0.5, 1)
    box(ax, 8.6, 0.6, 3.0, 3.6, fc="#f7f8fa", ec=C_GREY)
    text(ax, 10.1, 3.75, "VIB tool", bold=True)
    text(ax, 10.1, 3.2, "focus", size=10)
    text(ax, 10.1, 2.7, "calibration", size=10)
    text(ax, 10.1, 1.5, "once, by\nBioyond's tool", size=8.5, color=C_MUTE)
    mark(ax, 8.6, 4.2, 2)
    captions(ax, ("1  the two focus rings: do not touch them",
                  "2  a camera moved or refocused: the VIB tool again, then this manual again"), 0.5, -0.5)
    save(fig, "1_cameras")


def fig_marker_tab():
    fig, ax = figure(9, 5.4, (0, 17.5), (-2.0, 10.8))
    plate_top(ax, 0.8, 0.8)
    text(ax, 7.15, 9.9, "practice plate, from above", bold=True)
    text(ax, 7.15, 0.35, "127 mm", size=9, color=C_MUTE)
    text(ax, 0.3, 5.05, "85 mm", size=9, color=C_MUTE, ha="right")
    marker_tab(ax, 5.05, 2.05)                                                     # the tab, on the plate
    marker_tab(ax, 11.4, 1.6, scale=1.0)                                            # the tab alone, from the kit
    text(ax, 13.5, 9.9, "from the kit", size=9, bold=True)
    for hx in (13.5 - 1.5, 13.5 + 1.5):
        arrow(ax, (hx, 0.3), (hx, 2.1), lw=1.6)
    text(ax, 13.5, -0.2, "two screws", size=9.5, color=C_RED)
    arrow(ax, (11.2, 4.6), (9.4, 4.6), lw=2.0)
    mark(ax, 1.8, 9.9, 1)
    captions(ax, ("1  screw the tab onto the practice plate. The box reads the marker",), 0.5, -1.2)
    save(fig, "1_marker_tab")


# ------------------------------------------------------------------ 2 connect
def fig_connect():
    fig, ax = figure(9, 5.0, (0, 12), (-1.9, 5.4))
    rbox(ax, 0.6, 0.2, 10.8, 4.9, fc="white", ec=C_GREY, r=0.2)
    text(ax, 1.0, 4.6, "Setup", size=14, bold=True, ha="left")
    rows = (("Arm", True), ("Vision box", True), ("Rotation module", True), ("Squid microscope", False))
    for k, (name, ok) in enumerate(rows):
        y = 3.7 - k * 0.85
        text(ax, 1.0, y, name, size=12, ha="left")
        circle(ax, 5.6, y, 0.22, fc=C_OK if ok else "white", ec=C_OK if ok else C_RED, lw=1.6)
        text(ax, 6.1, y, "found" if ok else "not found", size=11, color=C_OK if ok else C_RED, ha="left")
        if not ok:
            button(ax, 8.2, y - 0.34, 2.7, 0.68, "Search again", size=9)
            mark(ax, 11.15, y + 0.42, 1)
    captions(ax, ("1  a row not green: power, key at Auto, that device's cable, then Search again",), 0.5, -0.6)
    save(fig, "2_connect")


# ------------------------------------------------------------------ 2 setup: the hotel
def fig_hotel_size():
    fig, ax = figure(9, 6.8, (0, 12), (-1.9, 7.6))
    button(ax, 0.8, 6.3, 2.3, 0.7, "15 slots", size=10)
    button(ax, 3.4, 6.3, 2.3, 0.7, "30 slots", size=10)
    mark(ax, 6.2, 6.65, 1)
    even = tuple(f"{c}{r}" for c in "ABC" for r in range(2, 11, 2))
    matrix(ax, 1.0, 0.3, cell=0.42, gap=0.06, off=even, size=5.5)
    text(ax, 1.7, -0.1, "15 slots", size=10, color=C_MUTE, ha="left")
    matrix(ax, 7.0, 0.3, cell=0.42, gap=0.06, size=5.5)
    text(ax, 7.7, -0.1, "30 slots", size=10, color=C_MUTE, ha="left")
    text(ax, 6.0, -0.7, "row 10 is the top shelf, row 1 the bottom", size=10, color=C_MUTE)
    captions(ax, ("1  press the hotel you have: 15 slots (every other shelf) or 30 slots",), 0.5, -1.3)
    save(fig, "2_hotel_size")


# ------------------------------------------------------------------ 3 teach
def fig_teach_bench():
    fig, ax = figure(9, 8.3, (0, 12), (-3.5, 8.2))
    pts = bench(ax, scopes=1)
    stage = pts["stage"]
    mark(ax, *pts["microscope"], 1)
    mark(ax, stage[0] - 0.45, stage[1], 2)
    mark(ax, stage[0] + 0.45, stage[1], 3)
    arrow(ax, (stage[0] + 1.3, stage[1] + 0.6), (stage[0] + 0.6, stage[1] + 1.25), rad=0.35, lw=1.4, color=C_INK)
    text(ax, stage[0] + 1.6, stage[1] + 1.2, "turn", size=9, color=C_MUTE)
    for k, slot in enumerate(("A1", "A10", "C1", "C10")):
        mark(ax, pts[slot][0], pts[slot][1], 4 + k, r=0.26, size=9)
    arrow(ax, (3.0, pts["microscope"][1]), (stage[0] - 0.9, stage[1] + 0.3), dashed=True, rad=-0.2)
    arrow(ax, (stage[0] + 0.9, stage[1] - 0.3), (8.9, 1.9), dashed=True, rad=-0.25)
    text(ax, 6, 7.95, "put the practice plate at each mark, hover the hand over it, press Record", size=11.5)
    captions(ax, ("1  the microscope holder, plate in the holder",
                  "2  rotation module at 0, plate seated, short edges in the fingers",
                  "3  rotation module at 90, plate seated, long edges in the fingers",
                  "4  hotel A1       5  hotel A10       6  hotel C1       7  hotel C10",
                  "dashed: the drag between two Records is the route the arm will use"), 0.5, -0.6)
    save(fig, "3_teach_bench")


def fig_hover():
    fig, ax = figure(9, 5.6, (0, 12), (-1.6, 6.6))
    line(ax, (0.5, 0.3), (11.5, 0.3), color=C_GREY, lw=1.5)
    seat = holder_side(ax, 2.2, 0.3, w=3.6, h=0.7)
    plate_side(ax, seat[0], seat[1], w=3.0)
    aruco(ax, 3.4, seat[1] + 0.28, 0.6)                                            # the marker on the plate
    gap_lo, gap_hi = seat[1] + 0.88, seat[1] + 0.88 + 1.7
    lens = hand_side(ax, 4.0, gap_hi, open_w=3.6)
    for sx in (-1, 1):
        line(ax, lens, (3.7 + sx * 0.6, seat[1] + 0.9), color=C_RED, dashed=True, lw=1.4)
    arrow(ax, (6.6, gap_lo), (6.6, gap_hi), both=True, lw=1.6, color=C_INK)
    hand_glyph(ax, 7.6, gap_lo, h=gap_hi - gap_lo)
    text(ax, 9.3, (gap_lo + gap_hi) / 2, "a hand's width", size=11, ha="left")
    mark(ax, 1.5, gap_hi + 0.3, 1)
    mark(ax, 1.5, seat[1] + 0.3, 2)
    captions(ax, ("1  a hand's width up, the marker whole in the picture: press Record",
                  "2  the plate rests where it will rest in a run"), 0.5, -0.6, dy=0.55)
    save(fig, "3_hover")


def fig_grip():
    fig, ax = figure(9, 5.4, (0, 30), (-5.0, 13.0))
    for k, (x0, mode, horizontal) in enumerate(((1.5, "short edges", True), (16.0, "long edges", False))):
        c = plate_top(ax, x0, 2.0)
        marker_tab(ax, c[0] - 2.1, c[1] - 3.0)
        if horizontal:                                                            # fingers on the 85 mm ends
            box(ax, x0 - 1.2, 2.5, 1.0, 7.5, fc=C_RED, ec=C_RED)
            box(ax, x0 + 12.9, 2.5, 1.0, 7.5, fc=C_RED, ec=C_RED)
            arrow(ax, (x0 - 1.8, 11.3), (x0 + 1.3, 11.3), lw=1.6)
            arrow(ax, (x0 + 14.5, 11.3), (x0 + 11.4, 11.3), lw=1.6)
        else:                                                                     # fingers on the 127 mm sides
            box(ax, x0 + 2.0, 0.8, 8.5, 1.0, fc=C_RED, ec=C_RED)
            box(ax, x0 + 2.0, 10.7, 8.5, 1.0, fc=C_RED, ec=C_RED)
            arrow(ax, (x0 + 14.2, 0.2), (x0 + 14.2, 2.7), lw=1.6)
            arrow(ax, (x0 + 14.2, 12.3), (x0 + 14.2, 9.8), lw=1.6)
        text(ax, c[0], 12.5, mode, size=12, bold=True)
        mark(ax, x0 + 13.5, -0.3, k + 1, r=0.7, size=11)
    captions(ax, ("1  short edges, the two ends: microscope holder, rotation module at 0",
                  "2  long edges, the two sides: rotation module at 90, hotel"), 1.0, -2.6, dy=1.4)
    save(fig, "3_grip")


def fig_travel():
    fig, ax = figure(9, 5.4, (0, 12), (-1.6, 7.0))
    line(ax, (0.5, 0.3), (11.5, 0.3), color=C_GREY, lw=1.5)
    box(ax, 0.9, 0.3, 2.2, 2.6, fc=C_SQUID)                                       # microscope
    text(ax, 2.0, 1.5, "microscope", size=9, color="white")
    holder_side(ax, 4.5, 0.3, w=2.4, h=0.8, fc="white")
    text(ax, 5.7, 0.0, "rotation module", size=9, color=C_MUTE)
    box(ax, 8.6, 0.3, 2.6, 3.6, fc="white")                                       # hotel
    for r in range(7):
        box(ax, 8.8, 0.5 + r * 0.48, 2.2, 0.08, fc=C_GREY, ec=C_GREY)
    text(ax, 9.9, 4.15, "hotel", size=9, color=C_MUTE)
    hand_side(ax, 5.7, 4.6, open_w=2.0, cam=False)
    line(ax, (0.5, 4.6), (11.5, 4.6), color=C_RED, dashed=True, lw=2.0)
    mark(ax, 0.9, 5.1, 1)
    text(ax, 11.4, 4.95, "travel height", size=10.5, color=C_RED, ha="right")
    captions(ax, ("1  lift the hand until it clears everything on the bench, press Record",), 0.5, -0.7)
    save(fig, "3_travel")


def fig_replay():
    fig, ax = figure(9, 6.4, (0, 12), (-1.6, 8.2))
    pts = bench(ax, scopes=1)
    arrow(ax, (3.0, pts["microscope"][1]), (5.1, 4.5), dashed=True, rad=-0.2, color=C_INK)
    arrow(ax, (6.9, 3.9), (8.9, 1.9), dashed=True, rad=-0.25, color=C_INK)
    button(ax, 0.6, -1.3, 3.4, 0.8, "Replay slowly", size=10)
    button(ax, 9.4, -1.3, 2.0, 0.8, "STOP", size=11)
    mark(ax, 4.4, -0.9, 1)
    mark(ax, 9.0, -0.9, 2)
    text(ax, 6, 7.95, "the arm repeats every route at low speed while you watch", size=11.5)
    save(fig, "3_replay")


# ------------------------------------------------------------------ 4 test
def fig_test():
    fig, ax = figure(9, 5.6, (0, 14), (-1.9, 7.0))
    text(ax, 1.0, 6.5, "Does the bench match your clicks?", size=12, bold=True, ha="left")
    button(ax, 1.0, 4.8, 4.0, 0.8, "Test everything", size=10)
    mark(ax, 5.5, 5.2, 2)
    pts = matrix(ax, 10.2, 0.0, cell=0.52, gap=0.07, plates=LOADED, learned=("A1", "A3", "C2", "C4"),
                 match=("A1", "A3", "C4"), miss=("C2",), size=6)
    mark(ax, 9.4, pts["A1"][1], 1)
    mark(ax, 9.4, pts["C2"][1], 3)
    for k, (col, lab) in enumerate(((C_OK, "green ring: the camera agrees with you"),
                                    (C_RED, "red ring: it disagrees. Look at the slot"),
                                    (None, "no ring: the camera does not know this slot yet"))):
        y = 3.4 - k * 0.8
        rbox(ax, 1.0, y - 0.22, 0.44, 0.44, fc=C_POLAR, ec=col or "#9aa3ae", lw=2.4 if col else 1.0, r=0.06)
        text(ax, 1.7, y, lab, size=9.5, ha="left")
    captions(ax, ("1  click the slots that really hold a plate",
                  "2  one button. The arm does the whole test on its own, slowly",
                  "3  a red ring: go and look at that slot"), 0.5, -0.5, dy=0.5)
    save(fig, "4_test")


def fig_test_run():
    fig, ax = figure(9, 8.4, (0, 12), (-4.2, 7.9))
    pts = bench(ax, scopes=1, hotel_plates=LOADED)
    stage, scope = pts["stage"], pts["microscope"]
    mark(ax, 8.6, 4.6, 1)                                                          # every slot
    mark(ax, scope[0] - 0.9, scope[1] - 0.6, 2)                                    # marker at the microscope
    mark(ax, stage[0] - 0.55, stage[1] + 0.65, 2)                                  # marker at the rotation module
    arrow(ax, (3.0, scope[1]), (stage[0] - 1.2, stage[1] + 0.2), rad=-0.2, lw=2.0, both=True)
    mark(ax, 4.3, 5.4, 3)                                                          # the path
    arrow(ax, (stage[0] + 1.3, stage[1] + 0.6), (stage[0] + 0.6, stage[1] + 1.25), rad=0.35, lw=1.4,
          color=C_INK)
    mark(ax, stage[0] + 1.8, stage[1] + 1.3, 4)                                    # turns the rotation module
    cx, cy = pts["A3"]
    cross(ax, cx, cy, r=0.22, lw=2.0)
    arrow(ax, (cx - 0.5, cy), (pts["B3"][0] - 0.3, pts["B3"][1] + 0.5), rad=-0.3, lw=1.4)
    mark(ax, 8.55, cy + 0.65, 5)                                                   # taken slot
    arrow(ax, (8.9, pts["A1"][1] + 0.1), (stage[0] + 1.0, stage[1] - 0.2), rad=0.3, lw=2.0, both=True)
    mark(ax, 8.6, 1.0, 6)                                                          # every plate
    captions(ax, ("1  looks into every slot and compares with your clicks",
                  "2  reads the marker at every taught position",
                  "3  drives the path slowly between the microscope and the rotation module",
                  "4  turns the rotation module, camera check first",
                  "5  orders a plate into a taken slot on purpose, backs out, uses another slot",
                  "6  takes every plate to the microscope and back, in turn: A1, C2, A3, ..."),
             0.5, -0.6, dy=0.55)
    save(fig, "4_test_run")


# ------------------------------------------------------------------ 5 run
def fig_run():
    fig, ax = figure(9, 3.6, (0, 12), (-1.3, 3.2))
    button(ax, 0.8, 1.2, 3.0, 1.3, "Start", size=14)
    mark(ax, 4.3, 2.6, 1)
    button(ax, 8.2, 1.2, 3.0, 1.3, "STOP", size=14)
    mark(ax, 11.7, 2.6, 2)
    captions(ax, ("1  the arm looks into every slot, shows what it found, then runs every plate in turn",
                  "2  STOP at any time. The arm holds still. Start again continues"), 0.5, -0.4)
    save(fig, "5_run")


# ------------------------------------------------------------------ 6 if something stops
def fig_fault():
    fig, ax = figure(9, 5.0, (0, 12), (-1.6, 5.8))
    line(ax, (0.5, 0.3), (5.5, 0.3), color=C_GREY, lw=1.5)
    seat = holder_side(ax, 1.0, 0.3, w=3.2, h=0.7)
    hand_side(ax, 2.6, seat[1] + 1.8, open_w=3.2)
    plate_side(ax, 1.4, seat[1] + 1.85, w=2.4)
    text(ax, 5.6, 5.3, "holding still", size=10, color=C_MUTE, ha="right")
    rbox(ax, 6.2, 0.6, 5.3, 4.4, fc="#dfe3e8", ec=C_GREY, r=0.15)
    rbox(ax, 6.5, 2.6, 4.7, 0.9, fc="white", ec=C_RED, lw=2.0, r=0.08)
    text(ax, 6.7, 3.05, "slot C2 looks taken", size=11, ha="left")
    mark(ax, 11.5, 3.7, 1)
    captions(ax, ("1  one line on screen says what the arm saw. Read it, then the next three pages",), 0.5, -0.7)
    save(fig, "6_fault")


def fig_cause_slot():
    fig, ax = figure(9, 4.6, (0, 12), (-1.4, 5.8))
    box(ax, 0.8, 0.3, 3.2, 4.0, fc="white")
    for r in range(6):
        y = 0.6 + r * 0.6
        box(ax, 1.0, y, 2.8, 0.08, fc=C_GREY, ec=C_GREY)
        if r == 1:
            plate_side(ax, 1.3, y + 0.09, w=2.2, h=0.22)
            mark(ax, 0.4, y + 0.2, 1)
    text(ax, 2.4, 4.6, "hotel, from the front", size=10, color=C_MUTE)
    pts = matrix(ax, 5.6, 0.0, cell=0.38, gap=0.05, plates=("A1", "A3"), miss=("C2",), size=5)
    text(ax, 6.2, 5.4, "the screen", size=10, color=C_MUTE)
    text(ax, 8.4, 2.6, "the plan says empty,", size=11, ha="left")
    text(ax, 8.4, 2.1, "the camera sees a plate", size=11, ha="left")
    captions(ax, ("1  a plate someone left in a slot: take it out, or click that slot in Test. Then Start",),
             0.5, -0.6)
    save(fig, "6_cause_slot")


def fig_cause_seat():
    fig, ax = figure(9, 4.2, (0, 12), (-1.4, 4.4))
    for x0, tilt, lab in ((0.8, 0.55, "not seated"), (6.8, 0.0, "seated")):
        line(ax, (x0 - 0.3, 0.3), (x0 + 4.6, 0.3), color=C_GREY, lw=1.5)
        seat = holder_side(ax, x0, 0.3, w=3.6, h=0.7)
        plate_side(ax, seat[0], seat[1], w=3.0, tilt=tilt, fc=C_POLAR)
        text(ax, x0 + 1.8, 3.3, lab, size=12, bold=True, color=C_RED if tilt else C_OK)
    mark(ax, 4.6, 1.6, 1)
    captions(ax, ("1  on a corner, on the rim, or turned: seat it flat by hand. Then Start",),
             0.5, -0.6)
    save(fig, "6_cause_seat")


def fig_cause_drift():
    fig, ax = figure(9, 4.8, (0, 12), (-1.4, 5.4))
    plate_top(ax, 1.0, 0.6, scale=0.3)
    aruco(ax, 2.25, 1.35, 1.2)
    circle(ax, 2.85, 1.95, 0.45, fc="none", ec=C_INK, lw=1.4, z=8)
    line(ax, (2.85, 1.3), (2.85, 2.6), color=C_INK, lw=1.0, z=8)
    line(ax, (2.2, 1.95), (3.5, 1.95), color=C_INK, lw=1.0, z=8)
    text(ax, 2.9, 3.9, "taught", size=10, color=C_MUTE)
    plate_top(ax, 6.6, 0.6, scale=0.3)
    aruco(ax, 7.85, 1.35, 1.2)
    circle(ax, 7.75, 2.65, 0.45, fc="none", ec=C_RED, lw=1.8, z=8)
    line(ax, (7.75, 2.0), (7.75, 3.3), color=C_RED, lw=1.2, z=8)
    line(ax, (7.1, 2.65), (8.4, 2.65), color=C_RED, lw=1.2, z=8)
    arrow(ax, (7.75, 2.65), (8.45, 1.95), lw=2.0)
    text(ax, 8.5, 3.9, "now", size=10, color=C_RED)
    mark(ax, 10.6, 2.9, 1)
    text(ax, 10.6, 1.6, "drifted", size=10, color=C_RED)
    captions(ax, ("1  something was bumped: Teach, that row, Record again. Then Start",),
             0.5, -0.6)
    save(fig, "6_cause_drift")


FIGURES = (fig_cables, fig_cameras, fig_marker_tab, fig_connect, fig_hotel_size, fig_teach_bench, fig_hover, fig_grip,
           fig_travel, fig_replay, fig_test, fig_test_run, fig_run,
           fig_fault, fig_cause_slot, fig_cause_seat, fig_cause_drift)


def make_all():
    for f in FIGURES:
        f()
    return len(FIGURES)


if __name__ == "__main__":
    print(make_all(), "drawings")
