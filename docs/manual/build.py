"""Build the operator manual: `python3 docs/manual/build.py` redraws every PNG, then writes hcs-workcell-manual.pdf.

One numbered step per page or half page: a heading, one or two sentences, one drawing. A4, large type, white."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import draw  # noqa: E402
from drawlib import PNG  # noqa: E402
from reportlab.lib.pagesizes import A4  # noqa: E402
from reportlab.lib.utils import ImageReader  # noqa: E402
from reportlab.pdfgen import canvas  # noqa: E402

PDF = os.path.join(HERE, "hcs-workcell-manual.pdf")
INK, MUTE, DARK, RED = "#1f2933", "#5c6673", "#1F3F5F", "#b23a3a"
W, H = A4
MARGIN = 54
BODY = 14.5
LEAD = 21

# (number, title, sentences, drawing, size) ; size "full" takes a page, "half" shares one with the next half
STEPS = (
    ("0", "Cables",
     ["Plug in the three cables.", "Switch the arm on. Key at Auto."], "0_cables", "full"),
    ("1", "Cameras",
     ["Screw the marker tab from the kit onto the practice plate."], "1_marker_tab", "half"),
    ("1.1", "Does it read the marker?",
     ["Put the practice plate with the marker under the camera. The screen says it reads the marker.",
      "If not: clean the lens, or move the plate closer."], "1_reads_marker", "half"),
    ("1.2", "Capture plate",
     ["Put the dotted calibration plate from the kit on the bench. Drag the arm so the plate fills the frame on screen. Press Capture plate.",
      "The arm moves around it by itself. Then a number and ok, or redo."],
     "1_capture_plate", "half"),
    ("1.3", "Capture hand",
     ["Keep the plate still. Press Capture hand: the same again, for how the camera sits on the hand."],
     "1_capture_hand", "half"),
    ("1.4", "Upload, Check",
     ["Press Upload: the box restarts. Then press Check: a number and ok."], "1_upload_check", "half"),
    ("1.5", "Add, Add, Link",
     ["Grip the practice plate with the marker, put it down flat, press Add. Open the fingers, hover a hand's width above it, press Add. Press Link.",
      "Once with the short edges in the fingers, once with the long edges."], "1_add_link", "half"),
    ("2", "Setup",
     ["Open the setup program. Cameras, Setup, Teach and Test live in it; run is the other program.",
      "It finds the arm and the vision box by itself. Nothing to type. Every row green: go on."], "2_connect", "half"),
    ("2.1", "The hotel",
     ["Press the hotel you have: 15 slots or 30 slots."], "2_hotel_size", "half"),
    ("3", "Teach",
     ["One list. Put the practice plate where the row says, hover the hand over it so the marker is in view, press Record.",
      "Go down the list. The way you drag the arm between two Records is the route it will use."],
     "3_teach_bench", "full"),
    ("3.1", "Hover",
     ["A hand's width above the plate. The marker whole in the down camera's picture."], "3_hover", "half"),
    ("3.2", "Grip",
     ["Plate between the fingers. Press Measure with the short edges in the fingers, then again with the long edges."],
     "3_grip", "half"),
    ("3.3", "Travel height",
     ["Lift the hand until it clears everything on the bench. Press Record."], "3_travel", "half"),
    ("3.4", "Replay slowly",
     ["Press Replay slowly. The arm repeats every route at low speed while you watch.",
      "Done when no row in the list is red."], "3_replay", "half"),
    ("4", "Test",
     ["Click the slots that really hold a plate. Press Test everything.",
      "Watch. STOP at any time. A red ring on a slot: go and look at it."], "4_test", "full"),
    ("4.1", "What the test does",
     ["The arm does all of it on its own, slowly, in this order.",
      "Done when every plate is back in its own slot and the screen says finished."], "4_test_run", "full"),
    ("5", "Run",
     ["When the tests pass, open the run program and press Start."], "5_run", "full"),
    ("6", "If something stops",
     ["The arm holds still. Read the one line on screen. It is one of the three below."], "6_fault", "half"),
    ("6.1", "A slot that looks taken",
     ["Take the plate out of that slot, or click the slot in Test. Then Start."], "6_cause_slot", "half"),
    ("6.2", "A plate that does not look seated",
     ["Seat the plate by hand, flat in the recess. Then Start."], "6_cause_seat", "half"),
    ("6.3", "A position that drifted",
     ["Something was bumped. Teach: find that row, Record again. Then Start."], "6_cause_drift", "half"),
)

CONTENTS = (("0", "Cables"), ("1", "Cameras"), ("2", "Setup"), ("3", "Teach"), ("4", "Test"),
            ("5", "Run"), ("6", "If something stops"))


def wrap(c, s, font, size, width):
    words, lines, cur = s.split(), [], ""
    for w in words:
        trial = (cur + " " + w).strip()
        if c.stringWidth(trial, font, size) <= width:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines


def heading(c, x, y, number, title):
    """Step number in a dark circle, the title beside it. Returns the baseline below."""
    r = 15
    c.setFillColor(DARK)
    c.circle(x + r, y - r + 5, r, stroke=0, fill=1)
    c.setFillColor("white")
    c.setFont("Helvetica-Bold", 11 if len(number) > 1 else 14)
    c.drawCentredString(x + r, y - r + 1, number)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 24)
    c.drawString(x + 2 * r + 12, y - 15, title)
    return y - 44


def sentences(c, x, y, lines, width):
    c.setFont("Helvetica", BODY)
    c.setFillColor(INK)
    for s in lines:
        for ln in wrap(c, s, "Helvetica", BODY, width):
            c.drawString(x, y, ln)
            y -= LEAD
    return y


def picture(c, name, x, top, width, height):
    """Fit the drawing in the box, keep its shape, hang it from the top."""
    img = ImageReader(os.path.join(PNG, name + ".png"))
    iw, ih = img.getSize()
    scale = min(width / iw, height / ih)
    w, h = iw * scale, ih * scale
    c.drawImage(img, x + (width - w) / 2, top - h, w, h)


def step(c, number, title, lines, name, top, bottom):
    x, width = MARGIN, W - 2 * MARGIN
    y = heading(c, x, top, number, title)
    y = sentences(c, x, y - 8, lines, width)
    picture(c, name, x, y - 14, width, y - 14 - bottom)


def footer(c, page, section):
    c.setFont("Helvetica", 9)
    c.setFillColor(MUTE)
    c.drawString(MARGIN, 30, "hcs-workcell  " + section)
    c.drawRightString(W - MARGIN, 30, str(page))


def cover(c):
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 40)
    c.drawString(MARGIN, H - 190, "hcs-workcell")
    c.setFont("Helvetica", 18)
    c.drawString(MARGIN, H - 225, "Setting up the cell and running it")
    c.setFont("Helvetica", 13)
    c.setFillColor(MUTE)
    c.drawString(MARGIN, H - 262, "Two programs. setup: Cameras, Setup, Teach, Test. run: Start, STOP.")
    y = H - 330
    for num, title in CONTENTS:
        c.setFillColor(DARK)
        c.circle(MARGIN + 13, y + 4, 13, stroke=0, fill=1)
        c.setFillColor("white")
        c.setFont("Helvetica-Bold", 13)
        c.drawCentredString(MARGIN + 13, y, num)
        c.setFillColor(INK)
        c.setFont("Helvetica", 18)
        c.drawString(MARGIN + 40, y, title)
        y -= 40
    c.setFillColor(RED)
    c.setFont("Helvetica-Bold", 13)
    c.drawString(MARGIN, 110, "STOP, on screen, at any time: the arm holds still.")
    c.showPage()


def pages():
    """Group the steps: a full step is a page, two halves share one."""
    out, pending = [], []
    for s in STEPS:
        if s[4] == "full":
            if pending:
                out.append(pending)
                pending = []
            out.append([s])
        else:
            pending.append(s)
            if len(pending) == 2:
                out.append(pending)
                pending = []
    if pending:
        out.append(pending)
    return out


def build():
    draw.make_all()
    c = canvas.Canvas(PDF, pagesize=A4)
    c.setTitle("hcs-workcell manual")
    c.setAuthor("Cephla")
    cover(c)
    top, bottom = H - MARGIN - 10, 60
    for n, group in enumerate(pages(), start=1):
        if len(group) == 1:
            step(c, *group[0][:4], top, bottom)
        else:
            mid = (top + bottom) / 2
            step(c, *group[0][:4], top, mid + 18)
            c.setStrokeColor("#e4e7eb")
            c.line(MARGIN, mid, W - MARGIN, mid)
            step(c, *group[1][:4], mid - 18, bottom)
        footer(c, n, group[0][1] if group[0][0].isdigit() else next(
            t for k, t in CONTENTS if k == group[0][0].split(".")[0]))
        c.showPage()
    c.save()
    return PDF


if __name__ == "__main__":
    print(build())
