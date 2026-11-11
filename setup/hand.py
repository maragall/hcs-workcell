"""The arm in the person's hand: loose so they can move it, locked again before a move, and the recovery after a hard
stop or a controller fault. Nothing here moves the arm."""
from __future__ import annotations


def drag(cell, on: bool, close_jaw: bool = True) -> None:
    """On: the jaw closes (on the plate or on itself) and the arm goes loose. Off: the arm locks and is enabled again."""
    if on:
        if close_jaw:
            try:
                cell.robot.jaw_to(0.0)
            except Exception:
                pass                               # the fingers met the plate: that is a closed jaw too
        cell.robot.drag(True)
        cell.arm = dict(cell.arm, drag=True)       # the readout thread catches up in half a second: do not wait for it
        cell.note("arm loose: move it by hand")
    else:
        cell.robot.drag(False)
        cell.robot.enable()
        cell.arm = dict(cell.arm, drag=False)
        cell.note("arm locked")


def loosen(cell) -> None:
    """The arm's resting state while a person works: loose, the jaw left as it is."""
    drag(cell, True, close_jaw=False)
