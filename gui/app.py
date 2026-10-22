"""
    python gui.py            the setup program on the real cell: Setup, Teach, Test
    python gui.py --sim      the same on the fakes (devices/fake_arm.py, tests/rig.py FakeVision), nothing real
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import webbrowser

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from devices import nova  # noqa: E402
from procedures import positions  # noqa: E402

from gui import server, sim  # noqa: E402
from gui.cell import Cell  # noqa: E402


def build(is_sim: bool) -> Cell:
    if is_sim:
        return Cell(*sim.build(), sim=True)
    L = positions.load()
    return Cell(positions.PATH, positions.REFS, positions.ROOT, positions.RUNS, nova.arm(L), nova.vision_box(L), sim=False)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="fake arm, fake vision box, fake microscope")
    ap.add_argument("--port", type=int, default=8744)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)
    cell = build(args.sim)
    srv = server.serve(cell, args.port)          # the page first: an arm that is off must not hold the start for the SDK's connect timeout
    cell.start("connecting", cell.start_up)
    url = f"http://127.0.0.1:{srv.server_address[1]}/"
    print(f"hcs-workcell {'simulator' if args.sim else 'cell'}: {url}", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        cell.stop()
        srv.shutdown()
        return 0


if __name__ == "__main__":
    sys.exit(main())
