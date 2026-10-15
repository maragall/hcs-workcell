"""
Run the cycle over a list of hotel slots, one `run.py run` per slot, stopping at the first stop or crash.

    python run_matrix.py A3 A5 A7 A9 B1 B3 --extra "--no-gates --no-vision-correction"

Each run starts from the state the previous one left (plate on the microscope).
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("slots", nargs="+")
    ap.add_argument("--mode", default="supervised")
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--extra", default="", help="extra flags passed to run.py run")
    ap.add_argument("--pause", type=float, default=5.0, help="seconds between slots")
    args = ap.parse_args()
    log = open(os.path.join(HERE, "runs", "matrix.log"), "a", encoding="utf-8")
    for i, slot in enumerate(args.slots):
        cmd = [sys.executable, os.path.join(HERE, "run.py"), "run", "--mode", args.mode, "--slot", slot,
               "--cycles", str(args.cycles), "--allow-no-tty", "--yes", "--record"] + args.extra.split()
        if i == 0:
            cmd += ["--set-state", "microscope"]
        line = f"{time.strftime('%H:%M:%S')} slot {slot}: {' '.join(cmd[2:])}"
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()
        rc = subprocess.call(cmd, cwd=HERE, env={**os.environ, "PYTHONIOENCODING": "utf-8"})
        log.write(f"{time.strftime('%H:%M:%S')} slot {slot}: exit {rc}\n")
        log.flush()
        if rc != 0:
            print(f"slot {slot} ended with exit {rc}; matrix stopped", flush=True)
            return rc
        time.sleep(args.pause)
    print("matrix complete", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
