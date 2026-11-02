"""
Keeps `cli.py shuffle` running for days. When the shuffle stops with a refusal or a fault, the reason is logged and it starts again
(the shuffle's own start takes the plate it finds in the claw, on the module or on the stage); the wait before the next start grows
with each failure in a row. It gives up, and leaves the arm where it is, when the same reason stops it GIVE_UP times in a row without
a run of at least GOOD_RUN_S in between: a repeat fault is a rule to write, not something to retry all night.

    python tools/keep_running.py [--speed supervised]        stop: create runs/STOP (it ends between plates, then this ends)
"""
import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNS = os.path.join(HERE, "runs")
LOG = os.path.join(RUNS, "keep_running.log")
GIVE_UP = 4                # the same reason this many times in a row
GOOD_RUN_S = 900.0
BACKOFF_S = (10, 20, 30, 45)     # short: a stopped arm that holds a plate lost it within 5 minutes of the last command (2026-11-02, the cause is probably the controller dropping the servos); every restart is a new command


def log(msg: str) -> None:
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {msg}"
    print(line, flush=True)
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def reason_of(path: str) -> str:
    """What the run said when it stopped: the REFUSED / fault line, else the last non-empty line."""
    try:
        text = open(path, encoding="utf-8", errors="replace").read().splitlines()
    except OSError:
        return "no output"
    for line in reversed(text[-60:]):
        if "REFUSED" in line or "Traceback" in line or "Error" in line or "error" in line.split(" ")[0:3]:
            return re.sub(r"\d\d:\d\d:\d\d", "", line).strip()[:200]
    for line in reversed(text):
        if line.strip():
            return re.sub(r"\d\d:\d\d:\d\d", "", line).strip()[:200]
    return "no output"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", default="supervised")
    a = ap.parse_args(argv)
    os.makedirs(RUNS, exist_ok=True)
    stop = os.path.join(RUNS, "STOP")
    last, repeats, fails = None, 0, 0
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    log(f"keep_running starts: shuffle at {a.speed}; stop with {stop}")
    while True:
        if os.path.exists(stop):
            log("runs/STOP exists: not starting another run")
            return 0
        out = os.path.join(RUNS, f"keep_running_{time.strftime('%Y%m%d_%H%M%S')}.txt")
        t0 = time.time()
        with open(out, "w", encoding="utf-8") as f:
            code = subprocess.call([sys.executable, "-X", "utf8", "-u", "cli.py", "shuffle", "--speed", a.speed],
                                   cwd=HERE, stdout=f, stderr=subprocess.STDOUT, env=env)
        ran = time.time() - t0
        if os.path.exists(stop):
            log(f"stopped by runs/STOP after {ran / 60:.1f} min (exit {code})")
            return 0
        why = reason_of(out)
        if ran >= GOOD_RUN_S:
            repeats, fails = 0, 0
        key = re.sub(r"[\d.\-]+", "#", why)[:80]
        repeats = repeats + 1 if key == last else 1
        last, fails = key, fails + 1
        log(f"run ended after {ran / 60:.1f} min (exit {code}): {why}   [same reason {repeats}x, failures in a row {fails}; output {out}]")
        if repeats >= GIVE_UP:
            msg = f"gave up: the same reason stopped the shuffle {repeats} times in a row: {why}"
            log(msg)
            with open(os.path.join(RUNS, "SUPERVISOR_GAVE_UP.txt"), "w", encoding="utf-8") as f:
                f.write(msg + "\n")
            return 1
        wait = BACKOFF_S[min(fails, len(BACKOFF_S)) - 1]
        log(f"waiting {wait} s, then the shuffle starts again")
        for _ in range(wait):
            if os.path.exists(stop):
                break
            time.sleep(1)


if __name__ == "__main__":
    sys.exit(main())
