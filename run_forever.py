"""
Supervisor for an unattended ping-pong run between the microscope and one hotel slot.

Launches `run.py run` and, when it ends, decides from the state file and the last stop reason whether a
relaunch is safe. Relaunches are rate limited. Anything that smells like contact, a lost plate or an
unknown arm position ends the supervision with an alert and leaves the arm where it is.

    python run_forever.py A1 --extra "--no-gates --no-vision-correction --drift-warn-only"
    stop:  create runs/STOP  (the cycle ends at the next boundary, the supervisor exits)
"""
import argparse
import json
import os
import subprocess
import sys
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")
STATE = os.path.join(HERE, "state.json")

RECOVERABLE = (
    "vision box unreachable",
    "controller lost enable",
    "did not reach",                 # SDK flag / readback mismatch with no motion; verified again on resume
    "no frame",
    "front camera frame failed",
    "connection",
    "timed out",
    "timeout",
)
FATAL = (
    "possible collision", "pose_deviation", "refused", "grasp failed", "nothing or too little",
    "obstruction", "drag mode", "disk nearly full", "not seated", "not held", "occupied", "exceeds",
    "does not exist", "no automatic resume", "jog it back",
)


def log(msg):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}"
    print(line, flush=True)
    with open(os.path.join(RUNS, "forever.log"), "a", encoding="utf-8") as f:
        f.write(line + "\n")


def last_stop_reason():
    dirs = sorted(d for d in os.listdir(RUNS) if d[:4].isdigit() and os.path.isdir(os.path.join(RUNS, d)))
    if not dirs:
        return "", ""
    path = os.path.join(RUNS, dirs[-1], "events.jsonl")
    reason, kind = "", ""
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                r = json.loads(line)
                if r["kind"] == "stop":
                    reason, kind = r.get("msg", ""), r.get("step", "")
    except OSError:
        pass
    return reason, kind


def state():
    try:
        return json.load(open(STATE, encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def relaunch_allowed(rc, reason, st):
    text = reason.lower()
    if any(k in text for k in FATAL):
        return False, "fatal reason"
    if st.get("plate_at") in ("microscope",) or str(st.get("plate_at", "")).startswith("hotel:"):
        if st.get("step_done"):
            return True, "clean boundary"
    if st.get("plate_at") == "gripper" and any(k in text for k in RECOVERABLE):
        return True, "carry paused by a recoverable condition"
    if st.get("plate_at") == "gripper" and rc == 1 and not reason:
        return True, "crash with plate held; resume verifies the arm is on its route"
    return False, "state not safe to resume without a human"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("slot", nargs="?", default="A1")
    ap.add_argument("--extra", default="--no-gates --no-vision-correction --drift-warn-only")
    ap.add_argument("--speeds", default="")
    ap.add_argument("--per-hour", type=int, default=3)
    ap.add_argument("--per-day", type=int, default=10)
    ap.add_argument("--cooldown", type=float, default=60.0)
    ap.add_argument("--first-state", choices=("microscope", "hotel", ""), default="")
    args = ap.parse_args()
    os.makedirs(RUNS, exist_ok=True)
    stop_file = os.path.join(RUNS, "STOP")
    if os.path.exists(stop_file):
        os.remove(stop_file)
    relaunches = deque()
    first = True
    while True:
        st = state()
        cmd = [sys.executable, os.path.join(HERE, "run.py"), "run", "--mode", "supervised", "--slot", args.slot,
               "--cycles", "1000000", "--allow-no-tty", "--yes", "--record", "--record-fps", "1"] + args.extra.split()
        if args.speeds:
            cmd += ["--speeds", args.speeds]
        if first and args.first_state:
            cmd += ["--set-state", args.first_state]
        elif not st.get("step_done", True):
            cmd += ["--resume"]
        first = False
        log(f"launch: {' '.join(cmd[2:])}")
        rc = subprocess.call(cmd, cwd=HERE, env={**os.environ, "PYTHONIOENCODING": "utf-8"})
        reason, where = last_stop_reason()
        st = state()
        log(f"run ended rc={rc} plate_at={st.get('plate_at')} step={st.get('step')} done={st.get('step_done')} "
            f"reason={reason[:120]!r}")
        if os.path.exists(stop_file):
            log("STOP file present; supervisor exits")
            return 0
        if rc == 0 and st.get("step_done"):
            log("run finished its cycle budget; exiting")
            return 0
        ok, why = relaunch_allowed(rc, reason, st)
        now = time.time()
        while relaunches and now - relaunches[0] > 86400:
            relaunches.popleft()
        hour = sum(1 for t in relaunches if now - t < 3600)
        if not ok:
            log(f"NOT relaunching: {why}. Arm left in place. Operator needed.")
            with open(os.path.join(RUNS, "ALERT_SUPERVISOR.txt"), "a", encoding="utf-8") as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} stopped: {reason} | {why}\n")
            return 2
        if hour >= args.per_hour or len(relaunches) >= args.per_day:
            log(f"relaunch budget exhausted ({hour}/h, {len(relaunches)}/day). Operator needed.")
            return 3
        relaunches.append(now)
        log(f"relaunch in {args.cooldown:.0f} s: {why}")
        time.sleep(args.cooldown)


if __name__ == "__main__":
    sys.exit(main())
