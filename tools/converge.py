"""
The self-converging loop: collect, retrain, judge, repeat.

Each round runs `python cli.py shuffle --moves K --probe P` (the arm moves plates between the taught slots through the microscope and,
every P moves, photographs every slot's approach with the hand empty and with a plate in it), then retrains every task in
perception/learn.py on all the frames so far and judges it on the later frames. The loop stops when every task that is not confounded
has been solved in two rounds in a row, when runs/STOP exists, when the arm run stops on a fault (it is not relaunched into one), or
after --max-rounds. Progress and the verdict go to runs/CONVERGED.md.

    python tools/converge.py [--moves 30] [--probe 10] [--max-rounds 8] [--speed supervised]
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from perception import learn  # noqa: E402

RUNS = os.path.join(HERE, "runs")
OUT = os.path.join(RUNS, "CONVERGED.md")
SINCE = "20261030_0"                    # runs from the first probe-capable shuffle on; older runs only feed the confounded tasks


def judge(results):
    return {r.task: r.solved for r in results if r.task not in learn.CONFOUNDED}


def write(rounds, results, verdict):
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(f"# Convergence ({time.strftime('%Y-%m-%d %H:%M:%S')})\n\n{verdict}\n\n")
        f.write("Tasks marked `confounded` cannot be judged from pick/place frames alone (the label equals the hand state); only the probe tours cross them.\n\n")
        f.write("```\n" + learn.report(results) + "\n```\n\n")
        f.write("| round | moves | arm run exit | solved |\n|---|---|---|---|\n")
        for r in rounds:
            f.write(f"| {r['n']} | {r['moves']} | {r['exit']} | {', '.join(k for k, v in r['solved'].items() if v) or 'none'} |\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--moves", type=int, default=30)
    ap.add_argument("--probe", type=int, default=10)
    ap.add_argument("--max-rounds", type=int, default=8)
    ap.add_argument("--speed", default="supervised", choices=("stress", "fast", "supervised"))
    a = ap.parse_args(argv)
    rounds, streak = [], 0
    results = learn.run(SINCE)
    for n in range(1, a.max_rounds + 1):
        if os.path.exists(os.path.join(RUNS, "STOP")):
            write(rounds, results, "stopped by runs/STOP")
            return 0
        print(f"round {n}: {a.moves} moves, probe tour every {a.probe}, speed {a.speed}", flush=True)
        code = subprocess.call([sys.executable, "cli.py", "shuffle", "--moves", str(a.moves), "--probe", str(a.probe), "--speed", a.speed], cwd=HERE)
        results = learn.run(SINCE)
        solved = judge(results)
        rounds.append({"n": n, "moves": a.moves, "exit": code, "solved": solved})
        print(learn.report(results), flush=True)
        if code != 0:
            write(rounds, results, f"the arm run stopped (exit {code}) in round {n}: read its log, fix, then run this again. Not relaunched into a fault.")
            return code
        streak = streak + 1 if solved and all(solved.values()) else 0
        if streak >= 2:
            write(rounds, results, f"CONVERGED: {', '.join(solved)} solved in {streak} rounds in a row.")
            return 0
        write(rounds, results, f"round {n} of {a.max_rounds}: not converged yet ({sum(solved.values())} of {len(solved)} tasks solved, streak {streak}).")
    write(rounds, results, f"not converged after {a.max_rounds} rounds: the criteria in perception/learn.py were not met.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
