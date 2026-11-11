"""Every method of the driver language (polar.Polar), on the arm: the moves and the refusals. The plate starts in the microscope
and ends there. Prints one line per check, PASS or FAIL with the reason, and a count at the end.

    python utils/test_api.py
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from polar import Polar, PolarError  # noqa: E402

results = []


def say(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    with open(os.path.join(HERE, "runs", "test_api.log"), "a", encoding="utf-8") as f:
        f.write(line + "\n")


def check(name, fn, want=None):
    """fn must return `want` (when given) without raising."""
    t = time.time()
    try:
        got = fn()
        ok = want is None or got == want
        results.append(ok)
        say(f"{'PASS' if ok else 'FAIL'}  {name}" + ("" if ok else f": got {got!r}, wanted {want!r}") + f"  ({time.time() - t:.0f} s)")
    except Exception as e:
        results.append(False)
        say(f"FAIL  {name}: {type(e).__name__}: {e}")
        raise


def refuses(name, fn, word):
    """fn must raise PolarError and say why; the arm must not have moved the plate."""
    try:
        fn()
    except PolarError as e:
        ok = word in str(e)
        results.append(ok)
        say(f"{'PASS' if ok else 'FAIL'}  refuses {name}: {e}")
        return
    results.append(False)
    say(f"FAIL  refuses {name}: it did not refuse")


def main() -> int:
    say("START: the plate is in the microscope")
    arm = Polar(plate_at="microscope")
    try:
        check("where() at the start", arm.where, "microscope")
        refuses("pick from an empty slot", lambda: arm.pick("A1"), "no plate at A1")
        refuses("place with an empty claw", lambda: arm.place("A1"), "holds no plate")
        refuses("an unknown place", lambda: arm.move("Z9", "A1"), "unknown place")
        refuses("an unknown destination, before anything moves", lambda: arm.move("microscope", "nowhere"), "unknown place")
        check("pick('microscope')", lambda: arm.pick("microscope"))
        check("where() with the plate in the claw", arm.where, "gripper")
        refuses("a second pick while holding", lambda: arm.pick("module"), "already holds")
        check("place('module'), short edges", lambda: arm.place("module"))
        check("where() on the module", arm.where, "module")
        check("pick('module')", lambda: arm.pick("module"))
        arm._store.s.hotel["B1"] = "another plate"                 # the ledger says B1 is taken
        refuses("place into an occupied slot", lambda: arm.place("B1"), "already holds a plate")
        arm._store.s.hotel.pop("B1")
        check("place('A3'): regrip at the module, then the slot", lambda: arm.place("A3"))
        check("where() in the slot", arm.where, "A3")
        check("move('A3', 'module')", lambda: arm.move("A3", "module"))
        check("move('module', 'B5')", lambda: arm.move("module", "B5"))
        check("move('B5', 'A1')", lambda: arm.move("B5", "A1"))
    finally:
        arm.close()
    t = time.time()
    r = subprocess.run([sys.executable, "-X", "utf8", "demo.py"], cwd=HERE, capture_output=True, text=True, errors="replace")
    ok = r.returncode == 0 and "READY" in r.stdout
    results.append(ok)
    say(f"{'PASS' if ok else 'FAIL'}  python demo.py: A1 -> microscope -> A1  ({time.time() - t:.0f} s)" + ("" if ok else f": {(r.stderr or r.stdout)[-400:]}"))
    arm = Polar()
    try:
        check("a new Polar() remembers where the plate is", arm.where, "A1")
        check("move('A1', 'microscope')", lambda: arm.move("A1", "microscope"))
        check("where() at the end", arm.where, "microscope")
    finally:
        arm.close()
    say(f"RESULT: {sum(results)} of {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        say(f"STOPPED: {type(e).__name__}: {e}. RESULT so far: {sum(results)} of {len(results)} passed. The arm stays where it is.")
        sys.exit(2)
