"""The program on the arm, one job at a time, from the PC. For the first runs: a person at the emergency stop.

    python utils/test_lua.py upload                        write, parse, back up, upload, read back, heartbeat. Nothing moves.
    python utils/test_lua.py speed travel 30               the arm goes 30 mm straight up and back at a speed class; prints the seconds
    python utils/test_lua.py turn 0                        the rotation module to 0 (or 90), the claw lifted clear first
    python utils/test_lua.py pick module --grip short_edges
    python utils/test_lua.py place module --grip short_edges
    python utils/test_lua.py pick A1                       from the module's hover to the slot, in, close, out, back to the hover
    python utils/test_lua.py place A1
    python utils/test_lua.py pick microscope               (an instrument taught with python cli.py teach)
      --profile supervised|fast|stress|overdrive           a speeds entry of layout.json; default: layout.json lua.profile

Every job starts with the arm at the module's hover and ends there. A refusal (code -1) moved nothing; a stop (code -2)
left the arm where it stopped. The program is started for the job and stopped after it, so the PC-driven tools still work."""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
os.chdir(HERE)
from utils.devices import clamp, nova  # noqa: E402
from utils.devices.controller import BrcapiError  # noqa: E402
from run import positions  # noqa: E402
from run import lua  # noqa: E402

CLASSES = {"travel": 1, "approach": 2, "contact": 3}


def say(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    with open(os.path.join(positions.RUNS, "test_lua.log"), "a", encoding="utf-8") as f:
        f.write(line + "\n")


def upload(L):
    folder = os.path.join(positions.RUNS, "lua_project")
    got = lua.build(L, json.load(open(os.path.join(HERE, "run", "preset", "routes.json"), encoding="utf-8")), folder)
    say(f"written: {got['places']} places, routes for {', '.join(got['with_routes'])}; none for {', '.join(got['without_routes']) or 'nothing'}")
    say(f"parsed by the vendor's Lua: {lua.check(folder)} files")
    robot, _ = nova.connect_readonly(L, lambda *a, **k: None)
    try:
        r = lua.upload(robot, folder, positions.RUNS, L)
    finally:
        robot.disconnect()
    say(f"uploaded and read back: {r['files']} files; what the arm had is in {r['backup']}")
    r = clamp.read(L["robot"]["ip"])
    say(f"the program runs: heartbeat seen, jaw {r['width']}, torque {r['torque']}; stopped again")


def job(L, code, a=0, b=0, profile=0, timeout=240.0):
    robot, st = nova.connect_readonly(L, lambda *a, **k: None)
    try:
        if st.drag:
            robot.drag(False)
        if not robot.enable():
            sys.exit("the arm could not be enabled")
        say(f"arm at {[round(v, 1) for v in robot.pose()]}, jaw {robot.jaw():.2f}")
    finally:
        robot.disconnect()
    c = nova.controller(L)
    c.connect()
    try:
        clamp._up(c)
        t = time.time()
        try:
            got = c.run_job({0: code, 1: a, 2: b, 3: profile}, timeout)
        except BrcapiError as e:
            say(f"no answer from the program: {e}")
            return 2
        dt = time.time() - t
        detail = c.last_detail
        words = {1: "done", -1: "REFUSED, nothing moved", -2: "STOPPED part way, the arm is where it stopped"}.get(got, f"code {got}")
        say(f"{words}{': ' + lua.DETAILS.get(detail, str(detail)) if detail else ''}  ({dt:.1f} s)")
        return 0 if got == 1 else 1
    finally:
        clamp._stop(c)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["upload", "speed", "turn", "pick", "place"])
    ap.add_argument("a", nargs="?")
    ap.add_argument("b", nargs="?")
    ap.add_argument("--grip", choices=list(lua.GRIPS))
    ap.add_argument("--profile")
    args = ap.parse_args()
    L = positions.load()
    profile = list(L["speeds"]).index(args.profile) + 1 if args.profile else 0
    say(f"START {args.what} {args.a or ''} {args.b or ''} {args.grip or ''} profile {args.profile or L['lua']['profile']}")
    if args.what == "upload":
        upload(L)
        return 0
    if args.what == "speed":
        return job(L, lua.JOBS["speed"], CLASSES[args.a], int(args.b), profile, 60.0)
    if args.what == "turn":
        return job(L, lua.JOBS["turn"], int(args.a), 0, profile, 60.0)
    place = lua.place_id(L, args.a)
    if args.a == "module" and not args.grip:
        sys.exit("the module needs --grip short_edges or --grip long_edges")
    return job(L, lua.JOBS[args.what], place, lua.GRIPS.get(args.grip, 0), profile)


if __name__ == "__main__":
    sys.exit(main())
