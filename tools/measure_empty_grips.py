"""Empty-jaw force close, both grips, in free air: the contrast to the on-plate closes (runs/force_close_practice_*.json).
The arm must be locked, the jaw empty, the hand clear of everything (it does not move the arm, only the jaw).
    python tools/measure_empty_grips.py        -> runs/force_close_practice_empty.json"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from run import positions  # noqa: E402
from devices import nova  # noqa: E402
from devices import clamp, nova  # noqa: E402

COMMAND = {"long_edges": 0.5, "short_edges": 21.4}     # the same commands as the on-plate closes
LIMIT, SPEED = 0.5, 20


def main() -> int:
    L = positions.load()
    ip = L["robot"]["ip"]
    b = nova.controller(L)
    b.connect()
    s = b.state()
    if s.drag or not s.enabled or s.errors:
        b.close()
        sys.exit(f"the arm must be locked, enabled and free of errors: {s}")
    if s.ctrl_mode != "auto":
        b.switch_auto(); time.sleep(1.0); b.enable(); time.sleep(1.0)
    b.call("setSysVarI", [{"index": i, "value": 0} for i in range(0, 4)])     # no job may be pending when the program starts
    b.close()
    out = {"at": time.strftime("%Y-%m-%d %H:%M:%S"), "limit": LIMIT, "speed": SPEED}
    for grip in ("long_edges", "short_edges"):
        robot, _ = nova.connect_readonly(L, lambda *a, **k: None)
        try:
            robot.jaw_to(L["gripper"]["open"][grip])
            opened = round(robot.jaw(), 2)
        finally:
            robot.disconnect()
        r = clamp.close(ip, COMMAND[grip], LIMIT, SPEED)
        out[grip] = {"opened": opened, "command": COMMAND[grip], "close": r}
        print(f"{grip}: opened {opened}, command {COMMAND[grip]} -> width {r['width']}, torque {r['torque']}", flush=True)
    with open(os.path.join(positions.RUNS, "force_close_practice_empty.json"), "w") as f:
        json.dump(out, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
