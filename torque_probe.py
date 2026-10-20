"""PC side of the gripper force probe (lua/torque_probe/ running on the controller).

  python torque_probe.py watch                 print width / torque / heartbeat at 10 Hz until Ctrl-C
  python torque_probe.py close 1.45 [--trq 0.2] [--v 50]   one close command to 1.45 mm, then the vendor's
                                               GripperEmptyDet verdict and the holding torque
  python torque_probe.py open 6.0 [--v 50]
  python torque_probe.py force 1.45 --trq 0.10 [--v 20]   close to 1.45 mm under a 10 % torque limit
                                               (MoveExtJWithForce), operator at the e-stop
  python torque_probe.py limit 0.5             SetGripperTorqueLimit (torque control mode only)
  python torque_probe.py burst                 1 s of torque samples on the controller; prints the peak

Read-only except for the jaw. Never moves the arm. Needs the Lua probe loaded and running (lua_ready true).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run as R                                   # noqa: E402
from nova_cell.brcapi import Brcapi               # noqa: E402

IDX_RAW, IDX_W, IDX_NORM, IDX_BEAT, IDX_REF = 4, 5, 6, 7, 8


def read(b: Brcapi):
    v = b.sysvars(range(0, 9))
    return {"w": v.get(IDX_W, 0) / 100.0, "norm": v.get(IDX_NORM, 0) / 1000.0, "raw": v.get(IDX_RAW, 0) / 1000.0,
            "beat": v.get(IDX_BEAT, 0), "req": v.get(0, 0), "ref": v.get(IDX_REF, 0) / 100.0}


def require_probe(b: Brcapi):
    st = b.state()
    if not st.lua_ready:
        sys.exit(f"Lua program not running (lua_ready false): {st}")
    a = read(b)["beat"]
    time.sleep(0.4)
    if read(b)["beat"] == a:
        sys.exit("heartbeat not advancing: the running Lua program is not the torque probe")


def job(b: Brcapi, req: int, w_mm: float = 0.0, trq: float = 0.0, v: int = 50, timeout: float = 30.0):
    code = b.run_job({0: req, 1: int(round(w_mm * 100)), 2: int(round(trq * 1000)), 3: int(v)}, timeout)
    return code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["watch", "close", "open", "force", "limit", "burst"])
    ap.add_argument("value", nargs="?", type=float)
    ap.add_argument("--trq", type=float, default=0.2, help="GripperEmptyDet torque threshold 0..1")
    ap.add_argument("--v", type=int, default=50, help="jaw speed 1..100")
    args = ap.parse_args()
    layout = R.load_layout()
    b = Brcapi(layout["robot"]["ip"], log=None)
    b.connect()
    try:
        require_probe(b)
        if args.what == "watch":
            print("time      width  cmd_w   norm_trq  raw_trq  beat")
            while True:
                r = read(b)
                print(f"{time.strftime('%H:%M:%S')}  {r['w']:6.2f} {r['ref']:6.2f}  {r['norm']:8.3f}  "
                      f"{r['raw']:7.3f}  {r['beat']:5d}")
                time.sleep(0.1)
        before = read(b)
        if args.what == "close":
            code = job(b, 21, args.value, args.trq, args.v)
        elif args.what == "open":
            code = job(b, 22, args.value, 0.0, args.v)
        elif args.what == "force":
            print("force-limited close; hand on the e-stop")
            code = job(b, 23, args.value, args.trq, args.v)
        elif args.what == "limit":
            code = job(b, 24, 0.0, args.value, args.v)
        else:
            code = job(b, 25, timeout=10.0)
        time.sleep(0.3)
        after = read(b)
        detail = b.last_detail
        extra = ""
        if args.what == "close":
            extra = {1: "  GripperEmptyDet: HOLDING", 2: "  GripperEmptyDet: EMPTY"}.get(detail, f"  detail {detail}")
        elif args.what in ("force", "burst"):
            extra = f"  peak/holding norm torque {detail / 1000.0:.3f}" if detail is not None else ""
        print(f"{args.what}: code {code}; width {before['w']:.2f} -> {after['w']:.2f}; "
              f"norm torque {before['norm']:.3f} -> {after['norm']:.3f}; raw {after['raw']:.3f}{extra}")
    except KeyboardInterrupt:
        pass
    finally:
        b.close()


if __name__ == "__main__":
    main()
