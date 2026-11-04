"""The gripper's force close, through the controller's Lua torque probe (lua/torque_probe, published in BioyondWorks).

While a Lua program runs the SDK's arm moves return OK and do nothing (2026-11-02), so the probe runs only for the close or the read: start it,
ask, sample, stop it. The clamp is MoveExtJWithForce (probe request 23): the jaw closes toward `command` under a torque limit and stops where it
meets the plate. A plate gives a torque at the limit and a width above the command; nothing gives the command's width. The clamp survives the probe
stopping: -0.50 torque and the same width after a 30 mm lift and 3 s (measured 2026-11-02, short edges at limit 0.5; 0.5 is the highest limit the
probe accepts). Each call opens its own short connection to the controller (BRCAPI port 56788).
"""
from __future__ import annotations

import time
from typing import Dict

from devices.controller import Brcapi, BrcapiError

PORT = 56788
IDX_W, IDX_NORM, IDX_BEAT = 5, 6, 7        # the probe's system variables: width x100, normalised torque x1000, heartbeat


class ClampError(Exception):
    pass


def _up(b: Brcapi) -> None:
    if b.state().prog_state != "running":                 # still running from a session the controller cut: use it as it is
        b.start_program()
    t0 = time.time()
    while True:
        if time.time() - t0 > 10.0:
            raise ClampError("the torque probe did not start")
        time.sleep(0.25)
        if time.time() - t0 < 1.0 or not b.state().lua_ready:
            continue
        beat = b.sysvars([IDX_BEAT]).get(IDX_BEAT, 0)
        time.sleep(0.4)
        if b.sysvars([IDX_BEAT]).get(IDX_BEAT, 0) != beat:
            return


def _sample(b: Brcapi, n: int = 10, dt: float = 0.1) -> Dict[str, float]:
    rows = []
    for _ in range(n):
        time.sleep(dt)
        v = b.sysvars(range(4, 9))
        rows.append((v.get(IDX_NORM, 0) / 1000.0, v.get(IDX_W, 0) / 100.0))
    t, w = [r[0] for r in rows], [r[1] for r in rows]
    return {"torque": round(sum(t) / n, 3), "torque_min": round(min(t), 3), "torque_max": round(max(t), 3), "width": round(sum(w) / n, 2)}


def _stop(b: Brcapi) -> None:
    for _ in range(2):                                     # a cut connection: once more on a fresh one, so the probe never stays running
        try:
            b.stop_program()
            break
        except OSError:
            continue
        except Exception:
            break
    time.sleep(0.7)
    b.close()


def _session(ip: str, port: int, work) -> Dict[str, float]:
    """`work(b)` on a connection of its own, the probe stopped after. If the controller cuts the connection (it did once in about fifty
    clamps with the window's pose poller on the same port, 2026-11-03 02:28, before the port had a lock), once more from the start: the
    jaw is where it is, a second close only presses again."""
    last = None
    for attempt in (1, 2):
        b = Brcapi(ip, port, log=None)
        try:
            b.connect()
            _up(b)
            return work(b)
        except OSError as e:
            last = e
        finally:
            _stop(b)
    raise ClampError(f"the controller cut the connection twice: {last}")


def close(ip: str, command: float, limit: float, speed: int = 20, port: int = PORT) -> Dict[str, float]:
    """Close toward `command` under the torque `limit` (0..0.5); the reading once it has stopped."""
    def work(b: Brcapi):
        try:
            code = b.run_job({0: 23, 1: int(round(command * 100)), 2: int(round(limit * 1000)), 3: int(speed)}, 15.0)
        except BrcapiError as e:
            code = "no push" if e.code == "no_push" else f"error {e.code}"      # BioyondWorks connected takes the push: the request clearing is the answer
        time.sleep(0.3)
        r = _sample(b)
        r["code"] = code
        return r
    return _session(ip, port, work)


def read(ip: str, port: int = PORT) -> Dict[str, float]:
    """The holding torque and width of the jaw as it is, nothing commanded."""
    return _session(ip, port, _sample)
