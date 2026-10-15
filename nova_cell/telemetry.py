"""
Structured logging: one JSONL file per run, one folder of images per cycle.
This is the training data for levels 2 and 3, so every step is recorded with the same keys.
"""
from __future__ import annotations

import json
import os
import re
import time
from typing import Optional


class Telemetry:
    def __init__(self, run_dir: str):
        self.run_dir = run_dir
        os.makedirs(run_dir, exist_ok=True)
        self.path = os.path.join(run_dir, "events.jsonl")
        self._f = open(self.path, "a", encoding="utf-8")
        self.cycle = 0
        self.t0 = time.time()

    def log(self, kind: str, step: str, msg: str = "", **data):
        rec = {"t": round(time.time(), 3), "cycle": self.cycle, "kind": kind, "step": step, "msg": msg}
        rec.update(data)
        self._f.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
        self._f.flush()
        stamp = time.strftime("%H:%M:%S")
        extra = " ".join(f"{k}={v}" for k, v in data.items() if k in ("dt", "jaw", "mag", "score", "state"))
        print(f"{stamp} c{self.cycle:04d} {kind:8s} {step:28s} {msg} {extra}", flush=True)

    def image(self, name: str, data: Optional[bytes]) -> str:
        if not data:
            return ""
        d = os.path.join(self.run_dir, "images", f"c{self.cycle:04d}")
        os.makedirs(d, exist_ok=True)
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", name)
        p = os.path.join(d, f"{int((time.time() - self.t0) * 1000):08d}_{safe}.jpg")
        with open(p, "wb") as f:
            f.write(data)
        return p

    def close(self):
        self._f.close()
