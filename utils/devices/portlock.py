"""One client at a time on the controller's BRCAPI port (measured 2026-11-03: a second connection is refused, and now and then the controller
cuts the first one instead, WinError 10054 inside a force clamp at 02:28). Everything on this PC that opens that port takes this lock first,
so the window's pose poller, the setup's readings and the force clamp take turns instead of racing. Windows: a named mutex, which the system
releases if its holder dies. Elsewhere: flock on a file in the temp folder."""
from __future__ import annotations

import os
import sys
import tempfile
import time


class PortLock:
    def __init__(self, host: str, port: int):
        self.name = f"brcapi-{host}-{port}".replace(":", "_")
        self._h = None
        self._f = None

    def acquire(self, timeout: float) -> bool:
        if sys.platform == "win32":
            return self._acquire_win(timeout)
        return self._acquire_file(timeout)

    def release(self) -> None:
        if sys.platform == "win32":
            if self._h is not None:
                import ctypes
                ctypes.windll.kernel32.ReleaseMutex(self._h)
                ctypes.windll.kernel32.CloseHandle(self._h)
                self._h = None
            return
        if self._f is not None:
            import fcntl
            fcntl.flock(self._f, fcntl.LOCK_UN)
            self._f.close()
            self._f = None

    # Windows: a per-session named mutex. WAIT_ABANDONED (0x80) means the last holder died holding it: ours now.
    def _acquire_win(self, timeout: float) -> bool:
        import ctypes
        k = ctypes.windll.kernel32
        h = k.CreateMutexW(None, False, "Local\\" + self.name)
        if not h:
            return True                                                   # no mutex to be had: do not block the work on it
        r = k.WaitForSingleObject(h, int(max(0.0, timeout) * 1000))
        if r in (0x0, 0x80):
            self._h = h
            return True
        k.CloseHandle(h)
        return False

    def _acquire_file(self, timeout: float) -> bool:
        import fcntl
        f = open(os.path.join(tempfile.gettempdir(), self.name + ".lock"), "a+")
        deadline = time.time() + timeout
        while True:
            try:
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self._f = f
                return True
            except OSError:
                if time.time() >= deadline:
                    f.close()
                    return False
                time.sleep(0.02)
