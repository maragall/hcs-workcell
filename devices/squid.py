"""
One Squid+ microscope, driven in-process from the Squid repo.

    home             park the stage so the arm can reach the holder
    acquire          start the acquisition that is configured on that microscope
    done()           True once the acquisition has finished
    last_dataset     path of what was just acquired

The Squid call names used here are placeholders until confirmed against the Squid repo
(see AI-docs/hcs-workcell/to-do/architecture.md, "names to confirm"). Every call raises until
then, so a real Squid cannot be driven by accident. Tests use devices.fake_squid.
"""
from __future__ import annotations

from typing import Optional


class SquidNotWired(RuntimeError):
    pass


class Squid:
    def __init__(self, name: str, config: dict, log=None):
        self.name, self.config = name, config
        self._log = log or (lambda *a: None)

    def home(self) -> None:
        raise SquidNotWired(f"{self.name}: Squid.home not wired yet (confirm the call name in the Squid repo)")

    def acquire(self) -> None:
        raise SquidNotWired(f"{self.name}: Squid.acquire not wired yet")

    def done(self) -> bool:
        raise SquidNotWired(f"{self.name}: Squid.done not wired yet")

    @property
    def last_dataset(self) -> Optional[str]:
        return None
