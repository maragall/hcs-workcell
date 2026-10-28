"""Print each line of the setup, wait for Enter on the user's, run the machine's, stop on the first refusal."""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Callable, List, Optional

ACTORS = ("user", "computer", "polar", "nova", "squid")


class Refused(Exception):
    """A check failed: the section stops, its file keeps the old values, the message says what to do."""


class Quit(Exception):
    pass


@dataclass
class Step:
    actor: str
    text: str
    run: Optional[Callable[[], Optional[dict]]] = None     # None: the user does it
    moves: bool = False                                    # the arm or the stage moves
    before: Optional[Callable[[], None]] = None            # a user step: run this first (let the arm go loose)

    def __post_init__(self):
        assert self.actor in ACTORS, self.actor


@dataclass
class Section:
    name: str                                              # the file the section fills: cameras.toml ...
    steps: List[Step] = field(default_factory=list)


class Runner:
    def __init__(self, say: Callable[[str], None] = print, ask: Callable[[str], str] = input,
                 stop: Optional[threading.Event] = None):
        self.say, self.ask, self.stop = say, ask, stop or threading.Event()

    def _wait(self, prompt: str) -> str:
        answer = self.ask(prompt).strip().lower()
        if answer in ("q", "quit"):
            raise Quit()
        return answer

    def run(self, sections: List[Section], start: Optional[str] = None, start_step: int = 0,
            pause_at_user: bool = False) -> Optional[str]:
        """Returns None when everything ran, else the name of the section that stopped (for --from). With
        pause_at_user the run stops before each step the user does (`self.waiting` = (section, index of the next step))
        so a driver can ask the user in chat and continue with start_step."""
        self.waiting = None
        names = [s.name for s in sections]
        if start is not None and start not in names:
            raise ValueError(f"no section {start!r}; the sections are {names}")
        todo = sections[names.index(start):] if start else sections
        current = todo[0].name if todo else None
        try:
            for n, sec in enumerate(todo):
                current = sec.name
                self.say(f"\n== {sec.name}")
                first = start_step if n == 0 else 0
                if any(s.moves for s in sec.steps[first:]):
                    self._wait("   The arm moves in this section. Hands clear? Enter to go, q to quit: ")
                for i, step in enumerate(sec.steps):
                    if i < first:
                        continue
                    if self.stop.is_set():
                        self.say("   STOP")
                        return sec.name
                    self.say(f"   {step.actor} : {step.text}")
                    if step.run is None:
                        if step.before:
                            step.before()
                        if pause_at_user:
                            self.waiting = (sec.name, i + 1)
                            return sec.name
                        self._wait("      done? Enter: ")
                        continue
                    try:
                        got = step.run()
                    except Refused as e:
                        self.say(f"      REFUSED: {e}")
                        return sec.name
                    except Exception as e:
                        self.say(f"      FAILED: {type(e).__name__}: {e}")
                        return sec.name
                    if got:
                        self.say("      -> " + ", ".join(f"{k} = {v}" for k, v in got.items()))
        except (Quit, EOFError):
            self.say("   quit")
            return current
        return None
