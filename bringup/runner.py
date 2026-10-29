"""Print each line of the setup, wait for Enter on the user's, run the machine's, stop on the first refusal."""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from bringup.colour import BAD, paint

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
                 stop: Optional[threading.Event] = None, on_section_done: Optional[Callable[[str], None]] = None):
        self.say, self.ask, self.stop = say, ask, stop or threading.Event()
        self.on_section_done = on_section_done                  # called with the section's name when every step of it ran

    def _wait(self, prompt: str) -> str:
        answer = self.ask(prompt).strip().lower()
        if answer in ("q", "quit"):
            raise Quit()
        return answer

    def run(self, sections: List[Section], start: Optional[str] = None, start_step: int = 0,
            pause_at_user: bool = False, skip: Optional[List[str]] = None,
            stop_after: Optional[int] = None) -> Optional[str]:
        """Returns None when everything ran, else the name of the section that stopped (for --from). With
        pause_at_user the run stops before each step the user does (`self.waiting` = (section, index of the next step))
        so a driver can ask the user in chat and continue with start_step."""
        self.waiting = None
        names = [s.name for s in sections]
        if start is not None and start not in names:
            raise ValueError(f"no section {start!r}; the sections are {names}")
        todo = sections[names.index(start):] if start else sections
        todo = [sec for sec in todo if sec.name not in (skip or [])]
        current = todo[0].name if todo else None
        try:
            for n, sec in enumerate(todo):
                current = sec.name
                self.say(f"\n== {sec.name}")
                first = start_step if n == 0 else 0
                if any(s.moves for s in sec.steps[first:]):
                    self.say("   (the arm moves in this section: hands assumed clear; Ctrl+C stops)")
                for i, step in enumerate(sec.steps):
                    if i < first:
                        continue
                    if self.stop.is_set():
                        self.say("   STOP")
                        return sec.name
                    self.say(paint(f"   {step.actor} : {step.text}") if step.run is None else f"   {step.actor} : {step.text}")
                    if step.run is None:
                        if step.before:
                            step.before()
                        if pause_at_user:
                            self.waiting = (sec.name, i + 1)
                            return sec.name
                        self._wait(paint("      done? Enter: "))
                        continue
                    try:
                        got = step.run()
                    except Refused as e:
                        self.say(paint(f"      REFUSED: {e}", BAD))
                        return sec.name
                    except Exception as e:
                        self.say(paint(f"      FAILED: {type(e).__name__}: {e}", BAD))
                        return sec.name
                    if got:
                        self.say("      -> " + ", ".join(f"{k} = {v}" for k, v in got.items()))
                    if n == 0 and stop_after is not None and i == stop_after:
                        self.say(f"   (stopped after step {i} of {sec.name}, as asked)")
                        return None
                if self.on_section_done:
                    self.on_section_done(sec.name)
        except (Quit, EOFError):
            self.say("   quit")
            return current
        return None
