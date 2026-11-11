"""Where the setup's questions go. In a terminal: input(). Under gui.py (NOVA_GUI=1): one line `@@ASK <question>` on stdout, the answer is one
line on stdin, so the window can show the question with a button. Same questions, same answers, either way."""
import os
import re
import sys

ANSI = re.compile(r"\x1b\[[0-9;]*m")


def under_gui() -> bool:
    return os.environ.get("NOVA_GUI") == "1"


def emit(tag: str, payload) -> None:
    """One machine line for the window (`@@TAG {json}`), nothing in a terminal: the planned route, for the model to draw."""
    if under_gui():
        import json
        print(f"@@{tag} " + json.dumps(payload), flush=True)


def ask(prompt: str) -> str:
    if under_gui():
        print("@@ASK " + ANSI.sub("", prompt).strip(), flush=True)
        line = sys.stdin.readline()
        if line == "":
            raise EOFError
        return line.rstrip("\r\n")
    return input(prompt)
