"""Terminal colours for what needs a person: their steps and questions stand out from the machine's lines. The console log strips them."""
from __future__ import annotations

import os
import re
import sys

ASK, BAD = "1;93", "1;91"            # bright yellow bold: your turn; bright red bold: stopped
_ANSI = re.compile(r"\x1b\[[0-9;]*m")
if os.name == "nt":
    os.system("")                    # switches the Windows console to understand the codes


def paint(text: str, code: str = ASK) -> str:
    return text if os.environ.get("NO_COLOR") or not sys.__stdout__ or not sys.__stdout__.isatty() else f"\x1b[{code}m{text}\x1b[0m"


def plain(text: str) -> str:
    return _ANSI.sub("", text)
