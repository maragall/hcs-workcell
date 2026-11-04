"""docs/MANUAL.md -> docs/MANUAL.html, one self-contained file (pictures embedded) to send or print. No markdown library needed: this
reads the few constructions the manual uses (headings, paragraphs, lists, tables, pictures, bold, code). `python tools/manual_html.py`."""
from __future__ import annotations

import base64
import html
import os
import re

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC, OUT = os.path.join(HERE, "docs", "MANUAL.md"), os.path.join(HERE, "docs", "MANUAL.html")

CSS = """
body{font-family:Segoe UI,Helvetica,Arial,sans-serif;max-width:900px;margin:32px auto;padding:0 16px;color:#1d232a;line-height:1.5}
h1{font-size:28px;margin-bottom:4px} h2{font-size:22px;margin-top:36px;border-bottom:1px solid #d9dee5;padding-bottom:4px} h3{font-size:17px;margin-top:24px}
img{max-width:100%;border:1px solid #d9dee5;border-radius:4px;margin:8px 0} em{color:#4a5564}
table{border-collapse:collapse;margin:12px 0;width:100%} td,th{border:1px solid #d9dee5;padding:6px 8px;vertical-align:top;text-align:left} th{background:#f2f4f7}
code{background:#f2f4f7;padding:1px 4px;border-radius:3px;font-size:90%} pre{background:#f2f4f7;padding:10px;border-radius:4px;overflow:auto}
li{margin:3px 0} .note{color:#4a5564;font-size:13px;margin-top:40px}
"""


def inline(s: str) -> str:
    s = html.escape(s, quote=False)
    s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
    s = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", s)
    s = re.sub(r"(?<![*\w])\*([^*]+)\*(?!\w)", r"<em>\1</em>", s)
    return s


def picture(path: str) -> str:
    full = os.path.join(HERE, "docs", path)
    with open(full, "rb") as f:
        data = base64.b64encode(f.read()).decode()
    return f'<img src="data:image/png;base64,{data}" alt="{html.escape(path)}">'


def convert(md: str) -> str:
    out, i, lines = [], 0, md.splitlines()
    while i < len(lines):
        line = lines[i]
        if line.startswith("```"):
            j = i + 1
            while j < len(lines) and not lines[j].startswith("```"):
                j += 1
            out.append("<pre>" + html.escape("\n".join(lines[i + 1:j])) + "</pre>")
            i = j + 1
            continue
        m = re.match(r"^(#{1,3}) (.*)", line)
        if m:
            out.append(f"<h{len(m.group(1))}>{inline(m.group(2))}</h{len(m.group(1))}>")
            i += 1
            continue
        m = re.match(r"^!\[([^\]]*)\]\(([^)]+)\)", line)
        if m:
            out.append(picture(m.group(2)))
            i += 1
            continue
        if line.startswith("|"):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                cells = [c.strip() for c in lines[i].strip().strip("|").split("|")]
                if not all(re.fullmatch(r"-+", c) for c in cells):
                    rows.append(cells)
                i += 1
            head, body = rows[0], rows[1:]
            out.append("<table><tr>" + "".join(f"<th>{inline(c)}</th>" for c in head) + "</tr>"
                       + "".join("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in r) + "</tr>" for r in body) + "</table>")
            continue
        m = re.match(r"^(\s*)([-*]|\d+\.) (.*)", line)
        if m:
            tag = "ol" if m.group(2)[0].isdigit() else "ul"
            items = []
            while i < len(lines) and re.match(r"^(\s*)([-*]|\d+\.) (.*)", lines[i]):
                item = re.match(r"^(\s*)([-*]|\d+\.) (.*)", lines[i]).group(3)
                i += 1
                while i < len(lines) and lines[i].startswith("   ") and not re.match(r"^\s*([-*]|\d+\.) ", lines[i]):
                    item += " " + lines[i].strip()
                    i += 1
                items.append(f"<li>{inline(item)}</li>")
            out.append(f"<{tag}>" + "".join(items) + f"</{tag}>")
            continue
        if not line.strip():
            i += 1
            continue
        para = []
        while i < len(lines) and lines[i].strip() and not re.match(r"^(#|!\[|\||```|\s*[-*] |\s*\d+\. )", lines[i]):
            para.append(lines[i].strip())
            i += 1
        out.append(f"<p>{inline(' '.join(para))}</p>")
    return "\n".join(out)


def main():
    md = open(SRC, encoding="utf-8").read()
    title = re.search(r"^# (.*)", md, re.M).group(1)
    page = f"<!doctype html>\n<html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>{html.escape(title)}</title><style>{CSS}</style></head><body>\n{convert(md)}\n<p class='note'>Generated from docs/MANUAL.md by tools/manual_html.py.</p>\n</body></html>\n"
    open(OUT, "w", encoding="utf-8").write(page)
    print("wrote", OUT, round(os.path.getsize(OUT) / 1e6, 2), "MB")


if __name__ == "__main__":
    main()
