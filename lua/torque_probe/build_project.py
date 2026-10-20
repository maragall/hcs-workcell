"""Build an importable BioyondWorks project (.bin) for the gripper force probe.

Format (reverse-read from the vendor templates, 2026-10-20): a .bin is the 7 ASCII bytes "byrobot" followed by a
plain ZIP whose entries sit under one folder named like the project (`<name>/data.lua`, `<name>/main.lua`, ...).
This script starts from the vendor's V3.9 clean template, replaces main.lua with the probe's main and adds
mod_TorqueProbe.lua. Import the result in BioyondWorks on the offline ("Empty") row, then Publish.

  python lua/torque_probe/build_project.py [--template PATH] [--name TorqueProbe] [--out PATH]
"""
import argparse
import io
import os
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
MAGIC = b"byrobot"
DEFAULT_TEMPLATE = r"C:\Users\ASUS\Downloads\bioyond-arm-setup\S4资料合集\V3.9_LUA干净模版S4测试.bin"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--template", default=DEFAULT_TEMPLATE)
    ap.add_argument("--name", default="TorqueProbe", help="project name: letters and digits only (IDE rule)")
    ap.add_argument("--out", default=os.path.join(HERE, "TorqueProbe.bin"))
    args = ap.parse_args()

    raw = open(args.template, "rb").read()
    if not raw.startswith(MAGIC):
        raise SystemExit(f"{args.template}: not a byrobot project file")
    src = zipfile.ZipFile(io.BytesIO(raw[len(MAGIC):]))
    names = src.namelist()
    prefix = names[0].split("/")[0] + "/"
    print("template entries:")
    for n in names:
        print("  ", n, src.getinfo(n).file_size)

    main_lua = open(os.path.join(HERE, "main.lua"), "rb").read()
    probe_lua = open(os.path.join(HERE, "mod_TorqueProbe.lua"), "rb").read()

    # The template's own __loader.lua lists modules that are not in the archive (stale); write one that matches
    # the files actually shipped, in the template's order: system libs, data, subroutines, errhandler, main.
    mods = sorted(n[len(prefix):-4] for n in names
                  if n.endswith(".lua") and n[len(prefix):-4] not in
                  ("__sys", "__lif", "__lapp", "__loader", "data", "errhandler", "main")) + ["mod_TorqueProbe"]
    loader = 'package.path = "../user/?.lua"\n' + "".join(
        f' require("{m}")\n' for m in ["__sys", "__lif", "__lapp", "data", *mods, "errhandler", "main"])
    loader += "\nfunction __loader()\n main()\n MOVFINISH()\nend\n\nfunction __errhandler()\n PRINT(debug.traceback())\n errhandler()\nend\n"

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as dst:
        for n in names:
            base = n[len(prefix):]
            if base == "main.lua":
                data = main_lua
            elif base == "__loader.lua":
                data = loader.encode("utf-8")
            else:
                data = src.read(n)
            dst.writestr(args.name + "/" + base, data)
        dst.writestr(args.name + "/mod_TorqueProbe.lua", probe_lua)
    out = MAGIC + buf.getvalue()
    open(args.out, "wb").write(out)
    print(f"\nwrote {args.out} ({len(out)} bytes) as project '{args.name}'")
    with zipfile.ZipFile(io.BytesIO(out[len(MAGIC):])) as chk:
        for n in chk.namelist():
            print("  ", n, chk.getinfo(n).file_size)


if __name__ == "__main__":
    main()
