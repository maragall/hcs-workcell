"""
The one-time setup of the arm, as a terminal program: `python cli.py setup`. It ends with READY.

    runner      Step, Section, Runner: print each line, wait for Enter on the user's, stop on the first refusal
    sections    the sections in order: connect (find the arm and the camera box), preset (write the arm's program and upload it),
                cameras (the marker plate on the rotation module, capture, solve, upload to the camera box)
    lua         the Lua writer: layout.json + run/preset/routes.json -> data.lua and mod_*.lua, parsed, uploaded, read back
    cameras     the camera calibration steps; cell: the two devices and the positions file; doctor: the read-only check of the PC
"""
