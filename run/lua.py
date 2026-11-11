"""The Lua writer (used by the setup and by teaching): layout.json + run/preset/routes.json -> the program the arm runs (data.lua and mod_*.lua), and its upload.

    build(layout, routes, folder)     write the project folder; returns what went into it
    check(folder)                     the vendor's own Lua (lua54.dll of BioyondWorks) parses every file
    upload(robot, folder, keep)       back up what is on the arm, upload, read back and compare

The program (run/controller/mod_Polar.lua) is fixed text. Every number it uses is the table POLAR at the end of data.lua,
written here: the places with their seats and stored routes, the grips, the module, the speeds. The folder holds the user's
files only (data.lua, mod_*.lua); the controller's fixed entry calls mod_Socket() (proven 2026-11-09 23:30).

Place ids: 1 the rotation module, 2.. the taught instruments in the order of layout.json routes.positions, column*10 + row a slot
(A1 = 11, B5 = 25). Grip ids: 1 short edges, 2 long edges.
"""
from __future__ import annotations

import ctypes
import filecmp
import os
import shutil
import time
from typing import Dict, List

from utils.devices import nova
from run.planner.safety import Safety
from run import hotel as H
from run import plates
from run.grasp import Grasp
from run.motion import offset

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = os.path.join(HERE, "run", "controller")
STATIC = ["mod_Polar.lua", "mod_Socket.lua", "mod_TorqueProbe.lua"]
GRIPS = {"short_edges": 1, "long_edges": 2}
LUA_DLL = r"C:\Bioyond\RobStudio\lua54.dll"
JOBS = {"pick": 31, "place": 32, "turn": 33, "speed": 34}
DETAILS = {11: "unknown place or value", 12: "unknown grip", 13: "no stored route to that place", 16: "the arm is not at the module's hover",
           21: "nothing clamped in two closes", 22: "the held width left its band", 23: "the plate sits crooked in the jaw",
           24: "the claw is not square to the slot", 25: "the jaw did not open", 26: "the module did not reach its angle"}


class LuaError(Exception):
    pass


def instruments(layout: dict) -> List[str]:
    F = layout["fixtures"]
    return [n for n in layout["routes"].get("positions", []) if "." not in n and F.get(n, {}).get("pose")]


def place_id(layout: dict, name: str) -> int:
    if name == "module":
        return 1
    if name in layout["fixtures"]["hotel"]["taught"]:
        return (ord(name[0]) - ord("A") + 1) * 10 + int(name[1:])
    names = instruments(layout)
    if name in names:
        return 2 + names.index(name)
    raise LuaError(f"unknown place {name!r}")


def _num(v: float) -> str:
    return repr(round(float(v), 4)).rstrip("0").rstrip(".") if isinstance(v, float) else repr(v)


def _lua(v, depth: int = 0) -> str:
    """A Python value as a Lua constructor. Dicts open one line per key for the first two levels."""
    if v is None:
        return "nil"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return _num(v)
    if isinstance(v, (list, tuple)):
        return "{" + ",".join(_lua(x, 9) for x in v) + "}"
    pad, items = "  " * (depth + 1), []
    for k, x in v.items():
        items.append((f"[{k}]" if isinstance(k, int) else str(k)) + "=" + _lua(x, depth + 1))
    if depth < 3:
        return "{\n" + "".join(f"{pad}{i},\n" for i in items) + "  " * depth + "}"
    return "{" + ",".join(items) + "}"


def _steps(route: dict) -> List[list]:
    """A stored route as the program's steps: {1, x, y, z, rx, ry, rz} a straight move, {2, j1, j2, j3, j4} the planner's joints."""
    out = []
    for i, p in enumerate(route["poses"]):
        j = route["joint_targets"].get(str(i))
        out.append([2] + [float(v) for v in j[:4]] if j is not None else [1] + [float(v) for v in p[:6]])
    return out


def _near(a, b, tol: float = 0.05) -> bool:
    return max(abs(float(x) - float(y)) for x, y in zip(a[:6], b[:6])) <= tol


def table(layout: dict, routes: dict) -> dict:
    """POLAR, as a Python dict. Refuses a stored route that does not start and end where the layout says it should."""
    L, R = layout, routes["routes"]
    if "lua" not in L:
        raise LuaError("layout.json has no `lua` block (profile, zone_mm, v_linear, v_joint, jaw_v)")
    lua, g, F = L["lua"], L["gripper"], L["fixtures"]
    plate = plates.default(L)
    grab = float(plate.grab_height_mm)
    polar = F["polar"]
    names = list(L["speeds"])
    seat = {"short_edges": polar["pose_at_0"]["pose"], "long_edges": polar["pose_at_90"]["pose"]}
    hover = {k: offset(list(p), [0, 0, polar["hover_mm"]]) for k, p in seat.items()}
    axis = L["polar"]["ext_axis"]
    reading = {deg: axis["zero_reading"] + axis["direction"] * deg for deg in (0, 90)}
    lift = polar.get("place_lift_mm") or {}
    bad: List[str] = []

    def want(key: str, start, end) -> list | None:
        r = R.get(key)
        if not r or "poses" not in r or r.get("branch", 1) != 1:
            return None
        if not _near(r["poses"][-1], end):
            bad.append(f"{key} ends at {[round(v, 2) for v in r['poses'][-1]]}, the layout has {[round(v, 2) for v in end]}")
        begins = r.get("start", routes.get("start")) if key.startswith("module->") else r.get("entry")
        if begins is None or not _near(begins, start):
            bad.append(f"{key} starts at {begins and [round(v, 2) for v in begins]}, the layout has {[round(v, 2) for v in start]}")
        return _steps(r)

    P = {
        "profile": names.index(lua["profile"]) + 1,
        "profiles": {i + 1: [max(1, int(round(L["speeds"][n][k] * 100))) for k in ("travel", "approach", "contact")] for i, n in enumerate(names)},
        "zone": lua["zone_mm"], "v_linear": lua["v_linear"], "v_joint": lua["v_joint"], "jaw_v": lua["jaw_v"],
        "clamp_v": int(g.get("clamp_speed", 20)), "clamp_limit": float(g.get("clamp_limit", 0.5)),
        "module_v": int(round(float(axis["speed"]) * 100)), "module_settle_ms": int(round(float(L["polar"].get("settle_s", 1.0)) * 1000)),
        "slow_mm": float(L["limits"].get("slow_zone_mm", 10.0)), "crooked_mm": Grasp.CROOKED_MM,
        "grip": {i: {"open": plate.jaw_open[n], "closed": plate.jaw_closed[n], "width": g["plate_width"][n], "band": list(g["accept_band"][n])}
                 for n, i in GRIPS.items()},
        "module": {"hover": float(polar["hover_mm"]), "clear": float(polar["clear_mm"]), "hover0": hover["short_edges"], "hover90": hover["long_edges"],
                   "reading": reading,
                   "seat": {i: {"pose": offset(list(seat[n]), [0, 0, grab]), "open": (polar.get("open_width") or {}).get(n) or plate.jaw_open[n],
                                "lift": float(lift.get(n, 0.0)) if isinstance(lift, dict) else float(lift), "deg": 0 if n == "short_edges" else 90}
                            for n, i in GRIPS.items()}},
        "place": {1: {"kind": 0}},
    }
    cls = next(c for c in vars(H).values() if isinstance(c, type) and hasattr(c, "_hotel_mouth"))
    geo = cls.__new__(cls)                       # the hotel's geometry needs only the layout and the workspace
    geo.L, geo.safety = L, Safety(L, "supervised")
    for s in sorted(F["hotel"]["taught"], key=lambda s: (s[0], int(s[1:]))):
        p, approach, _, carry = geo._hotel_geometry(s)
        entry = (R.get(f"module->{s}|long_edges") or {}).get("entry") or approach
        e = {"kind": 1, "seat": offset(p, [0, 0, grab]), "approach": approach, "carry": carry, "entry": entry}
        got = {"go_empty": want(f"module->{s}|empty", hover["long_edges"], approach), "back_empty": want(f"{s}->module|empty", approach, hover["long_edges"]),
               "go_plate": want(f"module->{s}|long_edges", hover["long_edges"], entry), "back_plate": want(f"{s}->module|long_edges", entry, hover["long_edges"])}
        if all(v is not None for v in got.values()):
            e.update(got)
        P["place"][place_id(L, s)] = e
    for n in instruments(L):
        f = F[n]
        top = offset(list(f["pose"]), [0, 0, f["hover_mm"]])
        e = {"kind": 2, "seat": offset(list(f["pose"]), [0, 0, grab]), "hover": float(f["hover_mm"]),
             "open": (f.get("open_width") or {}).get("short_edges") or plate.jaw_open["short_edges"],
             "lift": float((f.get("place_lift_mm") or {}).get("short_edges", 0.0)) if isinstance(f.get("place_lift_mm"), dict) else float(f.get("place_lift_mm") or 0.0)}
        got = {"go_empty": want(f"module->{n}|empty", hover["short_edges"], top), "back_empty": want(f"{n}->module|empty", top, hover["short_edges"]),
               "go_plate": want(f"module->{n}|short_edges", hover["short_edges"], top), "back_plate": want(f"{n}->module|short_edges", top, hover["short_edges"])}
        if all(v is not None for v in got.values()):
            e.update(got)
        P["place"][place_id(L, n)] = e
    if bad:
        raise LuaError("the stored routes do not match layout.json (plan them again: python utils/plan_preset_routes.py, python cli.py teach):\n  "
                       + "\n  ".join(bad))
    return P


def build(layout: dict, routes: dict, folder: str) -> dict:
    """Write the project folder. Returns {"files": [...], "places": n, "with_routes": [names], "without_routes": [names]}."""
    P = table(layout, routes)
    os.makedirs(folder, exist_ok=True)
    for old in os.listdir(folder):
        os.remove(os.path.join(folder, old))
    for name in STATIC:
        shutil.copyfile(os.path.join(SOURCE, name), os.path.join(folder, name))
    head = open(os.path.join(SOURCE, "data_head.lua"), "rb").read().decode("utf-8").replace("\r\n", "\n").rstrip("\n")
    note = (f"-- POLAR: written by run/lua.py on {time.strftime('%Y-%m-%d %H:%M:%S')} from layout.json and run/preset/routes.json "
            f"(planned {routes.get('planned_at')}). Do not edit: run python cli.py setup.")
    text = head + "\n\n" + note + "\nPOLAR=" + _lua(P) + "\n"
    open(os.path.join(folder, "data.lua"), "wb").write(text.replace("\n", "\r\n").encode("utf-8"))
    names = {place_id(layout, n): n for n in list(layout["fixtures"]["hotel"]["taught"]) + instruments(layout)}
    have = [names[i] for i, e in P["place"].items() if "go_empty" in e]
    return {"files": sorted(os.listdir(folder)), "places": len(P["place"]), "with_routes": have,
            "without_routes": [n for i, n in names.items() if n not in have]}


def check(folder: str, dll: str = LUA_DLL) -> int:
    """Every .lua file of the folder parsed by the Lua that BioyondWorks ships. Returns the number of files; raises on the first error.
    Without the DLL (BioyondWorks not installed) raises LuaError: the files are then unchecked."""
    if not os.path.exists(dll):
        raise LuaError(f"{dll} not found: BioyondWorks is a prerequisite, its Lua checks the program before the upload")
    lua = ctypes.CDLL(dll)
    lua.luaL_newstate.restype = ctypes.c_void_p
    lua.luaL_loadstring.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lua.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    lua.lua_tolstring.restype = ctypes.c_char_p
    lua.lua_close.argtypes = [ctypes.c_void_p]
    n = 0
    for name in sorted(os.listdir(folder)):
        if not name.endswith(".lua"):
            continue
        state = lua.luaL_newstate()
        try:
            if lua.luaL_loadstring(state, open(os.path.join(folder, name), "rb").read()) != 0:
                raise LuaError(f"{name}: {lua.lua_tolstring(state, -1, None).decode('utf-8', 'replace')}")
        finally:
            lua.lua_close(state)
        n += 1
    return n


def upload(robot, folder: str, keep: str, layout: dict | None = None) -> Dict[str, object]:
    """Back up the arm's project under `keep`, upload `folder`, read it back and compare file by file. With `layout` (the real arm)
    the running program is stopped first and a job left pending in the system variables is cleared: it would run when the program starts."""
    if layout is not None:
        b = nova.controller(layout)
        b.connect()
        try:
            if b.state().prog_state == "running":
                b.stop_program()
            b.call("setSysVarI", [{"index": i, "value": 0} for i in range(4)])
        finally:
            b.close()
    stamp = time.strftime("%Y-%m-%d_%H%M%S")
    before, after = os.path.join(keep, f"arm_project_before_{stamp}"), os.path.join(keep, f"arm_project_after_{stamp}")
    os.makedirs(before)
    os.makedirs(after)
    robot.download_program(before)
    robot.upload_program(os.path.abspath(folder))
    robot.download_program(after)
    names = sorted(n for n in os.listdir(folder))
    _, differ, missing = filecmp.cmpfiles(folder, after, names, shallow=False)
    extra = sorted(set(os.listdir(after)) - set(names))
    if differ or missing or extra:
        raise LuaError(f"the arm does not hold what was sent: differ {differ}, missing {missing}, extra {extra}. The project it had is in {before}")
    return {"backup": before, "files": len(names)}
