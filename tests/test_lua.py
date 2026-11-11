"""The program for the arm (lua/polar/mod_Polar.lua with the POLAR table of run/lua.py), run offline in the Lua that BioyondWorks
ships, against a stand-in for the controller's functions that records every command. No arm. Skipped where BioyondWorks is not installed.

What is checked: the order of the moves of a job, where it ends, what it answers, and that a refusal moves nothing.
What is not: speeds, blending, forces and the controller's own behaviour. Those are measured on the arm (tools/test_lua.py)."""
from __future__ import annotations

import ctypes
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from run import positions  # noqa: E402
from run import lua  # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.exists(lua.LUA_DLL), reason="BioyondWorks (its lua54.dll) is not installed")

STAND_IN = r"""
LOG={}
function rec(s) LOG[#LOG+1]=s end
T0=0 U0=0
SYSINVAR0=0 SYSINVAR1=0 SYSINVAR2=0 SYSINVAR3=0
ARM={x=0,y=0,z=0,rx=0,ry=0,rz=0,e1=8.2,e2=-0.192,ref1=8.2,ref2=-0.192,trq=0.4,scale=0}
PLATE=nil          -- the width the jaw stops at when it closes here, nil = nothing between the fingers
JPOSE={}           -- joints (as text) -> the wrist pose they give, from the stored routes
function RobTarget(x,y,z,rx,ry,rz,e1,e2) return {_x=x,_y=y,_z=z,_rx=rx,_ry=ry,_rz=rz,_ext1=e1,_ext2=e2} end
function JointTarget(j1,j2,j3,j4,j5,j6,e1,e2) return {_j1=j1,_j2=j2,_j3=j3,_j4=j4,_ext1=e1,_ext2=e2} end
function DefArray(a1,a2,a3,a4,a5,a6) return {_a1=a1,_a2=a2,_a3=a3,_a4=a4,_a5=a5,_a6=a6} end
function ext(e1,e2) ARM.ref1=e1 ARM.ref2=e2 ARM.e2=e2 if(PLATE==nil or e1>PLATE)then ARM.e1=e1 end end
function MoveL(p,v,zone)
  ARM.x=p._x ARM.y=p._y ARM.z=p._z ARM.rz=p._rz ext(p._ext1,p._ext2)
  rec(string.format("L %.2f %.2f %.2f %.2f|scale %d zone %d e1 %.2f e2 %.2f",p._x,p._y,p._z,p._rz,ARM.scale,zone,p._ext1,p._ext2)) return 1
end
function MoveAbsJ(j,v,zone)
  local k=string.format("%.3f %.3f %.3f %.3f",j._j1,j._j2,j._j3,j._j4)
  local p=JPOSE[k]
  if(p==nil)then error("a joint move to joints no stored route has: "..k) end
  ARM.x=p[1] ARM.y=p[2] ARM.z=p[3] ARM.rz=p[6] ext(j._ext1,j._ext2)
  rec(string.format("J %.2f %.2f %.2f %.2f|scale %d zone %d e1 %.2f e2 %.2f",p[1],p[2],p[3],p[6],ARM.scale,zone,j._ext1,j._ext2)) return 1
end
function MoveExtJ(j,v,zone) ext(j._ext1,j._ext2) ARM.trq=0.4 rec(string.format("E jaw %.2f module %.2f",j._ext1,j._ext2)) return 1 end
function MoveExtJWithForce(a,v,f)
  ARM.ref1=a._a1
  if(PLATE~=nil)then ARM.e1=PLATE ARM.trq=f._a1 else ARM.e1=a._a1 ARM.trq=0.36 end
  rec(string.format("F close %.2f limit %.2f -> %.2f",a._a1,f._a1,ARM.e1))
end
function GetCRobT() return RobTarget(ARM.x,ARM.y,ARM.z,ARM.rx,ARM.ry,ARM.rz,ARM.e1,ARM.e2) end
function GetCJoints() return {_ext1=ARM.e1,_ext2=ARM.e2,_j1=0,_j2=0,_j3=0,_j4=0} end
function GetCRefJoints() return {_ext1=ARM.ref1,_ext2=ARM.ref2,_j1=0,_j2=0,_j3=0,_j4=0} end
function GetCJointsNormTrq() return {_ext1=-ARM.trq,_ext2=0,_j1=0,_j2=0,_j3=0,_j4=0} end
function GetCJointsTrq() return GetCJointsNormTrq() end
function SetSpeedScale(s) ARM.scale=s end
function MOVFINISH() end
function DelayMS() end
function ShowInfo() end
function GetSysVarI() end
function SetSysInVarI() end
function ResetSysOutVarI() end
function SetLuaAppSta() end
function DisableGripperDet() end
function SendSysOutVarI(i,v) rec("REPLY "..i.." "..v) end
function job(a,b,c,d) LOG={} SYSINVAR0=a SYSINVAR1=b SYSINVAR2=c SYSINVAR3=d polar_Job() OUT=table.concat(LOG,"\n") end
"""


class Controller:
    def __init__(self):
        self.L = positions.load()
        routes = json.load(open(os.path.join(ROOT, "preset", "routes.json"), encoding="utf-8"))
        self.P = lua.table(self.L, routes)
        d = ctypes.CDLL(lua.LUA_DLL)
        d.luaL_newstate.restype = ctypes.c_void_p
        for f, args in (("luaL_openlibs", [ctypes.c_void_p]), ("luaL_loadstring", [ctypes.c_void_p, ctypes.c_char_p]),
                        ("lua_pcallk", [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]),
                        ("lua_getglobal", [ctypes.c_void_p, ctypes.c_char_p]), ("lua_settop", [ctypes.c_void_p, ctypes.c_int])):
            getattr(d, f).argtypes = args
        d.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        d.lua_tolstring.restype = ctypes.c_char_p
        self.d, self.s = d, d.luaL_newstate()
        d.luaL_openlibs(self.s)
        self.run(STAND_IN)
        jpose = {}
        for r in routes["routes"].values():
            for i, j in (r.get("joint_targets") or {}).items():
                jpose["%.3f %.3f %.3f %.3f" % tuple(round(float(v), 4) for v in j[:4])] = r["poses"][int(i)]
        self.run("JPOSE=" + lua._lua({f'["{k}"]': [float(v) for v in p] for k, p in jpose.items()}, 9))
        src = os.path.join(ROOT, "lua")
        for sub, name in lua.STATIC:
            self.run(open(os.path.join(src, sub, name), encoding="utf-8").read())
        self.run("POLAR=" + lua._lua(self.P))

    def run(self, text: str) -> None:
        if self.d.luaL_loadstring(self.s, text.encode("utf-8")) or self.d.lua_pcallk(self.s, 0, 0, 0, None, None):
            raise AssertionError(self.d.lua_tolstring(self.s, -1, None).decode("utf-8", "replace"))

    def at(self, pose, jaw: float, module_reading: float) -> None:
        self.run("LOG={} ARM.x=%r ARM.y=%r ARM.z=%r ARM.rz=%r ARM.e1=%r ARM.ref1=%r ARM.e2=%r ARM.ref2=%r" %
                 (pose[0], pose[1], pose[2], pose[5], jaw, jaw, module_reading, module_reading))

    def job(self, what: str, place: str = "", grip: str = "", plate=None, value: int = 0):
        """Returns (code, detail, the commands in order)."""
        a = lua.place_id(self.L, place) if place else value
        self.run(f"PLATE={'nil' if plate is None else plate} job({lua.JOBS[what]},{a},{lua.GRIPS.get(grip, 0)},0)")
        self.d.lua_getglobal(self.s, b"OUT")
        out = self.d.lua_tolstring(self.s, -1, None).decode().split("\n")
        self.d.lua_settop(self.s, 0)
        reply = [int(x.split()[2]) for x in out if x.startswith("REPLY")]
        return reply[0], reply[1], [x for x in out if not x.startswith("REPLY")]


@pytest.fixture(scope="module")
def c():
    return Controller()


def xyz(line: str):
    return [float(v) for v in line.split("|")[0].split()[1:4]]


def test_a_pick_from_a_slot_goes_in_closes_and_ends_over_the_module_with_the_plate(c):
    hover, slot = c.P["module"]["hover90"], c.P["place"][11]
    c.at(hover, 8.2, -0.192)
    code, detail, moves = c.job("pick", "A1", plate=1.46)
    assert (code, detail) == (1, 0)
    arm = [m for m in moves if m[0] in "LJ"]
    assert xyz(arm[-1]) == pytest.approx(hover[:3], abs=0.01)                       # ends where it started
    close = next(i for i, m in enumerate(moves) if m.startswith("F close"))
    assert xyz([m for m in moves[:close] if m[0] in "LJ"][-1]) == pytest.approx(slot["seat"][:3], abs=0.01)      # the close is at the seat
    assert all("e1 0.46" in m for m in moves[close + 1:] if m[0] in "LJ")           # every move after it carries the clamp command: the jaw keeps pressing
    inside = [m for m in arm if xyz(m)[0] > slot["approach"][0] + 0.01 and abs(xyz(m)[1] - slot["seat"][1]) < 1.0]
    assert inside and all(m[0] == "L" and m.split("|")[0].endswith(" 0.00") for m in inside)     # inside the hotel: straight moves only, yaw 0


def test_a_place_into_a_slot_opens_at_the_seat_and_comes_back_empty(c):
    hover, slot = c.P["module"]["hover90"], c.P["place"][25]
    c.at(hover, 0.46, -0.192)
    c.run("ARM.e1=1.46")
    code, detail, moves = c.job("place", "B5", plate=1.46)
    assert (code, detail) == (1, 0)
    opened = next(i for i, m in enumerate(moves) if m.startswith("E jaw 8.20"))
    assert xyz([m for m in moves[:opened] if m[0] in "LJ"][-1]) == pytest.approx(slot["seat"][:3], abs=0.01)
    assert xyz([m for m in moves if m[0] in "LJ"][-1]) == pytest.approx(hover[:3], abs=0.01)


def test_an_empty_slot_stops_the_arm_at_the_seat_with_the_jaw_open_again(c):
    c.at(c.P["module"]["hover90"], 8.2, -0.192)
    code, detail, moves = c.job("pick", "A3", plate=None)
    assert (code, detail) == (-2, 21)
    assert sum(m.startswith("F close") for m in moves) == 2
    assert xyz([m for m in moves if m[0] in "LJ"][-1]) == pytest.approx(c.P["place"][13]["seat"][:3], abs=0.01)      # left in place, not carried on


def test_a_job_away_from_the_module_is_refused_and_nothing_moves(c):
    c.at([150.0, 0.0, 300.0, 0, 0, 0.0], 8.2, -0.192)
    code, detail, moves = c.job("pick", "A1", plate=1.46)
    assert (code, detail) == (-1, 16) and moves == []


def test_column_c_has_no_stored_route_and_is_refused(c):
    c.at(c.P["module"]["hover90"], 8.2, -0.192)
    code, detail, moves = c.job("pick", "C1", plate=1.46)
    assert (code, detail) == (-1, 13) and moves == []


def test_a_module_pick_turns_the_module_first_when_it_stands_at_the_other_angle(c):
    seat = c.P["module"]["seat"][1]["pose"]
    c.at(c.P["module"]["hover90"], 8.2, -0.192)                                     # the module at 90, the short-edge seat needs 0
    code, detail, moves = c.job("pick", "module", "short_edges", plate=22.38)
    assert (code, detail) == (1, 0)
    turn = next(i for i, m in enumerate(moves) if m.startswith("E jaw 8.20 module 89.81"))
    assert xyz([m for m in moves[:turn] if m[0] in "LJ"][-1])[2] == pytest.approx(seat[2] + 34.4 + 60, abs=0.01)     # turned with the claw at the clear height
    assert any(m.startswith("F close 21.38") for m in moves[turn:])


def test_the_microscope_round_trip_ends_over_the_module_both_ways(c):
    hover0 = c.P["module"]["hover0"]
    c.at(hover0, 29.0, 89.808)
    code, detail, moves = c.job("pick", "microscope", plate=22.38)
    assert (code, detail) == (1, 0) and xyz([m for m in moves if m[0] in "LJ"][-1]) == pytest.approx(hover0[:3], abs=0.01)
    code, detail, moves = c.job("place", "microscope", plate=22.38)
    assert (code, detail) == (1, 0) and xyz([m for m in moves if m[0] in "LJ"][-1]) == pytest.approx(hover0[:3], abs=0.01)
    assert any(m.startswith("E jaw 28.50") for m in moves)


def test_a_plate_that_slipped_out_during_the_carry_stops_the_job(c):
    c.at(c.P["module"]["hover90"], 0.46, -0.192)                                    # the jaw at its command: nothing between the fingers
    code, detail, moves = c.job("place", "A1", plate=None)
    assert (code, detail) == (-2, 22)
    assert not any(m.startswith("E jaw") for m in moves)                            # the jaw is not opened over the slot


if __name__ == "__main__":
    k = Controller()
    k.at(k.P["module"]["hover90"], 8.2, -0.192)
    for what, place, grip, plate in (("pick", "A1", "", 1.46), ("place", "module", "long_edges", 1.46), ("turn", "", "", None)):
        code, detail, moves = k.job(what, place, grip, plate, value=0)
        print(f"\n{what} {place}: code {code} detail {detail}")
        print("\n".join("  " + m for m in moves))
