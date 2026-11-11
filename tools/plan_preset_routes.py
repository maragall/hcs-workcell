"""Plan the preset routes offline: rotation module seat (long edges) -> every hotel slot's entry point and back, with a plate and empty.
Keys "module->A1|long_edges" and "A1->module|long_edges" (and "|empty"). The way back starts with the joints the way there ends with.
Reads layout.json, writes preset/routes.json. No hardware.   python tools/plan_preset_routes.py"""
import json, os, sys, time
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE); os.chdir(HERE)
from run.safety import Safety, SafetyViolation  # noqa: E402
from run.motion import offset  # noqa: E402
from run import hotel as H  # noqa: E402
from planner import plan as P  # noqa: E402
from planner.volumes import Volumes  # noqa: E402
from planner import fk  # noqa: E402

L = json.load(open("layout.json", encoding="utf-8"))
polar = L["fixtures"]["polar"]
start = offset(list(polar["pose_at_90"]["pose"]), [0, 0, polar["hover_mm"]])
J = {1: fk.ik(start, elbow=1), -1: fk.ik(start, elbow=-1)}       # the module hover on both elbow sides; column C is only reachable on -1
SAFETY = {b: Safety(L, "supervised", lambda b=b: list(J[b])) for b in J}
cls = next(c for c in vars(H).values() if isinstance(c, type) and hasattr(c, "_hotel_mouth"))
g = cls.__new__(cls); g.L = L; g.safety = SAFETY[1]              # the hotel geometry needs only the layout and the safety
out = {"planned_at": time.strftime("%Y-%m-%d %H:%M:%S"), "layout_sig": P.layout_sig(L), "start": start, "start_joints": J[1], "routes": {}}

# the elbow switch at the module, empty claw: one joint move between the two joint vectors of the same wrist pose, tried at the hover
# and higher. Both elbows are checked at every sample (a Safety without joints), since the move crosses from one to the other.
attempts, at = [], None
for z in [start[2]] + [float(v) for v in range(120, int(L["workspace_box"]["z"][1]), 40)] + [float(L["workspace_box"]["z"][1])]:
    a, b = ([z - fk.MODEL["z_offset"]] + J[e][1:] for e in (1, -1))
    res = {}
    for name, ja, jb in (("to_mirror", a, b), ("to_standard", b, a)):
        try:
            Volumes(L, Safety(L, "supervised"), None).check_joint_leg(fk.fk(ja), fk.fk(jb), "switch", joints_a=ja, joints_b=jb); res[name] = "ok"
        except SafetyViolation as e:
            res[name] = str(e)
    attempts.append({"z": z, **res})
    at = z if at is None and set(res.values()) == {"ok"} else at
    print("switch at z", z, res, flush=True)
out["module_switch"] = {"ok": at is not None, "holding": "empty", "pose": start[:2] + [at if at is not None else start[2]] + start[3:], "z": at,
                        "joints": {"1": J[1], "-1": J[-1]}, "attempts": attempts}
bad = 0
for holding in ("long_edges", None):
    for s in sorted(L["fixtures"]["hotel"]["taught"], key=lambda s: (s[0], int(s[1:]))):
        key, branch = f"module->{s}|{holding or 'empty'}", -1 if s[0] == "C" else 1
        try:
            approach = g._hotel_geometry(s)[1]
            mouth = g._hotel_mouth(approach, holding)
            r = P.plan(L, SAFETY[branch], list(start), list(mouth), "polar.pose_at_90", f"mouth.{s}", holding=holding)
            out["routes"][key] = {"branch": branch, "entry": mouth, "approach": approach, "poses": r.poses, "joint_targets": {str(k): v for k, v in r.joint_targets.items()}}
            print(key, "ok", len(r.poses), "legs, branch", branch, flush=True)
        except Exception as e:
            bad += 1; out["routes"][key] = {"branch": branch, "error": f"{type(e).__name__}: {e}"}
            print(key, "FAILED", str(e)[:160], flush=True)
            r = None
        key = f"{s}->module|{holding or 'empty'}"                    # the way back: from the entry point, on the joints the way there ends with
        try:
            if r is None:
                raise SafetyViolation("the route to this entry point was not planned, so the joints it ends with are unknown")
            at = P.end_joints(r, list(start), J[branch], Volumes(L, SAFETY[branch], holding).lim)
            back = P.plan(L, Safety(L, "supervised", lambda at=at: list(at)), list(mouth), list(start), f"mouth.{s}", "polar.pose_at_90", holding=holding)
            out["routes"][key] = {"branch": branch, "entry": mouth, "approach": approach, "start_joints": at, "poses": back.poses,
                                  "joint_targets": {str(k): v for k, v in back.joint_targets.items()}}
            print(key, "ok", len(back.poses), "legs, branch", branch, flush=True)
        except Exception as e:
            bad += 1; out["routes"][key] = {"branch": branch, "error": f"{type(e).__name__}: {e}"}
            print(key, "FAILED", str(e)[:160], flush=True)
os.makedirs("preset", exist_ok=True)
json.dump(out, open("preset/routes.json", "w", encoding="utf-8"), indent=1)
print(f"{len(out['routes']) - bad} of {len(out['routes'])} planned -> preset/routes.json")
