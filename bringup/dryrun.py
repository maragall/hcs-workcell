"""
The collision oracle: a simulated arm in the REAL frame that checks every motion it is commanded against the CAD volumes
(layout `volumes`, routes/volumes.py) before the real arm is asked to make it.

`OracleRobot` is a SimRobot with real kinematics (routes_opt/fk): a linear move is sampled in x, y, z and yaw; a joint move
(cartesian target, joint-interpolated), an absolute-joint move and each leg of a taught-route replay are interpolated
in joint space, and every sample is tested with the claw, the plate in it (the cycle ledger says which grip holds one)
and the two links. Contact is a violation with no margin; a violation only inside the planner's margin is a warning.

Inside the hotel the volume is a solid box, so a slide into a slot would always collide. A slot has a corridor instead:
wrist y and yaw at the slot's, x anywhere in front of the seat, z between the seat and the seat plus the lift. Inside a
corridor the hotel is not a collision; instead the plate and the fingers have to fit the gap between two trays
(`slot_window`), the number `python cli.py` cannot know and the CAD can (bringup/rackcheck.py measures the fingers).

    from bringup.dryrun import OracleRobot
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from devices.fake_arm import SimRobot
from procedures.cad_poses import CadPoses
from procedures.safety import Safety, SafetyViolation
from routes.volumes import Volumes
from routes_opt import fk

STEP_MM = 4.0
STEP_DEG = 2.0
CORRIDOR_Y_MM, CORRIDOR_YAW_DEG, CORRIDOR_Z_SLACK_MM = 3.0, 3.0, 2.0
TRAY_PITCH_MM, TRAY_THICKNESS_MM = 38.0, 8.0          # the CAD: shelves 8 mm thick every 38 mm


class Collision(Exception):
    pass


@dataclass
class Event:
    kind: str                # "collision" | "margin" | "slot"
    label: str
    pose: List[float]
    note: str


@dataclass
class Report:
    events: List[Event] = field(default_factory=list)

    @property
    def collisions(self) -> List[Event]:
        return [e for e in self.events if e.kind == "collision"]

    @property
    def margins(self) -> List[Event]:
        return [e for e in self.events if e.kind == "margin"]

    def lines(self, kinds=("collision",)) -> List[str]:
        return [f"{e.kind}: {e.label}: {e.note}" for e in self.events if e.kind in kinds]


def slot_window(plate_height_mm: float, lid: bool = False, lid_mm: float = 2.43, clearance_mm: float = 1.0):
    """Wrist height above the seat that the plate in the claw may have inside a slot: not below the tray, and not so high that
    the plate's top meets the tray above (30 mm of gap between two trays 8 mm thick on a 38 mm pitch)."""
    gap = TRAY_PITCH_MM - TRAY_THICKNESS_MM
    top = plate_height_mm + (lid_mm if lid else 0.0)
    return -1.0, gap - top - clearance_mm


class OracleRobot(SimRobot):
    def __init__(self, layout: dict, start_pose: List[float], grip: Callable[[], Optional[str]] = lambda: None,
                 raise_on_collision: bool = True, log=None, jaw: float = 10.0, lid: bool = False):
        super().__init__(start_pose, None, log, jaw)
        self.L, self.grip, self.raise_on_collision, self.lid = layout, grip, raise_on_collision, lid
        self.report = Report()
        self._joints = list(fk.ik(self._pose, elbow=1)) + [0.0] * 4
        self._safety = Safety(layout, "supervised", self.joints)
        self._cad = CadPoses(layout)
        h = layout["fixtures"]["hotel"]
        self._slots = [(f"{c}{r}", self._cad.hotel_pose(f"{c}{r}")) for c in h["columns"] for r in h["rows"]]
        self._max_lift = float(h.get("max_lift_mm", 30.0))
        self._vols = {}

    # ---------------------------------------------------------------- the check
    def _vol(self, holding: Optional[str], hotel: bool, margin: float) -> Volumes:
        key = (holding, hotel, margin)
        if key not in self._vols:
            L = self.L if hotel else copy.deepcopy(self.L)
            if not hotel:
                L["volumes"] = dict(L["volumes"], obstacles={k: v for k, v in L["volumes"]["obstacles"].items() if k != "hotel"})
            v = Volumes(L, self._safety, holding)
            v.margin = margin
            self._vols[key] = v
        return self._vols[key]

    def corridor(self, pose: List[float]) -> Optional[str]:
        """The slot whose entry corridor the pose is in, else None."""
        for name, s in self._slots:
            if (abs(pose[1] - s[1]) <= CORRIDOR_Y_MM and abs(((pose[5] - s[5] + 180.0) % 360.0) - 180.0) <= CORRIDOR_YAW_DEG
                    and s[2] - CORRIDOR_Z_SLACK_MM <= pose[2] <= s[2] + self._max_lift + CORRIDOR_Z_SLACK_MM and pose[0] <= s[0] + 3.0):
                return name
        return None

    def check(self, pose: List[float], label: str) -> None:
        grip = self.grip()
        grip = grip if grip and grip != "none" else None
        slot = self.corridor(pose)
        for margin, kind in ((0.0, "collision"), (self._vol(grip, True, 10.0).margin, "margin")):
            try:
                self._vol(grip, slot is None, margin).check_pose(pose, label)
            except SafetyViolation as e:
                ev = Event(kind, label, [round(v, 1) for v in pose[:3]] + [round(pose[5], 1)], str(e))
                self.report.events.append(ev)
                if kind == "collision" and self.raise_on_collision:
                    raise Collision(str(e) if label in str(e) else f"{label}: {e}")
                break
        held_front = (self.L["volumes"].get("held") or {}).get(grip or "", {}).get("x", [0, 0])[1]
        front = self.L["volumes"]["obstacles"]["hotel"]["x"][0]
        if slot is not None and grip and pose[0] + held_front > front:       # the plate is in the rack, not just in front of it
            lo, hi = slot_window(self.L["plate"]["height"], self.lid)
            dz = pose[2] - dict(self._slots)[slot][2]
            if grip == "long_edges" and not (lo <= dz <= hi):
                msg = f"in slot {slot} the plate is {dz:+.1f} mm above its seat; the gap to the tray above allows {lo:+.0f}..{hi:+.1f}"
                self.report.events.append(Event("collision", label, [round(v, 1) for v in pose[:3]], msg))
                if self.raise_on_collision:
                    raise Collision(f"{label}: {msg}")

    # ---------------------------------------------------------------- motions
    def _walk(self, a: List[float], b: List[float], label: str) -> None:
        n = max(1, int(max(math.dist(a[:3], b[:3]) / STEP_MM, abs(((b[5] - a[5] + 180.0) % 360.0) - 180.0) / STEP_DEG)))
        for k in range(1, n + 1):
            t = k / n
            self.check([a[i] + t * (b[i] - a[i]) for i in range(3)] + [0.0, 0.0, a[5] + t * (((b[5] - a[5] + 180.0) % 360.0) - 180.0)],
                       f"{label} {k}/{n}")

    def _joint_walk(self, ja: List[float], jb: List[float], label: str) -> None:
        n = max(1, int(max(max(abs(jb[i] - ja[i]) for i in (1, 2, 3)) / STEP_DEG, abs(jb[0] - ja[0]) / STEP_MM)))
        for k in range(1, n + 1):
            t = k / n
            self.check(fk.fk([ja[i] + t * (jb[i] - ja[i]) for i in range(4)]), f"{label} {k}/{n}")

    def _ik(self, pose: List[float]) -> List[float]:
        cur = self._joints
        return fk.ik(pose, elbow=fk.elbow_sign(cur), j2_ref=cur[1], j3_ref=cur[2], j4_ref=cur[3])

    def _motion(self, target, kind):
        start = list(self._pose)
        label = f"{kind} -> ({target[0]:.0f}, {target[1]:.0f}, {target[2]:.0f}, yaw {target[5]:.0f})"
        if kind == "L":
            self._walk(start, list(target), label)
            self._joints = self._ik(list(target)) + self._joints[4:]
        else:                                                    # a joint move to a cartesian target
            jb = self._ik(list(target))
            self._joint_walk(self._joints[:4], jb, label)
            self._joints = jb + self._joints[4:]
        super()._motion(target, kind)

    def move_abs_joints(self, j):
        label = "abs joints -> " + ", ".join(f"{v:.0f}" for v in j[:4])
        self._joint_walk(self._joints[:4], list(j[:4]), label)
        super().move_abs_joints(j)
        self._pose = fk.fk(j[:4])                                # the real kinematics, not the taught demo pose


# ------------------------------------------------------------------------------------------------ a hotel visit
def hotel_visit(layout: dict, slot: str, lid: bool = False, lift_mm: Optional[float] = None, pick: bool = True):
    """The arm with a plate in the claw at the module hover places it in `slot` (planned mode), and picks it back, on an
    OracleRobot in the real frame. Returns (report, error text or None). Nothing here touches the real arm."""
    import os
    import sys
    import tempfile
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tests"))
    import rig
    from perception.lower_check import LowerCheck
    from procedures.corrector import StopSafe
    from procedures.cycle import Cycle
    from procedures.state import StateStore
    from procedures.telemetry import Telemetry
    L = copy.deepcopy(layout)
    L["fixtures"]["hotel"]["pose_source"] = "cad"
    if lift_mm is not None:
        L["fixtures"]["hotel"]["lift_mm"] = lift_mm
    L["routes"]["safe_z_mm"] = None
    L["limits"]["health_check_every_s"] = 0
    L["limits"]["min_disk_free_gb"] = 0
    L["gripper"]["closed"] = {"short_edges": 2.92, "long_edges": 2.92}
    L["gripper"]["open"] = {"short_edges": 10.0, "long_edges": 10.0}
    for fx in ("microscope", "polar"):
        L["fixtures"][fx].pop("grasp_width", None)
    tmp = tempfile.mkdtemp(prefix="dryrun_")
    ref_dir = os.path.join(tmp, "refs")
    lc = LowerCheck(ref_dir)
    for gate, state in rig.GATES:
        for _ in range(3):
            lc.add_reference(gate, state, rig.jpeg(rig.SAFE))
    tele = Telemetry(os.path.join(tmp, "run"))
    store = StateStore(os.path.join(tmp, "state.json"))
    hover = list(L["fixtures"]["polar"]["pose_at_0"]["pose"])
    hover[2] += L["fixtures"]["polar"]["hover_mm"]
    robot = OracleRobot(L, hover, lambda: store.s.grip if store.s.plate_at == "gripper" else None, raise_on_collision=True,
                        log=tele.log, lid=lid)
    robot._jaw = 2.92
    robot._ext[1] = L["polar"]["ext_axis"]["zero_reading"]
    robot.dis = {"P0": 1, "P90": 1}
    store.s.plate_at, store.s.grip, store.s.plate_id = "gripper", "long_edges", "DRY-1"
    cyc = Cycle(L, robot, rig.FakeVision(), tele, store, "unsupervised", os.path.join(tmp, "run"), ref_dir, clock=rig.FakeClock())
    err = None
    try:
        cyc._hotel_place(slot)
        if pick:
            cyc._hotel_pick(slot)
    except (Collision, StopSafe, SafetyViolation) as e:
        err = f"{type(e).__name__}: {e}"
    finally:
        tele.close()
    return robot.report, err
