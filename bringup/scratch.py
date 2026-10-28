"""
A layout for a cell nobody has taught: what a customer's first `python cli.py setup --scratch` starts from.

Everything here is a product constant (the CAD in hardware.json, the claw, the plate, the speed profiles) or a nominal value
computed from them (poses from the CAD through a Registration, the nest, the zones). There is no taught pose, no taught route
(`trajectories` is empty: the planner finds every way, routes/plan.py), no learned grip width, no reference picture, no result
of an earlier setup. The setup then measures what the machine cannot know: the plate on the module by the aruco servo, the stage
seat, the jaw widths, the hotel size.

    layout = scratch.build(hardware)                          # nominal frame: rotation 0, shift from the arm model
    layout = scratch.build(hardware, registration=reg)        # the frame the arm found itself (bringup/register.py)
    scratch.compare(layout, taught_layout)                    # how far each value is from a hand-taught cell's

What cannot come from the CAD and is NOT invented here is listed in `CANNOT_BE_KNOWN`.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

from bringup.register import K_ASSUMED, Registration, m
from routes_opt import fk

# What no file and no formula gives: the cell has to be taught (or measured) once for these. Each is the exact layout key.
CANNOT_BE_KNOWN = (
    "polar.ext_axis.zero_reading / direction: the axis reading with the plate parallel to x (the module's own zero)",
    "fixtures.microscope.pose[2]: the stage seat's height (the operator's heights give a nominal, 34 mm below a taught one: it "
    "needs a slow contact descent, not built)",
    "gripper.open.*: how wide the fingers may be opened on each grip (the microscope holder limits it)",
    "gripper.closed / accept_band: provisional wide values until the grips section measures them",
    "fixtures.hotel.grip_back_mm['15']: the 15-slot hotel's grip offset",
)

# the plate (operator's calipers, 2026-10-19 / 2026-10-26)
PLATE = {"width_between_long_edges": 84.57, "width_over_skirt": 85.14, "length_between_short_edges": 125.03,
         "length_over_skirt": 127.46, "height": 14.42, "height_practice": 14.5, "height_with_lid": 16.85}

# the claw as the CAD has it, in the tool frame, relative to the wrist axis the controller reports (hardware.json cad.claw)
CLAW = {"body": {"x": [-41.0, 88.0], "y": [-72.0, 72.0]}, "fingers": {"x": [88.0, 205.0], "y": [-48.0, 48.0]},
        "z": [-25.0, 60.0], "links_z": [-25.0, 130.0]}

# the operator's heights of the microscope above the base (2026-10-28)
MICROSCOPE = {"stage_lips_above_base_mm": 210.0, "stage_above_base_mm": 190.0, "flatfield_module_above_base_mm": 260.0}
PLATFORM_TOP_C = 93.0            # CAD y of the platform's top
SEAT_XY_YAW = (123.0, 340.0, 90.0)   # hardware.json microscope.placement: the holder within 20 mm of (123, 340), the claw pointing +y; 340 mm from j1 leaves the camera 40 mm of reach ahead
MICROSCOPE_HOVER_MM = 67.0
TAG_VIEW_SAFE_MM = 30.0          # clearance of the claw's lowest edge over the instrument's highest part
FOLDED_Z = 240.0                 # the nest's height: the middle of the workspace


def _rot(theta_deg: float, x: float, y: float):
    c, s = math.cos(math.radians(theta_deg)), math.sin(math.radians(theta_deg))
    return c * x - s * y, s * x + c * y


def nominal_registration(hardware: dict, theta_deg: float = 0.0) -> Registration:
    """The CAD frame in the robot's with no observation: rotation 0, shift so that j1 of the CAD is the arm model's j1."""
    cad = hardware["cad"]
    j1_c = cad["checks"]["j1_cad_xz"]                                   # (x_c, z_c)
    mc = m(j1_c[0], j1_c[1])
    j1_r = (fk.MODEL["x0"], fk.MODEL["y0"])
    rx, ry = _rot(theta_deg, float(mc[0]), float(mc[1]))
    ax = cad["rotation_module"]["axis_c"]
    axis = _rot(theta_deg, ax["z"], ax["x"])
    return Registration(theta_deg, j1_r[0] - rx, j1_r[1] - ry, K_ASSUMED,
                        "nominal: the wrist reading is at the fingers' mid height (checked against taught seats to ~1 mm)",
                        (axis[0] + j1_r[0] - rx, axis[1] + j1_r[1] - ry), {}, ("nominal frame: no observation of the module yet",))


def registration_dict(reg: Registration) -> dict:
    return {"theta_deg": reg.theta_deg, "tx": reg.tx, "ty": reg.ty, "k": reg.k, "k_source": reg.k_source,
            "axis_robot": list(reg.axis_robot), "checks": dict(reg.checks), "warnings": list(reg.warnings)}


def registration_from(d: dict) -> Registration:
    return Registration(float(d["theta_deg"]), float(d["tx"]), float(d["ty"]), float(d["k"]), d.get("k_source", ""),
                        tuple(d["axis_robot"]), dict(d.get("checks", {})), tuple(d.get("warnings", ())))


def _cad(reg: Registration, x_r: float, y_r: float):
    """A point of hardware.json's nominal robot frame (x = z_c + 228.1, y = x_c) through the registration."""
    p = _rot(reg.theta_deg, x_r - 228.1, y_r)
    return p[0] + reg.tx, p[1] + reg.ty


def _box(reg: Registration, x: List[float], y: List[float]) -> Dict[str, List[float]]:
    pts = [_cad(reg, a, b) for a in x for b in y]
    return {"x": [round(min(p[0] for p in pts), 1), round(max(p[0] for p in pts), 1)],
            "y": [round(min(p[1] for p in pts), 1), round(max(p[1] for p in pts), 1)]}


def build(hardware: dict, taught_layout: Optional[dict] = None, registration: Optional[Registration] = None) -> dict:
    """The scratch layout. `taught_layout` is accepted for one thing only: the module's own zero (CANNOT_BE_KNOWN), and only if the
    caller passes it; it is never copied otherwise."""
    from procedures.cad_poses import CadPoses
    cad = hardware["cad"]
    reg = registration or nominal_registration(hardware)
    grip = cad["grip"]
    head, back_long, back_short = float(grip["head_centre_ahead_mm"]), float(grip["long_edges_back_mm"]), float(grip["short_edges_back_mm"])
    plate_h = float(PLATE["height"])
    theta = reg.theta_deg

    # ---- the rotation module: the plate's centre on the pad is the module's axis; the wrist stands behind it along the claw
    axis_c = cad["rotation_module"]["axis_c"]
    ax, ay = reg.xy(axis_c["x"], axis_c["z"])
    d = (math.cos(math.radians(-90.0 + theta)), math.sin(math.radians(-90.0 + theta)))       # the claw points -y at the module
    z_module = reg.z(cad["rotation_module"]["pad_top_y_c"] + plate_h / 2.0)
    short_reach, long_reach = head - back_short, head + back_long
    pose_at_0 = [round(ax - short_reach * d[0], 3), round(ay - short_reach * d[1], 3), round(z_module, 3), 0.0, 0.0, round(-90.0 + theta, 3)]
    pose_at_90 = [round(ax - long_reach * d[0], 3), round(ay - long_reach * d[1], 3), round(z_module, 3), 0.0, 0.0, round(-90.0 + theta, 3)]

    # ---- the microscope's nominal seat: README placement, the stage height from the operator
    platform_top_z = reg.z(PLATFORM_TOP_C)
    seat_z = round(platform_top_z + MICROSCOPE["stage_above_base_mm"] + plate_h / 2.0, 3)
    sx, sy, syaw = SEAT_XY_YAW
    seat = [sx, sy, seat_z, 0.0, 0.0, syaw]
    highest = platform_top_z + MICROSCOPE["flatfield_module_above_base_mm"]
    free_above = math.ceil((highest - CLAW["z"][0] + TAG_VIEW_SAFE_MM) / 10.0) * 10.0

    # ---- the hotel from the CAD
    stub = {"fixtures": {"hotel": {"taught": {}, "grip_back_mm": {"30": back_long, "15": None}}}, "plate": dict(PLATE),
            "setup": {"hotel": {"size": 30}}}
    cp = CadPoses(stub, hardware, registration=reg)
    row1_z = cp.hotel_pose("A1")[2]
    hot = cad["hotel"]
    rows = list(range(1, int(hot["rows"]) + 1))
    active = [f"{'AC'[(r - 1) % 2]}{r}" for r in rows]
    wrist_x = cp.hotel_pose("A1")[0]

    # ---- the nest: wrist over j1, claw along +x
    nest = [round(fk.MODEL["x0"], 3), round(fk.MODEL["y0"], 3), FOLDED_Z, 0.0, 0.0, round(theta, 3)]

    L: dict = {
        "_units": "robot base frame, mm, radians; rz = gripper yaw",
        "_scratch": "built by bringup/scratch.py from hardware.json: no taught pose, route or grip; the setup measures the rest",
        "registration": registration_dict(reg),
        "plate": dict(PLATE, groove_offset_from_bottom=None),
        "plates": {"default": "production", "types": {"production": {
            "id": 1, "length_mm": PLATE["length_over_skirt"], "width_mm": PLATE["width_over_skirt"], "height_mm": plate_h,
            "grab_height_mm": 0.0}}},
        "robot": {"ip": None, "port": 52525, "grip_axis": 0},
        "vision": {
            "host": None, "port": 9000, "down_camera": "auto_teach_camera", "front_camera": "scan_camera", "workspace": "autoteach",
            "ssh": {"user": None, "password": None, "key_path": None}, "marker_mm": 40, "max_correction_mm": 25.0, "led": 0,
            "front_led": 0,
            "lower_check": {"min_refs": 3, "margin": 0.08, "floor": 0.55, "retries": 3, "front": {"auto_refs": 12, "enforce": "auto"}},
            "hub": {"record": False, "outbox": "hub_outbox"},
            "seated_front": {"roi": [0.0, 0.55, 1.0, 1.0], "auto_refs": 12, "enforce": "auto"},
            "slot_check": {"min_refs": 5, "auto_refs": 24, "spacing_s": 1800, "min_match": 0.75, "enforce": "auto"},
            "held_plate_auto_refs": 12, "stage_check": {"auto_refs": 12, "enforce": "auto"}, "calibration": {}},
        # nothing learned: the grips section measures closed and accept_band; these only let the first carry run
        "gripper": {"open": {"short_edges": 28.5, "long_edges": 8.2}, "closed": {"short_edges": 20.0, "long_edges": 0.5},
                    "tolerance": 0.1, "stall_lag": 0.3, "readings": {"empty_closed": 0.0}, "release_tolerance": 0.3,
                    "accept_band": {"short_edges": [15.0, 27.0], "long_edges": [0.8, 7.0]},
                    "_provisional": "open and closed are nominal, accept_band wide, until the grips section measures them"},
        "speeds": {"supervised": {"travel": 0.2, "approach": 0.1, "contact": 0.05, "jaw_step": 0.5, "jaw_pause": 0.15},
                   "unsupervised": {"travel": 0.03, "approach": 0.015, "contact": 0.01, "jaw_step": 0.5, "jaw_pause": 0.15},
                   "fast": {"travel": 0.4, "approach": 0.2, "contact": 0.08, "jaw_step": 0.5, "jaw_pause": 0.15},
                   "stress": {"travel": 0.5, "approach": 0.25, "contact": 0.1, "jaw_step": 0.5, "jaw_pause": 0.1}},
        "workspace_box": {"x": [90, 360], "y": [-170, 450], "z": [45, 435.0]},
        "regions": {
            "microscope": {"box": {"x": [round(sx - 63.0, 1), round(sx + 97.0, 1)], "y": [round(sy - 150.0, 1), round(sy + 50.0, 1)]},
                           "z_floor": round(seat_z - 4.6, 3), "no_swing": True, "max_lateral_mm": 2.0, "max_yaw_change_deg": 0.5,
                           "needs_parked": True, "free_above_z": free_above,
                           "_note": f"highest part {highest:.0f} (platform top {platform_top_z:.1f} + flatfield module "
                                    f"{MICROSCOPE['flatfield_module_above_base_mm']:.0f}); the claw's lowest edge is the wrist z {CLAW['z'][0]:.0f}"},
            "polar": {"box": {"x": [round(pose_at_0[0] - 43.0, 1), round(pose_at_0[0] + 47.0, 1)],
                              "y": [round(pose_at_0[1] - 45.0, 1), round(pose_at_0[1] + 45.0, 1)]}, "z_floor": round(z_module - 3.8, 3)},
            "hotel": {"box": {"x": [round(wrist_x - 7.0, 1), round(wrist_x + 68.0, 1)], "y": [-170, 170]}, "z_floor": round(row1_z - 19.6, 3)},
            "stage_sweep": {"box": None, "keep_out": True}},
        "fixtures": {
            "microscope": {"pose": seat, "hover_mm": MICROSCOPE_HOVER_MM, "approach": {"axis": [0, 0, 1], "mm": 0}, "grip": "short_edges",
                           "aruco_marker_id": 1,
                           "grasp_search": {"axes": ["x", "y"], "step_mm": 0.2, "max_mm": 0.8, "z_offsets_mm": [-0.3, 0.3, -0.6],
                                            "step_deg": 0.3, "max_deg": 0.6},
                           "_note": "nominal seat: x, y, yaw from docs/PLACEMENT.md, z from the operator's stage height; the setup finds "
                                    "x, y and yaw by the aruco servo; z is provisional"},
            "polar": {"pose_at_0": {"pose": pose_at_0}, "pose_at_90": {"pose": pose_at_90}, "hover_mm": 34.4, "clear_mm": 60,
                      "aruco_marker_id": 1, "display_name": "rotating-stage", "open_width": {"short_edges": 29.0},
                      "grasp_search": {"axes": ["z", "x", "y"], "step_mm": 0.3, "max_mm": 1.2, "z_offsets_mm": [-0.5, 0.5],
                                       "step_deg": 0.5, "max_deg": 1.5, "adopt_max_mm": 1.0}},
            "hotel": {"approach": {"axis": [-1, 0, 0], "mm": 100.0}, "lift_mm": 25.0, "max_lift_mm": 30, "headroom_mm": 17,
                      "grip_back_mm": {"30": back_long, "15": None}, "columns": list(hot["columns_y_r"]), "rows": rows, "taught": {},
                      "climb_x": 127.0, "active_slots": active, "loaded": {}, "pose_source": "cad",
                      "tray_pitch_mm": float(hot["row_pitch_mm"]), "tray_thickness_mm": 8.0, "tray_clearance_mm": 1.5}},
        "polar": {"driver": "ext_axis", "settle_s": 1.0,
                  "ext_axis": {"index": 1, "zero_reading": None, "direction": -1, "speed": 0.3, "tolerance_deg": 0.5,
                               "forward_max_deg": 100.0},
                  "do": {"rotate_90": "POLAR_ROT", "home": "POLAR_HOME", "pulse_s": 0.5}, "di": None},
        "volumes": {
            "margin_mm": 10.0, "z_offset_k": reg.k, "axis_keepout_mm": 60.0,
            "obstacles": {
                "hotel": {"x": _box(reg, hot["x_r"], [-hot["side_panels_y_r"][1], hot["side_panels_y_r"][1]])["x"],
                          "y": _box(reg, hot["x_r"], [-hot["side_panels_y_r"][1], hot["side_panels_y_r"][1]])["y"],
                          "z": [round(reg.z(104.0) - 6.6, 1), round(reg.z(508.0) + 9.4, 1)]},
                "pillar": {"x": _box(reg, cad["pillar"]["x_r"], cad["pillar"]["y_r"])["x"],
                           "y": _box(reg, cad["pillar"]["x_r"], cad["pillar"]["y_r"])["y"],
                           "z": [round(reg.z(90.5) - 0.0, 1), round(reg.z(760.5), 1)], "links": False}},
            "claw": CLAW,
            "held": {"long_edges": {"x": [round(long_reach - PLATE["length_over_skirt"] / 2, 1), round(long_reach + PLATE["length_over_skirt"] / 2, 1)],
                                    "y": [-round(PLATE["width_over_skirt"] / 2, 1), round(PLATE["width_over_skirt"] / 2, 1)]},
                     "short_edges": {"x": [round(short_reach - PLATE["width_over_skirt"] / 2, 1), round(short_reach + PLATE["width_over_skirt"] / 2, 1)],
                                     "y": [-round(PLATE["length_over_skirt"] / 2, 1), round(PLATE["length_over_skirt"] / 2, 1)]},
                     "hotel_front_allowance_mm": 45.0}},
        "routes": {"safe_z_mm": None, "positions": ["microscope", "polar.pose_at_0", "polar.pose_at_90"],
                   "nest": {"pose": nest, "_note": "the folded pose: wrist over j1, claw along +x; a joint move reaches and leaves it"}},
        "trajectories": {},
        "limits": {"max_waypoint_joint_step_deg": 25.0, "max_pose_deviation_mm": 1.0, "max_restart_offset_mm": 5.0, "min_disk_free_gb": 5.0,
                   "operator_timeout_s": 600, "step_timeout_s": 120, "health_check_every_s": 30, "max_waypoint_pose_dev_mm": 6.0,
                   "vision_retry_s": 300, "slow_zone_mm": 6.0, "max_yaw_deviation_deg": 0.5, "stage_park_timeout_s": 120},
        "squid": {"microscope": {"park_xy_mm": None, "park_tolerance_mm": None, "stage_limits_mm": None}},
    }
    if taught_layout is not None:                      # the module's own zero, the one value the CAD cannot give (CANNOT_BE_KNOWN)
        z = (taught_layout.get("polar", {}).get("ext_axis", {}))
        L["polar"]["ext_axis"].update({k: z[k] for k in ("zero_reading", "direction") if z.get(k) is not None})
    return L


def compare(scratch: dict, taught: dict) -> List[dict]:
    """How far each scratch value is from the taught cell's: module poses, the nest, the microscope seat, the hotel slots."""
    from procedures.cad_poses import CadPoses
    rows: List[dict] = []

    def add(name: str, a, b):
        if a is None or b is None:
            return
        d = [round(float(x) - float(y), 2) for x, y in zip(a[:3], b[:3])] + [round(float(a[5]) - float(b[5]), 2)]
        rows.append({"what": name, "scratch": [round(float(v), 2) for v in a[:3]] + [round(float(a[5]), 2)],
                     "taught": [round(float(v), 2) for v in b[:3]] + [round(float(b[5]), 2)], "diff": d})

    for k in ("pose_at_0", "pose_at_90"):
        add(f"module {k}", scratch["fixtures"]["polar"][k]["pose"], (taught["fixtures"]["polar"].get(k) or {}).get("pose"))
    add("nest", scratch["routes"]["nest"]["pose"], (taught["routes"].get("nest") or {}).get("pose"))
    add("microscope seat", scratch["fixtures"]["microscope"]["pose"], taught["fixtures"]["microscope"].get("pose"))
    cp = CadPoses({"fixtures": {"hotel": {"taught": {}, "grip_back_mm": scratch["fixtures"]["hotel"]["grip_back_mm"]}}, "plate": scratch["plate"],
                   "setup": {"hotel": {"size": 30}}}, hardware=_hardware_of(scratch), registration=registration_from(scratch["registration"]))
    for slot, pose in taught["fixtures"]["hotel"].get("taught", {}).items():
        if slot[:1] in scratch["fixtures"]["hotel"]["columns"] and slot[1:].isdigit():
            add(f"hotel {slot}", cp.hotel_pose(slot), pose)
    return rows


def _hardware_of(scratch: dict) -> dict:
    from procedures.cad_poses import load_hardware
    return load_hardware()


def table(rows: List[dict]) -> str:
    out = [f"{'what':22} {'scratch x, y, z, yaw':36} {'taught x, y, z, yaw':36} difference"]
    for r in rows:
        out.append(f"{r['what']:22} {str(r['scratch']):36} {str(r['taught']):36} {r['diff']}")
    return "\n".join(out)
