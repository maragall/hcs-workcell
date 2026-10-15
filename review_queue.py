"""
Review queue. Highest priority first. Each entry is a decision baked into the code that you should
confirm or correct. Tell me the id when reviewed and it gets removed; new entries are inserted by priority.

Priority: P0 = can damage hardware or lose a plate. P1 = can stop the 7-day run. P2 = quality / data.
"""

QUEUE = [
    {
        "id": "Q1", "priority": "P0", "file": "nova_cell/safety.py", "what": "z floors and workspace box",
        "decision": "Every commanded target is checked against the global box and the z floor of the region it "
                    "falls in. Linear moves are sampled every 5 mm along the segment. A fixture may go below its own "
                    "floor only during its lower step (allow_floor_regions).",
        "numbers": "workspace_box x 90..330, y -170..170, z 45..430 (layout.json). Microscope region box and floor "
                   "are NOT set yet (placeholders).",
        "snippet": '''
    def check_target(self, p, label, allow_floor_regions=()):
        for axis, i in (("x", 0), ("y", 1), ("z", 2)):
            lo, hi = self.box[axis]
            if not (lo <= p[i] <= hi):
                raise SafetyViolation(...)
        region = self.region_of(p)
        floor = self.z_floor(p)
        if p[2] < floor and region not in allow_floor_regions:
            raise SafetyViolation(...)
''',
    },
    {
        "id": "Q2", "priority": "P0", "file": "nova_cell/cycle.py:_move", "what": "collision / stall detection",
        "decision": "After every move the reported pose must be within max_pose_deviation_mm (1.0) of the target, "
                    "else StopSafe. This is the software-side crash detector; the controller's collision "
                    "sensitivity is the hardware side and must be set to maximum on the pendant.",
        "numbers": "max_pose_deviation_mm = 1.0",
        "snippet": '''
        dev = max(abs(after[i] - target[i]) for i in range(3))
        if dev > self.L["limits"]["max_pose_deviation_mm"]:
            self.store.fault(label, "pose_deviation", f"{dev:.2f} mm")
            raise StopSafe(f"{label}: arm stopped {dev:.1f} mm from target; possible collision or stall")
''',
    },
    {
        "id": "Q3", "priority": "P0", "file": "nova_cell/lower_check.py", "what": "safe-to-lower gate threshold",
        "decision": "Normalised cross-correlation at 160x120 grey against >=3 reference images of the SAME fixture "
                    "state. Threshold = (lowest pairwise correlation among the references) - 0.08, never below "
                    "0.55. Fewer than 3 references => always unsafe. NaN => unsafe. Bias is toward refusing.",
        "numbers": "min_refs 3, margin 0.08, floor 0.55, retries 3 with LED 0/30/60",
        "snippet": '''
    def threshold(self, refs):
        worst = min(ncc(refs[i], refs[j]) for i in range(len(refs)) for j in range(i + 1, len(refs)))
        return max(self.floor, worst - self.margin)

    def verdict(self, fixture, state, img_bytes):
        if len(refs) < self.min_refs: return LowerVerdict(False, ...)
        best = max(ncc(cur, r) for r in refs)
        return LowerVerdict(best >= thr, ...)
''',
    },
    {
        "id": "Q4", "priority": "P0", "file": "nova_cell/cycle.py:_jaw_close + predictor.PreChecks.grasp_ok",
        "what": "grasp inference without force sensing",
        "decision": "Close in 0.5 steps with 0.15 s pauses down to 2.92. Grasp is good only if the final jaw value "
                    "is within 0.35 of 2.92. Below => nothing held (missed). Above => obstruction / not seated. "
                    "One retry with fresh vision correction, then StopSafe.",
        "numbers": "closed_on_plate 2.92, tolerance 0.35, open 10.0",
        "snippet": '''
    def grasp_ok(self, jaw):
        if abs(jaw - self.jaw_plate) <= self.jaw_tol: return None
        if jaw < self.jaw_plate - self.jaw_tol: return "nothing or too little between the fingers"
        return "obstruction or plate not seated"
''',
    },
    {
        "id": "Q5", "priority": "P0", "file": "nova_cell/cycle.py:_vision_correction",
        "what": "how far vision may move a taught pose",
        "decision": "Only dx, dy from get_pose_offset are applied (dz and rotations ignored). If the 3D magnitude "
                    "exceeds max_correction_mm (25) the cycle stops: the fixture or marker moved.",
        "numbers": "max_correction_mm 25",
        "snippet": '''
        if mag > self.safety.vision_jump_mm:
            self.corr.on_vision_too_far(label, mag)
        return [off.d[0], off.d[1], 0.0, 0.0, 0.0, 0.0]
''',
    },
    {
        "id": "Q6", "priority": "P1", "file": "nova_cell/state.py + cycle.run", "what": "resume rule",
        "decision": "State is written before and after each step. On start, if the last step did not finish, the "
                    "cycle refuses to run until a human inspects and edits state.json. No automatic resume of a "
                    "half-finished grasp or release.",
        "snippet": '''
        if not s.step_done:
            raise StopSafe(f"last step '{s.step}' did not finish; plate_at={s.plate_at} grip={s.grip}. ...")
''',
    },
    {
        "id": "Q7", "priority": "P1", "file": "nova_cell/corrector.py", "what": "recovery budgets",
        "decision": "Per cycle: 1 regrasp, 3 re-images (LED 0/30/60), 1 polar retry, 1 re-enable. Anything else "
                    "=> StopSafe + ALERT.txt. In supervised mode the operator is asked; unsupervised never asks.",
        "snippet": "Budget(regrasp=1, reimage=3, polar_retry=1, enable_retry=1)",
    },
    {
        "id": "Q8", "priority": "P1", "file": "nova_cell/polar.py", "what": "polar stage interface is unknown",
        "decision": "Three drivers: robot DO pulse, TCP line device, manual prompt. Manual is refused in "
                    "unsupervised mode. Needs the real interface of the rotation unit.",
    },
    {
        "id": "Q9", "priority": "P1", "file": "nova_cell/trajectory.py:normalise", "what": "demo averaging",
        "decision": "Trim static ends, resample each demo to 200 points by joint-space arc length, average, "
                    "7-point moving average, thin to <=15 waypoints with joint steps <= 25 deg. Reject if any "
                    "step exceeds 25 deg. Each waypoint is checked against the box before replay.",
        "numbers": "n_samples 200, max_points 15, max_joint_step 25 deg",
    },
    {
        "id": "Q10", "priority": "P2", "file": "nova_cell/predictor.py:DriftMonitor", "what": "drift thresholds",
        "decision": "Robust z-score on the last 60 values per feature (median / MAD). z>=3.5 warn, z>=6 stop. "
                    "Needs 8 samples before it speaks. Features: step durations, jaw closed value, vision offset "
                    "magnitude, lower-check scores, plate-in-gripper scores, cycle time.",
    },
    {
        "id": "Q11", "priority": "P2", "file": "layout.json speeds", "what": "speed profiles",
        "decision": "supervised: travel 5%, approach 2%, contact 1%. unsupervised: travel 3%, approach 1.5%, "
                    "contact 1%. Both can be raised only after the acceptance tests.",
    },
]
