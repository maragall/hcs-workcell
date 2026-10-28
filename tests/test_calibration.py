"""The calibration package: the marker rendered through the vendor's files and solved back, the file formats
byte-compared, the plan, capture on the fakes, upload on a mocked SFTP, verify on a fake box."""
from __future__ import annotations

import glob
import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from calibration import capture, halcon, marker, plan, solve, synth, upload, validate, verify  # noqa: E402
from devices.fake_arm import SimRobot  # noqa: E402
from devices.vision_box import ArucoPose  # noqa: E402

CAL = os.path.join(ROOT, "vendor_data", "CalibrationData")
VCAM = halcon.CamParam.read(os.path.join(CAL, "intrinsic", "CamParam.dat"))
VPOSE = halcon.Pose.read(os.path.join(CAL, "HandEye", "CamInTool.dat"))
START = halcon.CamParam.read(os.path.join(CAL, "IniIntrinsic", "CamParam.dat"))


# ------------------------------------------------------------------ marker
def test_marker_corners_are_detected_where_they_were_drawn():
    T_bm = synth.facing(validate.START_POSE, VPOSE, 0.2)
    scene = synth.Scene(VCAM, VPOSE, T_bm, 0.04, noise_px=0.3)
    img = scene.render(validate.START_POSE)
    got = marker.detect(img)
    assert got.shape == (4, 2)
    assert np.linalg.norm(got - scene.corners(validate.START_POSE), axis=1).max() < 0.5
    with pytest.raises(marker.MarkerNotFound):
        marker.detect(np.full((400, 600), 128, np.uint8))


# ------------------------------------------------------------------ file formats
def test_camparam_round_trip_is_byte_identical():
    path = os.path.join(CAL, "intrinsic", "CamParam.dat")
    with open(path, "rb") as f:
        assert VCAM.dumps(VCAM.written).encode() == f.read()


def test_pose_round_trip_is_byte_identical():
    path = os.path.join(CAL, "HandEye", "CamInTool.dat")
    with open(path, "rb") as f:
        assert VPOSE.dumps(VPOSE.written).encode() == f.read()
    assert np.allclose(halcon.matrix_to_abg(VPOSE.matrix[:3, :3]), VPOSE.r, atol=1e-9)


def test_division_model_file_has_kappa_group(tmp_path):
    cam = halcon.CamParam(0.003, 1.12e-6, 1.12e-6, 1640, 1232, 3280, 2464, "area_scan_division", kappa=-100.0)
    p = tmp_path / "CamParam.dat"
    cam.write(str(p))
    text = p.read_bytes().decode()
    assert "Kappa:kappa:\t-100;" in text and "\r\n" in text and "Poly1" not in text
    assert halcon.CamParam.read(str(p)) == cam


# ------------------------------------------------------------------ the plan
def test_plan_spreads_and_ends_where_it_started():
    here = [400.0, -30.0, 300.0, 0, 0, -2.0]
    poses = plan.around(here)
    assert 30 <= len(poses) <= 36 and poses[-1] == here
    arr = np.array(poses[:-1])
    assert len(set(arr[:, 2])) == 4 and len(set(arr[:, 5])) == 4
    assert arr[:, 0].min() < here[0] - 15 < here[0] + 15 < arr[:, 0].max()
    dropped = plan.around(here, check=lambda p: (_ for _ in ()).throw(ValueError()) if p[2] > 320 else None)
    assert all(p[2] <= 320 for p in dropped) and dropped[-1] == here


# ------------------------------------------------------------------ the solve, against the vendor
@pytest.fixture(scope="module")
def solved(tmp_path_factory):
    folder = str(tmp_path_factory.mktemp("captures"))
    validate.synthetic_folder(VCAM, VPOSE, folder)
    return solve.run(folder, START, validate.MARKER_M), folder


def test_capture_folder_looks_right(solved):
    _, folder = solved
    pngs = sorted(glob.glob(os.path.join(folder, "*.png")))
    assert len(pngs) == len(plan.around(validate.START_POSE)) and os.path.basename(pngs[0]) == "001.png"
    assert all(os.path.exists(p[:-4] + ".json") for p in pngs)


def test_joint_solve_matches_the_vendor(solved):
    res, _ = solved
    for name in ("focus", "sx", "cx", "cy"):
        assert abs(getattr(res.camparam, name) - getattr(VCAM, name)) / getattr(VCAM, name) < 0.001, name
    drot = np.degrees(np.linalg.norm(cv2.Rodrigues(res.camintool.matrix[:3, :3].T @ VPOSE.matrix[:3, :3])[0]))
    assert drot < 0.05
    assert np.abs((np.array(res.camintool.t) - np.array(VPOSE.t)) * 1e3).max() < 0.1
    assert res.unobservable == ("tool z",) and res.images >= 16 and res.reprojection_px < 0.3
    assert np.sign(res.camparam.poly).tolist() == np.sign(VCAM.poly).tolist()


def test_result_writes_both_files(solved, tmp_path):
    res, _ = solved
    a, b = res.write(str(tmp_path))
    cam, pose = halcon.CamParam.read(a), halcon.Pose.read(b)
    assert np.isclose(cam.focus, res.camparam.focus) and np.allclose(cam.poly, res.camparam.poly) and cam.model == res.camparam.model
    assert np.allclose(pose.r, res.camintool.r) and np.allclose(pose.t, res.camintool.t)


# ------------------------------------------------------------------ capture on the fakes
def test_capture_on_the_fake_arm_and_box(tmp_path):
    robot = SimRobot(validate.START_POSE)
    scene = synth.Scene(VCAM, VPOSE, synth.facing(validate.START_POSE, VPOSE, 0.2), 0.04)
    poses = plan.around(validate.START_POSE)[:3]
    n = capture.run_plan(poses, "scan_camera", robot, synth.FakeBox(scene, robot), str(tmp_path))
    assert n == 3 and robot._speed == capture.DEFAULT_SPEED and [t[1] for t in robot.trace] == poses
    assert cv2.imread(str(tmp_path / "002.png")).shape == (2464, 3280, 3)
    assert capture.run_plan(None, "scan_camera", robot, synth.FakeBox(scene, robot), str(tmp_path)) == 4


# ------------------------------------------------------------------ upload on a mocked SFTP
class FakeSftp:
    def __init__(self, files):
        self.files = dict(files)
        self.dirs = set()

    def stat(self, path):
        if path in self.files or path in self.dirs or any(d.startswith(path + "/") for d in self.dirs):
            return True
        raise IOError(path)

    def mkdir(self, path):
        self.dirs.add(path)

    def rename(self, a, b):
        self.files[b] = self.files.pop(a)

    def put(self, local, remote):
        with open(local, "rb") as f:
            self.files[remote] = f.read()

    def close(self):
        pass


class FakeClient:
    def __init__(self, sftp):
        self.sftp, self.commands, self.closed = sftp, [], False

    def open_sftp(self):
        return self.sftp

    def exec_command(self, cmd):
        self.commands.append(cmd)

        class Ch:
            def recv_exit_status(self):
                return 0

        class Out:
            channel = Ch()
        return None, Out(), None

    def close(self):
        self.closed = True


def test_upload_backs_up_and_overwrites(tmp_path):
    cam, pose = tmp_path / "CamParam.dat", tmp_path / "CamInTool.dat"
    cam.write_bytes(b"new cam")
    pose.write_bytes(b"new pose")
    d = upload.remote_dir("scan_camera")
    sftp = FakeSftp({f"{d}/CamParam.dat": b"old cam", f"{d}/CamInTool.dat": b"old pose"})
    clients = []

    def connect_fn(host, creds):
        assert host == "box" and creds["user"] == "pi"
        clients.append(FakeClient(sftp))
        return clients[-1]

    creds = {"user": "pi", "password": "x", "key_path": None}
    backups = upload.push("box", "scan_camera", str(cam), str(pose), creds, connect_fn, now=0)
    assert d == "/home/pi/Nova/data/calibration/scan_camera/active"
    assert sftp.files[f"{d}/CamParam.dat"] == b"new cam" and sftp.files[f"{d}/CamInTool.dat"] == b"new pose"
    assert len(backups) == 2 and all(b.startswith(d + "/Cam") and b.endswith(".bak") for b in backups)
    assert sftp.files[backups[0]] == b"old cam" and clients[0].closed
    upload.reboot("box", creds, connect_fn)
    assert clients[-1].commands == ["sudo reboot"]


def test_upload_refuses_without_credentials():
    with pytest.raises(upload.NoCredentials):
        upload.credentials({"vision": {"ssh": {"user": None, "password": None, "key_path": None}}})
    with pytest.raises(upload.NoCredentials):
        upload.credentials({"vision": {}})
    assert upload.credentials({"vision": {"ssh": {"user": "pi", "key_path": "/k"}}})["key_path"] == "/k"


# ------------------------------------------------------------------ verify on a fake box
class MarkerBox:
    def aruco_pose(self, camera, marker_id=1, marker_size_m=None, led=0):
        return ArucoPose([0, 0, 0, 0], [0] * 6, [300.0, -50.0, 20.0, 0, 0, 0], "robot")


def test_verify_reports_the_marker_drift_between_two_poses():
    robot = SimRobot([0, 0, 0, 0, 0, 0])
    assert verify.check(MarkerBox(), "auto_teach_camera", robot, [100, 0, 50, 0, 0, 0], [120, 0, 50, 0, 0, 0]) == 0.0
    assert len(robot.trace) == 2


def test_the_plate_tag_is_found_in_any_turn_with_the_same_corner_first():
    """The tag on the practice plate is a black square with a 6 x 6 code, not a dictionary marker: the corners come out in
    the code's own order whatever the yaw, so they mean the same thing in every picture."""
    import cv2
    import numpy as np
    from calibration import marker
    cell = 60
    tag = np.full((8 * cell, 8 * cell), 0, np.uint8)
    for i in range(4):
        for j in range(4):
            if marker.PLATE_TAG[i, j]:
                tag[(i + 1) * cell:(i + 2) * cell, (j + 1) * cell:(j + 2) * cell] = 255
    corners_tag = np.array([[0, 0], [8 * cell, 0], [8 * cell, 8 * cell], [0, 8 * cell]], np.float32)
    for turn in range(4):
        canvas = np.full((900, 1200), 235, np.uint8)
        angle = turn * 90 + 17.0
        centre = (8 * cell / 2, 8 * cell / 2)
        R = cv2.getRotationMatrix2D(centre, -angle, 1.0)
        pts = np.hstack([corners_tag, np.ones((4, 1), np.float32)]) @ R.T
        shift = np.array([380.0, 250.0], np.float32) - pts.mean(0)
        M = np.vstack([R, [0, 0, 1]]).astype(np.float64)
        T = np.array([[1, 0, shift[0]], [0, 1, shift[1]], [0, 0, 1]], np.float64)
        canvas = cv2.warpPerspective(tag, T @ M, (1200, 900), dst=canvas, borderMode=cv2.BORDER_TRANSPARENT)
        got = marker.detect(canvas, None)
        want = (np.hstack([corners_tag, np.ones((4, 1), np.float32)]) @ (T @ M)[:2].T)
        assert np.abs(got - want).max() < 2.0, (turn, got.tolist(), want.tolist())


# ------------------------------------------------------------------ the staged solve for a flat marker at a known height
CELL_CAMINTOOL = halcon.Pose.from_matrix(np.block([[cv2.Rodrigues(np.array([2.257, -2.257, 0.126]))[0], np.array([[0.0785], [0.0014], [0.0127]])],
                                                   [np.zeros((1, 3)), np.ones((1, 1))]]))
CELL_MARKER = np.block([[halcon.rot_z(np.radians(-176.8)), np.array([[0.3203], [0.1205], [0.0741]])], [np.zeros((1, 3)), np.ones((1, 1))]])


def _cell_folder(tmp_path, noise_px=0.3):
    """The cell's geometry: the camera looks down from the hand, the plate lies flat 74 mm above the base plane."""
    start = [311.0, 233.1, 161.2, 0.0, 0.0, -91.3]
    scene = synth.Scene(VCAM, CELL_CAMINTOOL, CELL_MARKER, 0.04, noise_px)
    arm = validate._Arm(start)
    capture.run_plan(plan.around(start), "cam", arm, synth.FakeBox(scene, arm), str(tmp_path))
    return scene


def test_the_flat_solve_recovers_the_camera_from_noisy_synthetic_captures(tmp_path):
    _cell_folder(tmp_path)
    res = solve.run(str(tmp_path), START, 0.04, marker_z_m=0.0741)
    # the lens is the weak part (12 poses, 4 corners each): about 2 px is left even from the true extrinsics, the extrinsics are right to a mm
    assert res.reprojection_px < 3.5 and res.mean_error_m < 1e-3
    assert np.abs((np.array(res.camintool.t) - np.array(CELL_CAMINTOOL.t)) * 1e3).max() < 1.5            # mm
    assert np.abs((res.marker_in_base.matrix[:2, 3] - CELL_MARKER[:2, 3]) * 1e3).max() < 1.5
    assert res.unobservable == ("marker height",)


def test_the_flat_solve_drops_a_picture_taken_too_close_and_repeats_of_one_pose(tmp_path):
    _cell_folder(tmp_path)
    caps = solve.load_folder(str(tmp_path))
    twice = solve.Captures(caps.T_bt + caps.T_bt[:3], caps.corners + caps.corners[:3], caps.files + caps.files[:3],
                           caps.width, caps.height)
    assert len(solve.distinct(twice)) == len(caps.files)


def test_the_flat_solve_on_the_real_captures_of_the_cell():
    folder = os.path.join(ROOT, "reference_images", "calibration", "auto_teach_camera", "capture")
    if len(glob.glob(os.path.join(folder, "*.png"))) < 20:
        pytest.skip("this checkout has no real captures")
    start = halcon.CamParam.read(os.path.join(CAL, "IniIntrinsic", "CamParam.dat"))
    res = solve.run(folder, start, 0.04, marker_z_m=0.0741)
    assert res.reprojection_px < 5.0 and res.mean_error_m < 1e-3                  # 3.0 px, 0.4 mm when written
    x, y = res.marker_in_base.matrix[:2, 3] * 1e3
    assert abs(x - 312.6) < 15 and abs(y - 118.0) < 15                            # the module's axis in the CAD


class _Cell:
    """What bringup.cameras.solve needs of a cell."""
    sim = False

    def __init__(self, tmp):
        self.ref_dir = str(tmp)
        self.L = {"vision": {"down_camera": "down_cam", "front_camera": "front_cam", "marker_mm": 40, "led": 0}}
        self.notes = []

    def layout(self):
        return self.L

    def edit(self, fn):
        fn(self.L)

    def note(self, text):
        self.notes.append(text)


def _result(px, mm):
    from types import SimpleNamespace
    written = []
    return SimpleNamespace(reprojection_px=px, mean_error_m=mm / 1e3, images=9, write=lambda d: written.append(d)), written


def test_a_refused_solve_writes_no_file_and_no_number_but_last_failed(tmp_path, monkeypatch):
    from bringup import cameras
    cell = _Cell(tmp_path)
    res, written = _result(9.0, 5.0)
    monkeypatch.setattr(solve, "run", lambda *a, **k: res)
    out = cameras.solve(cell, "down")
    node = cell.L["vision"]["calibration"]["down_cam"]
    assert out["down_cam"]["ok"] is False and not written
    assert "px" not in node and node["last_failed"]["px"] == 9.0
    assert "nothing was written" in cell.notes[-1]


def test_an_accepted_solve_writes_and_the_next_one_starts_from_it_not_from_a_refused_one(tmp_path, monkeypatch):
    from bringup import cameras
    cell = _Cell(tmp_path)
    starts = []

    good, written = _result(0.3, 0.5)
    monkeypatch.setattr(solve, "run", lambda folder, start, *a, **k: (starts.append(start), good)[1])
    cameras.solve(cell, "down")
    assert written and cell.L["vision"]["calibration"]["down_cam"]["px"] == 0.3
    # a refused solve afterwards leaves the accepted files where they are
    bad, none_written = _result(9.0, 5.0)
    monkeypatch.setattr(solve, "run", lambda *a, **k: bad)
    cameras.solve(cell, "down")
    assert not none_written and cell.L["vision"]["calibration"]["down_cam"]["px"] == 0.3
