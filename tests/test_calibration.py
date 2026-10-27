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
    assert 24 <= len(poses) <= 30 and poses[-1] == here
    arr = np.array(poses[:-1])
    assert len(set(arr[:, 2])) == 3 and len(set(arr[:, 5])) == 4
    assert arr[:, 0].min() < here[0] - 30 < here[0] + 30 < arr[:, 0].max()
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
