"""The calibration package on the vendor's captures and on the fakes. No hardware, no network."""
from __future__ import annotations

import glob
import json
import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from calibration import capture, halcon, handeye, intrinsics, plate, teach_free, upload, verify  # noqa: E402
from devices.fake_arm import SimRobot  # noqa: E402
from devices.vision_box import ArucoPose  # noqa: E402

VENDOR = os.path.join(ROOT, "vendor_data")
INTR = os.path.join(VENDOR, "CalibrationData", "intrinsic")
HAND = os.path.join(VENDOR, "CalibrationData", "HandEye")
PLAN = os.path.join(VENDOR, "CalibrationPlans", "scan_camera.json")


# ------------------------------------------------------------------ plate
def test_plate_detection_orders_and_orients_the_vendor_image():
    det = plate.detect(os.path.join(INTR, "intrinsic_008.png"))
    assert det.image_points.shape == (49, 2) and det.object_points.shape == (49, 3)
    assert np.allclose(det.object_points[0, :2], [-3 * plate.PLATE_PITCH_M] * 2)     # first mark: min x, min y
    pred = cv2.perspectiveTransform(det.object_points[:, :2].reshape(-1, 1, 2).astype(np.float32),
                                    det.homography).reshape(-1, 2)
    assert np.linalg.norm(pred - det.image_points, axis=1).max() < 2.0


def test_plate_detection_under_flare():
    det = plate.detect(os.path.join(HAND, "handeye_014.png"))
    assert det.image_points.shape == (49, 2)


def test_plate_not_found_raises():
    with pytest.raises(plate.PlateNotFound):
        plate.detect(np.full((400, 600), 128, np.uint8))


# ------------------------------------------------------------------ file formats
def test_camparam_round_trip_is_byte_identical():
    path = os.path.join(INTR, "CamParam.dat")
    cam = halcon.CamParam.read(path)
    with open(path, "rb") as f:
        assert cam.dumps(cam.written).encode() == f.read()


def test_pose_round_trip_is_byte_identical():
    path = os.path.join(HAND, "CamInTool.dat")
    pose = halcon.Pose.read(path)
    with open(path, "rb") as f:
        assert pose.dumps(pose.written).encode() == f.read()
    assert np.allclose(halcon.matrix_to_abg(pose.matrix[:3, :3]), pose.r, atol=1e-9)


def test_division_model_file_has_kappa_group(tmp_path):
    cam = halcon.CamParam(0.003, 1.12e-6, 1.12e-6, 1640, 1232, 3280, 2464, "area_scan_division", kappa=-100.0)
    p = tmp_path / "CamParam.dat"
    cam.write(str(p))
    text = p.read_bytes().decode()
    assert "Kappa:kappa:\t-100;" in text and "\r\n" in text and "Poly1" not in text
    assert halcon.CamParam.read(str(p)) == cam


# ------------------------------------------------------------------ the maths, against the vendor
@pytest.fixture(scope="module")
def our_camparam():
    with open(PLAN) as f:
        n = len(json.load(f)["Offsets"])
    return intrinsics.solve(INTR, limit=n)


def test_intrinsics_match_the_vendor(our_camparam):
    vendor = halcon.CamParam.read(os.path.join(INTR, "CamParam.dat"))
    cam = our_camparam
    for name in ("focus", "cx", "cy"):
        assert abs(getattr(cam, name) - getattr(vendor, name)) / getattr(vendor, name) < 0.005, name
    assert cam.reprojection_px <= 0.162
    assert np.sign(cam.poly) .tolist() == np.sign(vendor.poly).tolist()


def test_handeye_matches_the_vendor(our_camparam):
    vendor = halcon.Pose.read(os.path.join(HAND, "CamInTool.dat"))
    res = handeye.solve(HAND, our_camparam)
    drot = np.degrees(np.linalg.norm(cv2.Rodrigues(res.matrix[:3, :3].T @ vendor.matrix[:3, :3])[0]))
    assert drot < 0.5
    assert np.abs((np.array(res.pose.t) - np.array(vendor.t)) * 1e3).max() < 1.0
    assert res.unobservable == ("tool z",)
    assert set(res.dropped) == {f"handeye_{i:03d}.png" for i in (14, 15, 16, 17)}
    assert res.mean_error_m < 0.5e-3


# ------------------------------------------------------------------ capture on the fakes
class FakeBox:
    """Vendor frames back from the folder, marker readings fixed in the robot frame, teach API recorded."""

    def __init__(self, frames=None):
        self.frames = frames or sorted(glob.glob(os.path.join(HAND, "handeye_*.png")))[:3]
        self.n = 0
        self.calls = []
        self.stored = []

    def frame_full(self, camera, led=0):
        img = cv2.imread(self.frames[self.n % len(self.frames)])
        self.n += 1
        return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()

    def aruco_pose(self, camera, marker_id=1, marker_size_m=None, led=0):
        return ArucoPose([0, 0, 0, 0], [0] * 6, [300.0, -50.0, 20.0, 0, 0, 0], "robot")

    def positions(self, workspace="autoteach", ptype="all"):
        return [dict(type=t, name=n) for t, n in self.stored]

    def delete_position(self, name, ptype, workspace="autoteach"):
        self.calls.append(("delete", name, ptype))
        self.stored.remove((ptype, name))

    def add_teach(self, name, pose, pick="wide", workspace="autoteach"):
        self.calls.append(("teach", name, list(pose), pick, workspace))
        self.stored.append(("teach", name))

    def add_trigger(self, name, pose, camera, marker="aruco", caltab_mm=150, pick="wide", workspace="autoteach", led=0):
        self.calls.append(("trigger", name, list(pose), camera, marker, caltab_mm, pick, workspace, led))
        self.stored.append(("trigger", name))

    def add_relation(self, trigger_name, teach_name, relation_name, pick="wide", workspace="autoteach"):
        self.calls.append(("relation", trigger_name, teach_name, relation_name, pick, workspace))


def test_plan_offsets_compose_in_the_initial_frame():
    plan = capture.load_plan(PLAN)
    for i, off in enumerate(plan["Offsets"], start=1):
        got = capture.compose(plan["InitialToolInBase"], off)
        want = handeye.read_pose_file(os.path.join(HAND, f"pose_{i:03d}.txt"))
        assert abs(got[2] - want[2]) < 1.0 and abs(got[5] - want[5]) < 0.5          # z, rz add
        assert np.linalg.norm(np.array(got[:2]) - np.array(want[:2])) < 15.0         # plan re-saved off the start


def test_capture_saves_like_the_vendor(tmp_path):
    plan = capture.load_plan(PLAN)
    plan["Offsets"] = plan["Offsets"][:3]
    robot = SimRobot([0, 0, 0, 0, 0, 0])
    box = FakeBox()
    out = str(tmp_path / "HandEye")
    files = capture.run_plan(plan, "scan_camera", robot, box, out)
    assert [os.path.basename(f) for f in files] == ["handeye_001.png", "handeye_002.png", "handeye_003.png"]
    assert robot._speed == capture.DEFAULT_SPEED and len(robot.trace) == 3
    with open(os.path.join(out, "pose_002.txt"), "rb") as f:
        text = f.read().decode()
    assert text.count(",") == 5 and "\n" not in text
    assert [round(float(v), 3) for v in text.split(",")] == [round(v, 3) for v in robot.trace[1][1]]
    assert cv2.imread(files[0]).shape == (2464, 3280, 3)
    assert plate.detect(files[0]).image_points.shape == (49, 2)
    intr = capture.run_plan(plan, "scan_camera", robot, box, str(tmp_path / "intrinsic"), kind="intrinsic")
    assert os.path.basename(intr[0]) == "intrinsic_001.png" and not glob.glob(str(tmp_path / "intrinsic" / "pose_*"))


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


# ------------------------------------------------------------------ verify and teach-free on the fake box
def test_verify_reports_the_marker_drift_between_two_poses():
    robot = SimRobot([0, 0, 0, 0, 0, 0])
    assert verify.check(FakeBox(), "auto_teach_camera", robot, [100, 0, 50, 0, 0, 0], [120, 0, 50, 0, 0, 0]) == 0.0
    assert len(robot.trace) == 2


def test_teach_free_add_stores_teach_trigger_relation():
    box = FakeBox()
    box.stored.append(("teach", "slot-teach"))
    names = teach_free.add(box, "long", [1, 2, 3, 0, 0, 0], [4, 5, 6, 0, 0, 0], "auto_teach_camera", "slot")
    assert names == {"teach": "slot-teach", "trigger": "slot-trigger", "relation": "slot-rel"}
    kinds = [c[0] for c in box.calls]
    assert kinds == ["delete", "teach", "trigger", "relation"]
    assert box.calls[2][6] == "long" and box.calls[3][1:4] == ("slot-trigger", "slot-teach", "slot-rel")
    with pytest.raises(ValueError):
        teach_free.add(box, "narrow", [0] * 6, [0] * 6, "auto_teach_camera", "x")
