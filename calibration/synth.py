"""
Synthetic captures: the marker rendered through a known CamParam and CamInTool at a given arm
pose, with pixel noise. For validating the solve offline against the vendor's files
(validate.py, tests) and for a fake box that answers frame_full like the real one.
"""
from __future__ import annotations

from typing import Sequence

import cv2
import numpy as np

from calibration import halcon, marker
from calibration.solve import _inv, tool_matrix

QUIET_ZONE = 0.5         # white border around the marker, in marker sizes
BACKGROUND = 120         # grey of the bench


class Scene:
    """A camera (CamParam, CamInTool) on the arm and the marker fixed in the base frame."""

    def __init__(self, cam: halcon.CamParam, camintool: halcon.Pose, marker_in_base: np.ndarray, size_m: float,
                 noise_px: float = 0.3, seed: int = 0):
        self.cam, self.T_tc, self.T_bm, self.size = cam, camintool.matrix, marker_in_base, size_m
        self.noise, self.rng = noise_px, np.random.default_rng(seed)
        bits = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(marker.DICTIONARY), marker.MARKER_ID, 200)
        q = int(200 * QUIET_ZONE)
        self.tile = cv2.copyMakeBorder(bits, q, q, q, q, cv2.BORDER_CONSTANT, value=255)
        self.q = q

    def corners(self, pose_mm_deg: Sequence[float]) -> np.ndarray:
        """Where the marker's corners land in the image at this arm pose (4, 2)."""
        P = _inv(self.T_tc) @ _inv(tool_matrix(pose_mm_deg)) @ self.T_bm
        pts = marker.object_points(self.size) @ P[:3, :3].T + P[:3, 3]
        if (pts[:, 2] <= 0).any():
            raise ValueError("marker behind the camera")
        return self.cam.project(pts)

    def render(self, pose_mm_deg: Sequence[float]) -> np.ndarray:
        """Grey image (height, width) uint8 of the marker on its white tab, plus noise."""
        c = self.corners(pose_mm_deg).astype(np.float32)
        q, n = self.q - 0.5, 200                      # the bits' outer edges, pixel centres at integers
        src = np.array([[q, q], [q + n, q], [q + n, q + n], [q, q + n]], np.float32)
        H = cv2.getPerspectiveTransform(src, c)
        canvas = cv2.warpPerspective(self.tile, H, (self.cam.width, self.cam.height), flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=BACKGROUND)
        canvas = cv2.GaussianBlur(canvas, (0, 0), 1.0)
        img = canvas.astype(np.float32) + self.rng.normal(0.0, self.noise * 8.0, canvas.shape)
        return np.clip(img, 0, 255).astype(np.uint8)


def facing(cam_pose: Sequence[float], camintool: halcon.Pose, distance_m: float, tilt_deg: float = 8.0) -> np.ndarray:
    """A marker-in-base that sits `distance_m` in front of the camera at this arm pose, slightly tilted."""
    T_cm = np.eye(4)
    T_cm[:3, :3] = halcon.rot_x(np.radians(tilt_deg)) @ np.diag([1.0, -1.0, -1.0])    # z out of the tab, toward the lens
    T_cm[:3, 3] = [0.0, 0.0, distance_m]
    return tool_matrix(cam_pose) @ camintool.matrix @ T_cm


class FakeBox:
    """frame_full(camera) renders the scene at the robot's current pose, as the box would see it."""

    def __init__(self, scene: Scene, robot, quality: int = 95):
        self.scene, self.robot, self.quality = scene, robot, quality

    def frame_full(self, camera: str, led: int = 0) -> bytes:
        img = self.scene.render(self.robot.pose())
        return cv2.imencode(".png", img)[1].tobytes()

    def frame(self, camera: str, width=1920, height=1080, quality=85, exposure_us=None, led=0) -> bytes:
        img = cv2.resize(self.scene.render(self.robot.pose()), (width, height))
        return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()
