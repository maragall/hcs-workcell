"""
HALCON 12 camera model and file formats, as the vision box reads them.

area_scan_polynomial (CamParam.dat): a point (x, y, z) in camera coordinates projects to the
undistorted image plane u = Focus*x/z, v = Focus*y/z (metres), the lens distorts it to (u~, v~),
and the sensor maps that to pixels col = u~/Sx + Cx, row = v~/Sy + Cy (pixel centres at
integer coordinates, as in OpenCV).

HALCON writes the polynomial model from distorted to undistorted plane coordinates:
    u = u~ + u~(K1 r^2 + K2 r^4 + K3 r^6) + P1 (r^2 + 2 u~^2) + 2 P2 u~ v~
    v = v~ + v~(K1 r^2 + K2 r^4 + K3 r^6) + 2 P1 u~ v~ + P2 (r^2 + 2 v~^2),   r^2 = u~^2 + v~^2
so projecting means inverting that numerically (a few fixed-point steps). The file stores
Poly1..3 = K1..K3 (1/m^2, 1/m^4, 1/m^6) and Poly4..5 = P1, P2 * 1000.

area_scan_division (one Kappa): u = u~ / (1 + Kappa r^2); projecting inverts it in closed form.

Pose files (CamInTool.dat) are HALCON type 2 = "Rp+T, abg, point": p_tool = R p_cam + t with
R = Rz(gamma) Ry(beta) Rx(alpha), angles in degrees, t in metres. Confirmed against the
vendor's own CamInTool.dat in handeye.solve.
"""
from __future__ import annotations

import re
import time
from dataclasses import dataclass, field, replace
from typing import Optional, Tuple

import numpy as np

CRLF = "\r\n"


# ------------------------------------------------------------------ number formatting
def fmt(x: float) -> str:
    """%.15g with the three-digit exponent of the vendor's Windows build."""
    s = "%.15g" % x
    return re.sub(r"e([+-])(\d\d)$", r"e\g<1>0\2", s)


def stamp(t: Optional[float] = None) -> str:
    return "HALCON Version 12.0 --  " + time.asctime(time.localtime(t))


# ------------------------------------------------------------------ camera parameters
@dataclass(frozen=True)
class CamParam:
    focus: float
    sx: float
    sy: float
    cx: float
    cy: float
    width: int
    height: int
    model: str = "area_scan_polynomial"
    poly: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)   # file values: K1 K2 K3 P1*1000 P2*1000
    kappa: float = 0.0                                     # division model only
    reprojection_px: float = float("nan")
    written: str = field(default="", compare=False)        # the timestamp line, kept on read
    notes: str = field(default="", compare=False)          # which captures the fit used

    # pixels
    @property
    def fx(self) -> float:
        return self.focus / self.sx

    @property
    def fy(self) -> float:
        return self.focus / self.sy

    @property
    def K(self) -> np.ndarray:
        """Pinhole matrix in pixels (no distortion), for solvePnP on undistorted points."""
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    # ------------------------------------------------------------- the model
    def undistort_plane(self, ud: np.ndarray) -> np.ndarray:
        """Distorted plane coords (N, 2) m -> undistorted, HALCON's stated direction."""
        if self.model == "area_scan_division":
            r2 = (ud ** 2).sum(axis=1, keepdims=True)
            return ud / (1.0 + self.kappa * r2)
        k1, k2, k3, p1, p2 = self.poly[0], self.poly[1], self.poly[2], self.poly[3] / 1e3, self.poly[4] / 1e3
        u, v = ud[:, 0], ud[:, 1]
        r2 = u * u + v * v
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        uu = u * radial + p1 * (r2 + 2 * u * u) + 2 * p2 * u * v
        vv = v * radial + 2 * p1 * u * v + p2 * (r2 + 2 * v * v)
        return np.stack([uu, vv], axis=1)

    def distort_plane(self, un: np.ndarray, iters: int = 12) -> np.ndarray:
        """Undistorted plane coords -> distorted (inverse of undistort_plane)."""
        if self.model == "area_scan_division":
            r2 = (un ** 2).sum(axis=1, keepdims=True)
            disc = np.sqrt(np.maximum(1.0 - 4.0 * self.kappa * r2, 0.0))
            return 2.0 * un / (1.0 + disc)
        ud = un.copy()
        with np.errstate(over="ignore", invalid="ignore"):
            for _ in range(iters):
                ud = ud + (un - self.undistort_plane(ud))
        return np.where(np.isfinite(ud), ud, un)

    def project(self, pts_cam: np.ndarray) -> np.ndarray:
        """Camera-frame points (N, 3) m -> pixel (col, row) (N, 2)."""
        z = pts_cam[:, 2:3]
        un = self.focus * pts_cam[:, :2] / z
        ud = self.distort_plane(un)
        return np.stack([ud[:, 0] / self.sx + self.cx, ud[:, 1] / self.sy + self.cy], axis=1)

    def pixel_to_plane(self, px: np.ndarray) -> np.ndarray:
        """Pixels (N, 2) -> undistorted plane coords (N, 2) m."""
        ud = np.stack([(px[:, 0] - self.cx) * self.sx, (px[:, 1] - self.cy) * self.sy], axis=1)
        return self.undistort_plane(ud)

    def normalized(self, px: np.ndarray) -> np.ndarray:
        """Pixels -> ideal pinhole pixels with K (for cv2.solvePnP with zero distortion)."""
        un = self.pixel_to_plane(px)
        return np.stack([un[:, 0] / self.sx + self.cx, un[:, 1] / self.sy + self.cy], axis=1)

    # ------------------------------------------------------------- file format
    def write(self, path: str, when: Optional[str] = None) -> None:
        with open(path, "wb") as f:
            f.write(self.dumps(when).encode("ascii"))

    def dumps(self, when: Optional[str] = None) -> str:
        if self.model == "area_scan_polynomial":
            names = ["Focus", "Poly1", "Poly2", "Poly3", "Poly4", "Poly5", "Sx", "Sy", "Cx", "Cy",
                     "ImageWidth", "ImageHeight"]
            rows = [
                ("Focus", "foc", self.focus, "DOUBLE:0.0:", "Focal length of the lens"),
                ("Poly1", "poly1", self.poly[0], "DOUBLE::", "1st polynomial distortion coefficient"),
                ("Poly2", "poly2", self.poly[1], "DOUBLE::", "2nd polynomial distortion coefficient"),
                ("Poly3", "poly3", self.poly[2], "DOUBLE::", "3rd polynomial distortion coefficient"),
                ("Poly4", "poly4", self.poly[3], "DOUBLE::", "4th polynomial distortion coefficient * 1000"),
                ("Poly5", "poly5", self.poly[4], "DOUBLE::", "5th polynomial distortion coefficient * 1000"),
            ]
        elif self.model == "area_scan_division":
            names = ["Focus", "Kappa", "Sx", "Sy", "Cx", "Cy", "ImageWidth", "ImageHeight"]
            rows = [
                ("Focus", "foc", self.focus, "DOUBLE:0.0:", "Focal length of the lens"),
                ("Kappa", "kappa", self.kappa, "DOUBLE::", "Radial distortion coefficient"),
            ]
        else:
            raise ValueError(self.model)
        rows += [
            ("Sx", "sx", self.sx, "DOUBLE:0.0:", "Width of a cell on the sensor"),
            ("Sy", "sy", self.sy, "DOUBLE:0.0:", "Height of a cell on the sensor"),
            ("Cx", "cx", self.cx, "DOUBLE::", "X-coordinate of the image center"),
            ("Cy", "cy", self.cy, "DOUBLE::", "Y-coordinate of the image center"),
            ("ImageWidth", "imgw", self.width, "INT:1:32768", "Width of the images"),
            ("ImageHeight", "imgh", self.height, "INT:1:32768", "Height of the images"),
        ]
        out = list(_CAMPARAM_HEAD)
        out += ["#", "#  Camera : Parameter"] + [f"#    >  {n}" for n in names] + ["#"]
        out += ["#" * 78, "", "ParGroup: Camera: Parameter;", '\t"Internal camera parameters";', ""]
        for name, short, val, typ, desc in rows:
            v = str(val) if isinstance(val, int) else fmt(val)
            out += [f"{name}:{short}:\t{v};", f"\t{typ};", f'\t"{desc}";', ""]
        out += ["", "#", "# " + (when or stamp()), "#"]
        return CRLF.join(out) + CRLF

    @classmethod
    def read(cls, path: str) -> "CamParam":
        with open(path, "r", encoding="ascii", errors="replace") as f:
            text = f.read()
        vals = {m.group(1): float(m.group(2)) for m in re.finditer(r"^(\w+):\w+:\s*([-+0-9.eE]+);", text, re.M)}
        when = re.search(r"^# (HALCON Version .*?)\s*$", text, re.M)
        common = dict(focus=vals["Focus"], sx=vals["Sx"], sy=vals["Sy"], cx=vals["Cx"], cy=vals["Cy"],
                      width=int(vals["ImageWidth"]), height=int(vals["ImageHeight"]),
                      written=when.group(1) if when else "")
        if "Kappa" in vals:
            return cls(model="area_scan_division", kappa=vals["Kappa"], **common)
        return cls(model="area_scan_polynomial", poly=tuple(vals[f"Poly{i}"] for i in range(1, 6)), **common)

    def with_error(self, px: float) -> "CamParam":
        return replace(self, reprojection_px=px)


_CAMPARAM_HEAD = [
    "#", "# INTERNAL CAMERA PARAMETERS", "#", "",
    "# ===== generic parameter syntax ==========================================",
    "#",
    "# Syntax:      ParGroup: <pargroupname>;",
    "#                        <pargroup_description>;",
    "#",
    "#               <name> : <shortname> : <default_value>;",
    "#                        <type> : <lower_bound> : <upper_bound>;",
    "#                        <description>;",
    "#",
    "#                [ <type> ::= BOOL|XBOOL|INT|FLOAT|DOUBLE|STRING ]",
    "#",
    "# =========================================================================",
    "", "", "", "#" * 78,
]


# ------------------------------------------------------------------ poses
def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(b):
    c, s = np.cos(b), np.sin(b)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(g):
    c, s = np.cos(g), np.sin(g)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def abg_to_matrix(alpha_deg, beta_deg, gamma_deg) -> np.ndarray:
    """HALCON 'abg' point view: R = Rz(gamma) Ry(beta) Rx(alpha)."""
    a, b, g = np.radians([alpha_deg, beta_deg, gamma_deg])
    return rot_z(g) @ rot_y(b) @ rot_x(a)


def matrix_to_abg(R: np.ndarray) -> Tuple[float, float, float]:
    """Inverse of abg_to_matrix, angles in [0, 360) as HALCON writes them."""
    beta = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if abs(np.cos(beta)) < 1e-12:
        alpha, gamma = np.arctan2(R[0, 1], R[1, 1]), 0.0
    else:
        alpha = np.arctan2(R[2, 1], R[2, 2])
        gamma = np.arctan2(R[1, 0], R[0, 0])
    return tuple(float(np.degrees(x) % 360.0) for x in (alpha, beta, gamma))


@dataclass(frozen=True)
class Pose:
    """A HALCON pose file: f 2, r (deg), t (m)."""
    r: Tuple[float, float, float]
    t: Tuple[float, float, float]
    written: str = field(default="", compare=False)

    @property
    def matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = abg_to_matrix(*self.r)
        T[:3, 3] = self.t
        return T

    @classmethod
    def from_matrix(cls, T: np.ndarray) -> "Pose":
        return cls(matrix_to_abg(T[:3, :3]), tuple(float(v) for v in T[:3, 3]))

    def dumps(self, when: Optional[str] = None) -> str:
        lines = ["# ", "# 3D POSE PARAMETERS: rotation and translation", "# ", "",
                 "# Used representation type:", "f 2", "",
                 "# Rotation angles [deg] or Rodriguez vector:", "r " + " ".join(fmt(v) for v in self.r), "",
                 "# Translation vector (x y z [m]):", "t " + " ".join(fmt(v) for v in self.t), "",
                 "# ", "# " + (when or stamp()), "", "# "]
        return CRLF.join(lines) + CRLF

    def write(self, path: str, when: Optional[str] = None) -> None:
        with open(path, "wb") as f:
            f.write(self.dumps(when).encode("ascii"))

    @classmethod
    def read(cls, path: str) -> "Pose":
        with open(path, "r", encoding="ascii", errors="replace") as f:
            text = f.read()
        kind = re.search(r"^f\s+(\d+)", text, re.M)
        if not kind or kind.group(1) != "2":
            raise ValueError(f"{path}: pose type {kind.group(1) if kind else '?'}, only type 2 is handled")
        r = tuple(float(v) for v in re.search(r"^r\s+(.*)$", text, re.M).group(1).split())
        t = tuple(float(v) for v in re.search(r"^t\s+(.*)$", text, re.M).group(1).split())
        when = re.search(r"^# (HALCON Version .*?)\s*$", text, re.M)
        return cls(r, t, when.group(1) if when else "")
