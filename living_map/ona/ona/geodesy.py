"""
GALERIA - The Living Map
Frame translation: private robot coordinates (SLAM 'map' frame) <-> real-world GPS (WGS84).

Origin: trans.py (GALERIA team, "Partie 5"). The WGS84 / ECEF / ENU conversions, the
Umeyama alignment and the LocalToGpsCalibrator are that code, with the comments
translated to English. Additions for the ONA are marked "v9" in comments.

Principle:
    1. 3-4 anchor points are measured in both frames at the entrance, where GPS
       works: the robot's SLAM position (map frame, metres) and a GPS fix.
    2. Umeyama finds the rotation R and translation t (scale fixed at 1: SLAM is
       metric) that best map the robot points onto the GPS points (local ENU).
    3. Any new local position (a beacon, a robot) is mapped with (R, t) into
       ENU, then ENU -> ECEF -> WGS84.
    4. v9: inside the building the three ONA gateways measure the robot's
       position by ranging (multilateration.py). Each good fix is a new
       (map, ENU) pair, so the same alignment can be re-checked during the
       mission without GPS.

Dependencies: numpy only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# WGS84 constants
# ---------------------------------------------------------------------------
WGS84_A = 6378137.0                 # semi-major axis (m)
WGS84_F = 1 / 298.257223563         # flattening
WGS84_E2 = WGS84_F * (2 - WGS84_F)  # first eccentricity squared


@dataclass
class AnchorPoint:
    """A point known in both frames."""
    local_xyz: np.ndarray   # position in the robot frame (map), metres
    lat: float              # measured GPS latitude (degrees)
    lon: float              # measured GPS longitude (degrees)
    alt: float = 0.0        # measured GPS altitude (m)
    weight: float = 1.0     # v9: confidence of this pair (1 / variance, relative)


# ---------------------------------------------------------------------------
# Geodetic conversions: WGS84 (lat/lon/alt) <-> ECEF <-> local ENU
# ---------------------------------------------------------------------------
def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> np.ndarray:
    """Latitude / longitude / altitude (WGS84) -> ECEF cartesian coordinates."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
    x = (n + alt) * cos_lat * cos_lon
    y = (n + alt) * cos_lat * sin_lon
    z = (n * (1 - WGS84_E2) + alt) * sin_lat
    return np.array([x, y, z])


def ecef_to_geodetic(ecef: np.ndarray) -> tuple:
    """ECEF -> (lat_deg, lon_deg, alt_m). Iterative (Bowring) method."""
    x, y, z = ecef
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    alt = 0.0
    for _ in range(5):  # converges fast, 5 iterations are plenty
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - WGS84_E2 * n / (n + alt)))
    return math.degrees(lat), math.degrees(lon), alt


def _enu_matrix(ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    return np.array([
        [-sin_lon, cos_lon, 0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
    ])


def ecef_to_enu(ecef: np.ndarray, ref_ecef: np.ndarray, ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    """ECEF vector -> local ENU centred on a reference point."""
    return _enu_matrix(ref_lat_deg, ref_lon_deg) @ (np.asarray(ecef) - ref_ecef)


def enu_to_ecef(enu: np.ndarray, ref_ecef: np.ndarray, ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    """Local ENU -> ECEF, the inverse of ecef_to_enu (the matrix is orthogonal)."""
    return ref_ecef + _enu_matrix(ref_lat_deg, ref_lon_deg).T @ np.asarray(enu)


class EnuFrame:
    """v9: a local East-North-Up frame at a reference point, with both directions."""

    def __init__(self, lat: float, lon: float, alt: float = 0.0):
        self.lat, self.lon, self.alt = float(lat), float(lon), float(alt)
        self.ref_ecef = geodetic_to_ecef(self.lat, self.lon, self.alt)

    def to_enu(self, lat: float, lon: float, alt: Optional[float] = None) -> np.ndarray:
        return ecef_to_enu(geodetic_to_ecef(lat, lon, self.alt if alt is None else alt), self.ref_ecef, self.lat,
                           self.lon)

    def to_geodetic(self, enu) -> tuple:
        e = np.asarray(enu, dtype=float)
        if e.shape[0] == 2:
            e = np.array([e[0], e[1], 0.0])
        return ecef_to_geodetic(enu_to_ecef(e, self.ref_ecef, self.lat, self.lon))


# ---------------------------------------------------------------------------
# Umeyama alignment: robot frame (map) -> local ENU frame ("world")
# ---------------------------------------------------------------------------
def umeyama_alignment(src: np.ndarray, dst: np.ndarray, estimate_scale: bool = False,
                      weights: Optional[np.ndarray] = None) -> tuple:
    """
    Find (R, t, s) minimising  sum_i w_i || dst_i - (s R src_i + t) ||^2.

    src, dst : arrays (N, 3), N >= 3.
    estimate_scale : False here, SLAM already gives the true scale.
    weights : v9, optional per-pair weights (default: all 1, the original algorithm).

    Returns (R, t, s).
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    assert src.shape == dst.shape and src.shape[0] >= 3
    w = np.ones(src.shape[0]) if weights is None else np.asarray(weights, dtype=float)
    w = w / w.sum()

    mu_src = (w[:, None] * src).sum(axis=0)
    mu_dst = (w[:, None] * dst).sum(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    cov = (dst_c * w[:, None]).T @ src_c
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt

    if estimate_scale:
        var_src = (w * (src_c ** 2).sum(axis=1)).sum()
        s = np.trace(np.diag(D) @ S) / var_src
    else:
        s = 1.0
    t = mu_dst - s * R @ mu_src
    return R, t, s


def planar_alignment(src: np.ndarray, dst: np.ndarray, weights: Optional[np.ndarray] = None) -> tuple:
    """v9: Umeyama restricted to a rotation about the vertical axis (yaw) + a 3D translation.

    A ground robot's SLAM map is level, so only the yaw between map and ENU is
    unknown. Fitting a full 3D rotation to GPS anchors lets GPS altitude noise
    (metres) tilt the whole frame; this variant cannot. Returns (R, t, s=1)."""
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    w = np.ones(src.shape[0]) if weights is None else np.asarray(weights, dtype=float)
    w = w / w.sum()
    mu_s = (w[:, None] * src).sum(axis=0)
    mu_d = (w[:, None] * dst).sum(axis=0)
    a = src[:, :2] - mu_s[:2]
    b = dst[:, :2] - mu_d[:2]
    num = (w * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])).sum()
    den = (w * (a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])).sum()
    yaw = math.atan2(num, den)
    c, s_ = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    t = mu_d - R @ mu_s
    return R, t, 1.0


# ---------------------------------------------------------------------------
# Full calibrator: local (map) -> world (ENU) -> GPS
# ---------------------------------------------------------------------------
class LocalToGpsCalibrator:
    """
    Calibrated once at start-up (3-4 anchor points at the entrance), then
    re-checked during the mission with every good gateway fix.
    """

    def __init__(self, anchors: Sequence[AnchorPoint], planar: bool = False, drift_coeff: float = 0.02,
                 err_cap_m: float = 5.0):
        self.planar = planar            # v9: yaw-only rotation (see planar_alignment)
        self.drift_coeff = drift_coeff  # m of error per m driven since the last anchor (to calibrate in the field)
        self.err_cap_m = err_cap_m
        self.recalibrate(anchors)

    def recalibrate(self, anchors: Sequence[AnchorPoint]) -> None:
        if len(anchors) < 3:
            raise ValueError('Umeyama needs at least 3 anchor points.')
        # ENU reference = the first anchor (origin of the "world" frame)
        self.anchors = list(anchors)
        self.ref_anchor = anchors[0]
        self.ref_ecef = geodetic_to_ecef(self.ref_anchor.lat, self.ref_anchor.lon, self.ref_anchor.alt)

        src = np.array([np.asarray(a.local_xyz, dtype=float) for a in anchors])   # map frame (robot)
        # v9: collinear anchors cannot fix the rotation
        sv = np.linalg.svd(src[:, :2] - src[:, :2].mean(axis=0), compute_uv=False)
        if sv[0] < 1e-6 or sv[1] < 1e-3 * sv[0]:
            raise ValueError('anchor points are (almost) on one line: the rotation is undetermined')
        dst_enu = np.array([ecef_to_enu(geodetic_to_ecef(a.lat, a.lon, a.alt), self.ref_ecef,
                                        self.ref_anchor.lat, self.ref_anchor.lon) for a in anchors])
        w = np.array([a.weight for a in anchors], dtype=float)
        if self.planar:
            self.R, self.t, self.s = planar_alignment(src, dst_enu, w)
        else:
            self.R, self.t, self.s = umeyama_alignment(src, dst_enu, estimate_scale=False, weights=w)

        # RMS residual -> first estimate of the calibration error
        pred = (self.s * (src @ self.R.T)) + self.t
        self.residuals = np.linalg.norm((pred - dst_enu)[:, :2], axis=1)
        self.calib_rms_m = float(np.sqrt((self.residuals ** 2).mean()))

    # ------------------------------------------------------------------ v9 helpers
    @classmethod
    def from_anchor_yaw(cls, lat: float, lon: float, alt: float = 0.0, map_yaw_deg: float = 0.0,
                        spread_m: float = 5.0, **kw) -> 'LocalToGpsCalibrator':
        """The Writer's simple anchor (one GPS point + the compass direction of map +x) as 4
        exact entrance markers at (0,0) (s,0) (0,s) (s,s), like the original test setup."""
        frame = EnuFrame(lat, lon, alt)
        yaw = math.radians(map_yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        anchors = []
        for (x, y) in ((0, 0), (spread_m, 0), (0, spread_m), (spread_m, spread_m)):
            e, n = c * x - s * y, s * x + c * y
            la, lo, al = frame.to_geodetic([e, n, 0.0])
            anchors.append(AnchorPoint(np.array([x, y, 0.0]), la, lo, al))
        return cls(anchors, **kw)

    @property
    def yaw_rad(self) -> float:
        """Rotation map -> ENU about the vertical (map +x = compass east rotated by this)."""
        return math.atan2(self.R[1, 0], self.R[0, 0])

    @property
    def enu(self) -> EnuFrame:
        f = EnuFrame(self.ref_anchor.lat, self.ref_anchor.lon, self.ref_anchor.alt)
        return f

    def local_to_enu(self, p_map) -> np.ndarray:
        p = np.asarray(p_map, dtype=float)
        if p.shape[0] == 2:
            p = np.array([p[0], p[1], 0.0])
        return self.s * (self.R @ p) + self.t

    def enu_to_local(self, p_enu) -> np.ndarray:
        p = np.asarray(p_enu, dtype=float)
        if p.shape[0] == 2:
            p = np.array([p[0], p[1], self.t[2]])
        return self.R.T @ (p - self.t) / self.s

    def gps_to_local(self, lat: float, lon: float, alt: Optional[float] = None) -> np.ndarray:
        enu = ecef_to_enu(geodetic_to_ecef(lat, lon, self.ref_anchor.alt if alt is None else alt), self.ref_ecef,
                          self.ref_anchor.lat, self.ref_anchor.lon)
        return self.enu_to_local(enu)

    def enu_to_gps(self, p_enu) -> tuple:
        p = np.asarray(p_enu, dtype=float)
        if p.shape[0] == 2:
            p = np.array([p[0], p[1], 0.0])
        return ecef_to_geodetic(enu_to_ecef(p, self.ref_ecef, self.ref_anchor.lat, self.ref_anchor.lon))

    def gps_to_enu(self, lat: float, lon: float, alt: Optional[float] = None) -> np.ndarray:
        return ecef_to_enu(geodetic_to_ecef(lat, lon, self.ref_anchor.alt if alt is None else alt), self.ref_ecef,
                           self.ref_anchor.lat, self.ref_anchor.lon)

    def local_to_gps(self, p_map: np.ndarray, dist_since_last_anchor_m: float = 0.0, digits: int = 7) -> dict:
        """
        p_map : position (x, y, z) in the robot's SLAM 'map' frame.
        dist_since_last_anchor_m : distance driven since the last re-anchoring,
            used for the growing error (wheel + gyro drift) until a new anchor.

        Returns {lat, lon, alt, err_m}, ready for the beacon message ("gps": {...}).
        (v9: 7 decimals by default = the 1.1 cm resolution of an LMB2 record; the
        original rounded to 6.)
        """
        p_world = self.local_to_enu(p_map)              # world frame (ENU)
        ecef = enu_to_ecef(p_world, self.ref_ecef, self.ref_anchor.lat, self.ref_anchor.lon)
        lat, lon, alt = ecef_to_geodetic(ecef)
        # Simple error model: calibration error + linear drift with the
        # distance driven since the last anchor (coefficient to calibrate in the field)
        err_m = self.calib_rms_m + self.drift_coeff * dist_since_last_anchor_m
        err_m = min(err_m, self.err_cap_m)   # upper bound in a beaconed environment
        return {'lat': round(lat, digits), 'lon': round(lon, digits), 'alt': round(alt, 2), 'err_m': round(err_m, 2)}


def check_against(cal_a: LocalToGpsCalibrator, cal_b: LocalToGpsCalibrator, pts_local) -> dict:
    """v9: how far two calibrations disagree over a set of map points (metres, degrees)."""
    d = [float(np.linalg.norm((cal_a.local_to_enu(p) - cal_b.local_to_enu(p))[:2])) for p in pts_local]
    dyaw = math.degrees((cal_a.yaw_rad - cal_b.yaw_rad + math.pi) % (2 * math.pi) - math.pi)
    return {'max_m': max(d) if d else 0.0, 'mean_m': sum(d) / len(d) if d else 0.0, 'dyaw_deg': dyaw}
