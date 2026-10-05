"""
GALERIA - The Living Map
Kalman filters for the robot position seen from the Outside Network Area.

Origin: ekf.py (GALERIA team). PositionEKF is that filter, unchanged in
behaviour (comments in English): the prediction step moves the state by the
odometry displacement and GROWS the covariance by the process noise (wheel slip,
gyro drift); the correction step pulls the state towards a position measurement
and SHRINKS the covariance.

v9 gives it a real, independent measurement. Inside the building there is no
GPS, but the three ONA gateways measure their distance to the robot (two-way
ranging). RangeEKF is the same filter on the (east, north) plane, with one
correction per measured range: a nonlinear measurement h(x) = |x - gateway|,
linearised at every step (hence "extended" Kalman filter). This is "tight
coupling": even a single range corrects the robot along its line of sight, so
the filter keeps working when one or two gateways are not heard.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# Position EKF (state = [x, y, z] in the ENU "world" frame) - original
# ---------------------------------------------------------------------------
class PositionEKF:
    def __init__(self, x0: np.ndarray, P0: np.ndarray):
        self.x = np.asarray(x0, dtype=float).copy()   # estimated state
        self.P = np.asarray(P0, dtype=float).copy()   # covariance

    def predict(self, delta_odom: np.ndarray, odom_noise_std: np.ndarray) -> None:
        """
        Prediction: move the state by the displacement measured by odometry
        (wheels, corrected by the IMU) and GROW the covariance by the process
        noise of this step (wheel slip + gyro drift).

        delta_odom     : displacement since the previous step (m)
        odom_noise_std : standard deviation of the noise on it (m), per axis
        """
        # Simple linear model: x_k = x_{k-1} + delta_odom  (F = I, B = I)
        self.x = self.x + np.asarray(delta_odom, dtype=float)
        Q = np.diag(np.asarray(odom_noise_std, dtype=float) ** 2)   # process noise of this step
        self.P = self.P + Q                                          # F P F^T + Q, with F = I

    def update(self, z: np.ndarray, R: np.ndarray) -> None:
        """
        Correction: a position measurement z with covariance R. Pulls x
        towards z and SHRINKS P.
        """
        n = self.x.shape[0]
        H = np.eye(n)                        # the position is measured directly
        y = np.asarray(z, dtype=float) - H @ self.x   # innovation
        S = H @ self.P @ H.T + R             # innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(n) - K @ H) @ self.P

    @property
    def err_m(self) -> float:
        """Equivalent 1-sigma uncertainty (square root of the trace of P)."""
        return float(np.sqrt(np.trace(self.P)))


# ---------------------------------------------------------------------------
# v9: range EKF on the ground plane
# ---------------------------------------------------------------------------
@dataclass
class RangeUpdate:
    gw: str
    accepted: bool
    innovation_m: float
    z_score: float
    weight: float


class RangeEKF(PositionEKF):
    """State [east, north] (m, ENU at the calibration origin); robot height is known.

    predict()        : PositionEKF.predict, fed with the robot's own displacement
                       (its SLAM/odometry pose difference, rotated into ENU)
    update_range()   : one gateway range, nonlinear, robust (Huber) and gated
    update()         : PositionEKF.update, a whole position fix (loose coupling)
    """

    def __init__(self, e: float, n: float, sigma0_m: float, huber_k: float = 2.0, reject_z: float = 5.0):
        super().__init__(np.array([e, n]), np.eye(2) * sigma0_m ** 2)
        self.huber_k = huber_k      # beyond k sigma a range counts less (walls make ranges long)
        self.reject_z = reject_z    # beyond this many sigmas a range is ignored (outlier)
        self.updates = 0
        self.rejected = 0
        self.nis = deque(maxlen=12)  # recent normalised innovations squared (expected mean: 1)

    @property
    def adapt(self) -> float:
        """Adaptive process noise: when the ranges keep disagreeing with the prediction (mean
        NIS well above 1), the robot's own displacement is not what it claims (wheel slip, SLAM
        drift): trust it less, up to 5x the nominal noise."""
        if len(self.nis) < 4:
            return 1.0
        return float(min(5.0, max(1.0, math.sqrt(sum(self.nis) / len(self.nis)))))

    def predict_move(self, d_enu, dist_m: float, noise_per_m: float = 0.05, floor_m: float = 0.02) -> None:
        """Predict with a displacement d_enu (2,), noise growing with the distance driven."""
        s = (floor_m + noise_per_m * max(0.0, dist_m)) * self.adapt
        self.predict(np.asarray(d_enu, dtype=float)[:2], np.array([s, s]))

    def update_range(self, gw: str, gw_enu, r_meas: float, sigma_m: float, robot_height: float = 0.0
                     ) -> RangeUpdate:
        g = np.asarray(gw_enu, dtype=float)
        dz = robot_height - (g[2] if g.shape[0] > 2 else 0.0)
        d = self.x - g[:2]
        h = math.sqrt(float(d @ d) + dz * dz)
        if h < 1e-6:
            return RangeUpdate(gw, False, 0.0, 0.0, 0.0)
        H = (d / h).reshape(1, 2)
        y = float(r_meas) - h
        R = sigma_m ** 2
        S = (H @ self.P @ H.T).item() + R
        z = y / math.sqrt(S)
        self.nis.append(min(z * z, 25.0))
        if abs(z) > self.reject_z:
            self.rejected += 1
            return RangeUpdate(gw, False, y, z, 0.0)
        w = 1.0 if abs(z) <= self.huber_k else self.huber_k / abs(z)   # Huber: long tail counts less
        R_eff = R / w
        S = (H @ self.P @ H.T).item() + R_eff
        K = (self.P @ H.T) / S                    # (2, 1)
        self.x = self.x + (K * y).ravel()
        I_KH = np.eye(2) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + (K @ K.T) * R_eff   # Joseph form: stays symmetric positive
        self.updates += 1
        return RangeUpdate(gw, True, y, z, w)

    @property
    def sigma_m(self) -> float:
        """1-sigma radius of the largest error axis."""
        return float(math.sqrt(max(np.linalg.eigvalsh(self.P))))

    def ellipse(self) -> dict:
        """1-sigma error ellipse: semi-axes (m) and orientation (rad from east)."""
        vals, vecs = np.linalg.eigh(self.P)
        a, b = math.sqrt(max(vals[1], 0.0)), math.sqrt(max(vals[0], 0.0))
        ang = math.atan2(vecs[1, 1], vecs[0, 1])
        return {'a_m': round(a, 3), 'b_m': round(b, 3), 'angle_rad': round(ang, 4)}

    def mahalanobis_to(self, z, Rz) -> float:
        """Distance of a position z (2,) with covariance Rz from the estimate, in sigmas."""
        dv = np.asarray(z, dtype=float)[:2] - self.x
        S = self.P + np.asarray(Rz, dtype=float)
        return float(math.sqrt(max(0.0, dv @ np.linalg.solve(S, dv))))


def chi2_threshold(dof: int, p_fa: float = 0.01) -> float:
    """Chi-square quantile 1 - p_fa for small dof (table: 1 %, 0.1 %)."""
    table = {0.01: [0.0, 6.635, 9.210, 11.345, 13.277, 15.086, 16.812],
             0.001: [0.0, 10.828, 13.816, 16.266, 18.467, 20.515, 22.458]}
    t = table.get(p_fa, table[0.01])
    return t[min(dof, len(t) - 1)] if dof > 0 else math.inf


def fix_covariance_ok(P: Optional[np.ndarray], max_sigma_m: float) -> bool:
    return P is not None and math.sqrt(max(np.linalg.eigvalsh(P))) <= max_sigma_m
