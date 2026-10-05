"""
GALERIA - The Living Map
Test EKF : fusion odométrie/IMU (prédiction) + recalage GPS aux balises (correction)

Utilise EXACTEMENT les mêmes ancrages et la même balise que le test Umeyama
précédent, pour pouvoir comparer directement :

    Umeyama seul (formule fixe)      -> err_m = 1.22 m
    EKF (covariance réelle propagée) -> err_m = ?

Différence clé avec l'approche précédente :
    - Avant : err_m = calib_rms + 0.02 * distance   (formule à la main)
    - Ici   : l'incertitude est une VRAIE covariance qui grandit à chaque
              pas de prédiction (bruit odométrie/IMU) et qui RETRECIT
              brutalement à chaque correction par une balise (recalage GPS).
"""

from __future__ import annotations
import numpy as np
from trans import AnchorPoint, LocalToGpsCalibrator, ecef_to_enu, geodetic_to_ecef

np.set_printoptions(suppress=True)


# ---------------------------------------------------------------------------
# EKF de position (état = [x, y, z] dans le repère ENU "world")
# ---------------------------------------------------------------------------

class PositionEKF:
    def __init__(self, x0: np.ndarray, P0: np.ndarray):
        self.x = x0.astype(float).copy()   # état estimé (3,)
        self.P = P0.astype(float).copy()   # covariance (3,3)

    def predict(self, delta_odom: np.ndarray, odom_noise_std: np.ndarray) -> None:
        """
        Étape de prédiction : avance l'état d'un déplacement mesuré par
        l'odométrie (roues) recalé par l'IMU, et fait GRANDIR la covariance
        d'autant de bruit de processus (glissement des roues + dérive gyro).

        delta_odom      : déplacement (dx, dy, dz) depuis le pas précédent, m
        odom_noise_std  : écart-type du bruit sur ce déplacement (m), par axe
        """
        # Modèle linéaire simple : x_k = x_{k-1} + delta_odom  (F = I, B = I)
        self.x = self.x + delta_odom

        Q = np.diag(odom_noise_std ** 2)   # bruit de processus de ce pas
        self.P = self.P + Q                # F P F^T + Q, avec F = I

    def update(self, z: np.ndarray, R: np.ndarray) -> None:
        """
        Étape de correction : une balise vient d'être lue/déposée, on connaît
        sa position GPS -> convertie en ENU (z), avec une incertitude R
        (précision du GPS ayant servi à l'ancrage). Recale x et réduit P.
        """
        H = np.eye(3)                       # mesure directe de la position
        y = z - H @ self.x                  # innovation
        S = H @ self.P @ H.T + R            # covariance de l'innovation
        K = self.P @ H.T @ np.linalg.inv(S)  # gain de Kalman

        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    @property
    def err_m(self) -> float:
        """Incertitude 1-sigma équivalente (racine de la trace de P)."""
        return float(np.sqrt(np.trace(self.P)))


# ---------------------------------------------------------------------------
# Test avec les MEMES entrées que le test Umeyama précédent
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # --- 1. Mêmes 4 ancrages qu'avant ---
    anchors = [
        AnchorPoint(np.array([0.0, 0.0, 0.0]), lat=34.4310, lon=8.7840, alt=310.0),
        AnchorPoint(np.array([5.0, 0.0, 0.0]), lat=34.4310, lon=8.78406, alt=310.1),
        AnchorPoint(np.array([0.0, 5.0, 0.0]), lat=34.43105, lon=8.7840, alt=310.0),
        AnchorPoint(np.array([5.0, 5.0, 0.2]), lat=34.43105, lon=8.78406, alt=310.2),
    ]
    calib = LocalToGpsCalibrator(anchors)
    print(f"Erreur de calibration Umeyama (RMS) : {calib.calib_rms_m:.3f} m\n")

    # --- 2. Même balise cible : position locale (12.3, 7.8, 0.0), 42 m parcourus ---
    p_balise_local = np.array([12.3, 7.8, 0.0])
    dist_total_m = 42.0

    # --- 3. Initialisation de l'EKF au dernier point de recalage (origine, ancrage 0) ---
    x0 = np.array([0.0, 0.0, 0.0])            # position ENU au dernier recalage
    P0 = np.eye(3) * (calib.calib_rms_m ** 2)  # incertitude initiale = erreur de calibration
    ekf = PositionEKF(x0, P0)

    # --- 4. Phase de prédiction : simulate N pas d'odométrie/IMU vers la balise ---
    # (trajectoire rectiligne simplifiée vers p_balise_local, dérive proportionnelle
    #  à la distance parcourue, comme le ferait l'odométrie+IMU réels)
    n_steps = 20
    step_vec = p_balise_local / n_steps        # déplacement par pas (m)
    step_dist = dist_total_m / n_steps         # distance parcourue par pas
    # écart-type du bruit odométrie/IMU : ~2% de la distance parcourue par pas
    odom_noise_std = np.array([0.02, 0.02, 0.01]) * step_dist

    for _ in range(n_steps):
        ekf.predict(step_vec, odom_noise_std)

    print(f"Après {n_steps} pas de prédiction ({dist_total_m} m parcourus) :")
    print(f"  Position estimée (world/ENU) : {ekf.x}")
    print(f"  Incertitude avant correction  : {ekf.err_m:.3f} m\n")

    # --- 5. Phase de correction : la balise est déposée, on lit sa position GPS ---
    # (même position GPS que celle obtenue par Umeyama pour comparaison directe)
    gps_umeyama = calib.local_to_gps(p_balise_local, dist_since_last_anchor_m=dist_total_m)
    print(f"Position GPS (référence Umeyama) : {gps_umeyama}")

    z_ecef = geodetic_to_ecef(gps_umeyama["lat"], gps_umeyama["lon"], gps_umeyama["alt"])
    z_enu = ecef_to_enu(z_ecef, calib.ref_ecef, calib.ref_anchor.lat, calib.ref_anchor.lon)

    # Incertitude de la mesure GPS de la balise (précision du récepteur GPS embarqué)
    gps_measurement_std_m = 3.0   # ex : GPS civil standard
    R = np.eye(3) * (gps_measurement_std_m ** 2)

    ekf.update(z_enu, R)

    print(f"\nAprès correction par la balise :")
    print(f"  Position estimée (world/ENU) : {ekf.x}")
    print(f"  Incertitude après correction  : {ekf.err_m:.3f} m")

    print("\n--- Comparaison finale ---")
    print(f"  Umeyama seul (formule fixe)      : err_m = 1.22 m")
    print(f"  EKF  (covariance réelle propagée) : err_m = {ekf.err_m:.2f} m")