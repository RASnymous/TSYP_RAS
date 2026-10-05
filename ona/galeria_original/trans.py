"""
GALERIA - The Living Map
Partie 5 : Transformation de coordonnées locales (repère SLAM 'map'/'world')
           vers coordonnées GPS réelles (WGS84).

Principe :
    1. On dispose de 3-4 points d'ancrage mesurés à la fois :
         - dans le repère du robot (map / world, en mètres, ENU local)
         - en GPS réel (lat, lon, alt) au même endroit
    2. L'algorithme d'Umeyama trouve la rotation R, la translation t (et un
       facteur d'échelle s, ici fixé à 1 puisque le SLAM est métrique) qui
       alignent au mieux les points du robot sur les points GPS connus.
    3. Toute nouvelle position locale (ex. une balise) est ensuite projetée
       avec (R, t) dans le repère ENU local, puis convertie en GPS
       (ENU -> ECEF -> WGS84).
    4. Chaque balise traversée sert de nouveau point de recalage : on peut
       ré-estimer (R, t) régulièrement pour limiter la dérive.

Dépendances : numpy uniquement.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
import numpy as np

# ---------------------------------------------------------------------------
# Constantes WGS84
# ---------------------------------------------------------------------------
WGS84_A = 6378137.0                 # demi-grand axe (m)
WGS84_F = 1 / 298.257223563         # aplatissement
WGS84_E2 = WGS84_F * (2 - WGS84_F)  # excentricité au carré


@dataclass
class AnchorPoint:
    """Point d'ancrage : position connue dans les deux repères."""
    local_xyz: np.ndarray   # position dans le repère robot (map/world), mètres
    lat: float               # latitude GPS mesurée (degrés)
    lon: float               # longitude GPS mesurée (degrés)
    alt: float = 0.0         # altitude GPS mesurée (m)


# ---------------------------------------------------------------------------
# Conversions géodésiques : WGS84 (lat/lon/alt) <-> ECEF <-> ENU local
# ---------------------------------------------------------------------------

def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> np.ndarray:
    """Latitude/longitude/altitude (WGS84) -> coordonnées cartésiennes ECEF."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)

    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)

    x = (n + alt) * cos_lat * cos_lon
    y = (n + alt) * cos_lat * sin_lon
    z = (n * (1 - WGS84_E2) + alt) * sin_lat
    return np.array([x, y, z])


def ecef_to_geodetic(ecef: np.ndarray) -> tuple[float, float, float]:
    """ECEF -> (lat_deg, lon_deg, alt_m). Méthode itérative de Bowring."""
    x, y, z = ecef
    lon = math.atan2(y, x)

    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    for _ in range(5):  # convergence rapide, 5 itérations suffisent largement
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - WGS84_E2 * n / (n + alt)))

    return math.degrees(lat), math.degrees(lon), alt


def ecef_to_enu(ecef: np.ndarray, ref_ecef: np.ndarray,
                 ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    """Vecteur ECEF -> ENU local, centré sur un point de référence."""
    d = ecef - ref_ecef
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)

    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)

    R = np.array([
        [-sin_lon,            cos_lon,           0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [ cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat],
    ])
    return R @ d


def enu_to_ecef(enu: np.ndarray, ref_ecef: np.ndarray,
                 ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    """ENU local -> ECEF, inverse de ecef_to_enu (R est orthogonale)."""
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)

    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)

    R = np.array([
        [-sin_lon,            cos_lon,           0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [ cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat],
    ])
    return ref_ecef + R.T @ enu


# ---------------------------------------------------------------------------
# Alignement d'Umeyama : repère robot (map) -> repère ENU local ("world")
# ---------------------------------------------------------------------------

def umeyama_alignment(src: np.ndarray, dst: np.ndarray,
                       estimate_scale: bool = False) -> tuple[np.ndarray, np.ndarray, float]:
    """
    Trouve (R, t, s) minimisant  sum_i || dst_i - (s*R*src_i + t) ||^2.

    src, dst : arrays (N, 3), N >= 3.
    estimate_scale : False ici car le SLAM métrique donne déjà l'échelle réelle.

    Retourne (R, t, s).
    """
    assert src.shape == dst.shape and src.shape[0] >= 3

    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)

    src_c = src - mu_src
    dst_c = dst - mu_dst

    n = src.shape[0]
    cov = (dst_c.T @ src_c) / n

    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt

    if estimate_scale:
        var_src = (src_c ** 2).sum() / n
        s = np.trace(np.diag(D) @ S) / var_src
    else:
        s = 1.0

    t = mu_dst - s * R @ mu_src
    return R, t, s


# ---------------------------------------------------------------------------
# Calibrateur complet : local (map) -> world (ENU) -> GPS
# ---------------------------------------------------------------------------

class LocalToGpsCalibrator:
    """
    Calibré une fois au démarrage (3-4 balises d'ancrage à l'entrée),
    puis recalé au fil de la mission à chaque nouvelle balise franchie
    (section 5 : "chaque balise déposée sert de nouveau point de recalage").
    """

    def __init__(self, anchors: list[AnchorPoint]):
        self.recalibrate(anchors)

    def recalibrate(self, anchors: list[AnchorPoint]) -> None:
        if len(anchors) < 3:
            raise ValueError("Umeyama nécessite au moins 3 points d'ancrage.")

        # Point de référence ENU = premier ancrage (origine du repère "world")
        self.ref_anchor = anchors[0]
        self.ref_ecef = geodetic_to_ecef(self.ref_anchor.lat, self.ref_anchor.lon,
                                          self.ref_anchor.alt)

        src = np.array([a.local_xyz for a in anchors])           # repère map (robot)
        dst_enu = np.array([
            ecef_to_enu(geodetic_to_ecef(a.lat, a.lon, a.alt),
                        self.ref_ecef, self.ref_anchor.lat, self.ref_anchor.lon)
            for a in anchors
        ])

        self.R, self.t, self.s = umeyama_alignment(src, dst_enu, estimate_scale=False)

        # Résidu RMS -> première estimation de l'incertitude de calibration
        pred = (self.s * (src @ self.R.T)) + self.t
        residuals = np.linalg.norm(pred - dst_enu, axis=1)
        self.calib_rms_m = float(np.sqrt((residuals ** 2).mean()))

    def local_to_gps(self, p_map: np.ndarray, dist_since_last_anchor_m: float = 0.0
                      ) -> dict:
        """
        p_map : position (x, y, z) dans le repère SLAM 'map' du robot.
        dist_since_last_anchor_m : distance parcourue depuis le dernier
            recalage, utilisée pour estimer l'erreur croissante (dérive
            roues + gyroscope) tant qu'aucune nouvelle balise n'a recalé.

        Retourne un dict {lat, lon, alt, err_m} prêt à être inséré dans
        le message de balise ("gps": {"lat":..., "lon":..., "err_m":...}).
        """
        p_world = self.s * (self.R @ p_map) + self.t          # repère world (ENU)
        ecef = enu_to_ecef(p_world, self.ref_ecef,
                            self.ref_anchor.lat, self.ref_anchor.lon)
        lat, lon, alt = ecef_to_geodetic(ecef)

        # Modèle d'erreur simple : erreur de calibration + dérive linéaire
        # avec la distance parcourue depuis le dernier point de recalage.
        # Coefficient de dérive à calibrer sur le terrain (ex: 0.02 m/m).
        drift_coeff = 0.02
        err_m = self.calib_rms_m + drift_coeff * dist_since_last_anchor_m
        err_m = min(err_m, 5.0)  # borne haute raisonnable en environnement balisé

        return {"lat": round(lat, 6), "lon": round(lon, 6),
                "alt": round(alt, 2), "err_m": round(err_m, 2)}


# ---------------------------------------------------------------------------
# Exemple d'utilisation
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # 4 marqueurs posés à l'entrée : position robot (SLAM) + GPS mesuré au même point
    anchors = [
        AnchorPoint(np.array([0.0, 0.0, 0.0]), lat=34.4310, lon=8.7840, alt=310.0),
        AnchorPoint(np.array([5.0, 0.0, 0.0]), lat=34.4310, lon=8.78406, alt=310.1),
        AnchorPoint(np.array([0.0, 5.0, 0.0]), lat=34.43105, lon=8.7840, alt=310.0),
        AnchorPoint(np.array([5.0, 5.0, 0.2]), lat=34.43105, lon=8.78406, alt=310.2),
    ]

    calib = LocalToGpsCalibrator(anchors)
    print(f"Erreur de calibration (RMS) : {calib.calib_rms_m:.3f} m")

    # Position d'une balise déposée par le Writer, 42 m après le dernier recalage
    p_balise = np.array([12.3, 7.8, 0.0])
    gps = calib.local_to_gps(p_balise, dist_since_last_anchor_m=42.0)
    print("Position GPS de la balise :", gps)