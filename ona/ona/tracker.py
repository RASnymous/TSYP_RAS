"""
Robot tracking at the ONA.

Every few seconds each robot sends a signed ROBOT frame ("ping") with its own
SLAM pose. Each gateway that hears it measures its distance to the robot.
For every ping the ONA then has two independent answers to "where is it?":

    reported  the robot's own pose, private map frame -> GPS with the entrance
              calibration (frame translation, geodesy.py)
    measured  the gateways' ranges, fused by the range EKF (kalman.py) with the
              robot's own displacement since its last ping

and the raw 3-sphere fix (multilateration.py) with its integrity check.

When the two answers agree within their error bounds the position on the
Command Post's map is confirmed by physics, not only by a signature. When they
drift apart, the robot's SLAM has slipped (or the calibration is off): the
dashboard says so, and the measured position is the one to trust.

Good fixes are also (map, ENU) pairs: the same Umeyama alignment as at the
entrance, re-computed during the mission, checks the frame translation itself.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .geodesy import LocalToGpsCalibrator, planar_alignment
from .kalman import RangeEKF
from .multilateration import RangeObs, trilaterate, Fix


@dataclass
class TrackerParams:
    robot_height_m: float = 0.30        # antenna height of a robot above the floor
    ping_window_s: float = 1.0          # receptions of one ping are grouped over this time
    noise_per_m: float = 0.05           # EKF process noise: m of error per m driven (SLAM odometry)
    noise_floor_m: float = 0.03
    unknown_motion_mps: float = 0.8     # motion assumed when a ping's content is not confirmed
    agree_sigma: float = 3.0            # reported vs measured: agree below this many sigmas
    max_speed_mps: float = 1.0          # faster than this between two pings = a jump of the robot's own pose
    drift_confirm: int = 3              # pings in a row before a drift is declared (or cleared)
    drift_clear_m: float = 1.0          # ... and to clear it, own pose back within max(this, half the drift)
    static_cap: int = 15                # parked: range weight / sqrt(1 + parked pings), at most / 4
    few_gw_inflation: float = 1.6       # range sigma x this when fewer than 3 gateways hear a ping
    drift_needs_fix: bool = True        # a drift is declared only if a 3-sphere fix disagrees too
    jump_motion: float = 0.5            # own pose jumped: assume it moved this x max speed x dt (sigma)
    drift_distrust: float = 8.0         # process noise x this once the robot's own pose drifts
    corr_inflation: float = 2.5         # range sigma x this in the EKF: wall errors repeat from ping to ping
    floor_vs_fix: float = 0.0           # optional: EKF sigma never below this x the single-ping sigma
    calib_min_pairs: int = 6
    calib_min_spread_m: float = 3.0
    calib_max_hdop: float = 3.0
    calib_max_sigma_m: float = 2.0
    calib_tol_m: float = 1.5            # through walls the check resolves gross errors only:
    calib_tol_deg: float = 6.0          # a frame off by more than 1.5 m or 6 degrees
    history: int = 600
    rssi_ref_dbm: float = -15.0         # RSSI at 1 m in line of sight (868 MHz, 14 dBm, 2 dBi antennas)
    rssi_exponent: float = 3.0          # path-loss exponent
    rssi_sigma_db: float = 4.0          # shadowing (random part of the path loss)
    nlos_mitigation: bool = True        # correct time-of-flight ranges for walls, from the excess path loss
    wall_loss_db: float = 6.0           # extra loss per wall             } calibrate these on site: drive
    wall_bias_m: float = 0.6            # extra ToF range per wall (mean) } the robot to 3 known points
    wall_bias_sd_m: float = 0.25        # spread of that extra range per wall
    max_walls: float = 4.0              # never correct for more walls than this
    rssi_autoref: bool = False          # learn line-of-sight RSSI from the data (only if the robot sees each gateway)
    rssi_ranging: bool = True           # turn an RSSI into a (rough) distance when no time of flight is given.
                                        # Underground (the mine config): False - through rock and around corners
                                        # the RSSI says nothing about the distance


@dataclass
class Reception:
    gw: str
    t: float
    range_m: Optional[float]
    range_sd: Optional[float]
    rssi: Optional[float]
    snr: Optional[float]
    src: str


@dataclass
class PingGroup:
    seq: int
    t0: float
    rx: dict = field(default_factory=dict)      # gw -> Reception
    done: bool = False


def nlos_correct(r: float, sd: float, rssi: Optional[float], p: TrackerParams, ref_dbm: Optional[float] = None
                 ) -> tuple:
    """A wall between a gateway and the robot makes the radio path longer (the signal goes through or
    around it, slower): time-of-flight ranges come out too long, never too short. The same wall also
    weakens the signal. The ONA compares the RSSI with what free space would give at that range; the
    excess loss says how many walls are in the way, and the range is shortened by their expected bias,
    with a wider error bar. Returns (corrected range, sigma, estimated walls)."""
    if not p.nlos_mitigation or rssi is None:
        return r, sd, 0.0
    ref = p.rssi_ref_dbm if ref_dbm is None else ref_dbm
    expected = ref - 10.0 * p.rssi_exponent * math.log10(max(r, 1.0))
    walls = min(p.max_walls, max(0.0, (expected - float(rssi)) / p.wall_loss_db))
    r2 = max(0.1, r - walls * p.wall_bias_m)
    sd2 = math.sqrt(sd * sd + (walls * p.wall_bias_sd_m) ** 2 + (p.rssi_sigma_db / p.wall_loss_db * p.wall_bias_m) ** 2
                    * (1.0 if walls > 0 else 0.25))
    return r2, sd2, walls


def rssi_to_range(rssi: float, p: TrackerParams) -> tuple:
    """Log-distance path loss inverted. Very rough through rubble: sigma grows with distance."""
    d = 10 ** ((p.rssi_ref_dbm - rssi) / (10.0 * p.rssi_exponent))
    sd = d * math.log(10) / (10.0 * p.rssi_exponent) * p.rssi_sigma_db
    return d, max(sd, 0.5)


class RobotTrack:
    def __init__(self, robot_id: int, cal: LocalToGpsCalibrator, gateways_enu: dict, params: TrackerParams):
        self.robot_id = robot_id
        self.cal = cal
        self.gws = gateways_enu
        self.p = params
        self.groups: dict = {}
        self.ekf: Optional[RangeEKF] = None
        self.last_local: Optional[np.ndarray] = None
        self.last_t: Optional[float] = None
        self.last_report: Optional[dict] = None
        self.last_seq: Optional[int] = None
        self.history = deque(maxlen=params.history)       # (t, e, n, sigma)
        self.pairs: list = []                               # (local xy, enu xy, weight)
        self.calib_check: Optional[dict] = None
        self.role = None
        self.processed = 0
        self.last_fix: Optional[Fix] = None
        self.last_payload: Optional[dict] = None
        self.rssi_hist: dict = {}
        self.drifting = False
        self.time_scale = 1.0          # a simulator running N x faster: N mission seconds per clock second
        self.static = 0
        self.jumps = 0
        self.drift_run = 0
        self.agree_run = 0

    # --------------------------------------------------------------- input
    def add_reception(self, seq: int, rec: Reception) -> None:
        g = self.groups.get(seq)
        if g is None:
            if self.last_seq is not None:
                d = (seq - self.last_seq) & 0xFFFF
                if d == 0 or d > 0x8000:
                    return                                    # already processed, or older
            g = self.groups[seq] = PingGroup(seq, rec.t)
        if g.done:
            return
        g.rx.setdefault(rec.gw, rec)

    def ready(self, now: float, n_gateways: int) -> list:
        out = []
        for seq, g in list(self.groups.items()):
            if g.done:
                continue
            if len(g.rx) >= n_gateways or now - g.t0 >= self.p.ping_window_s:
                out.append(g)
        out.sort(key=lambda g: g.t0)
        return out

    # --------------------------------------------------------------- helpers
    def _obs(self, g: PingGroup, excluded: set) -> list:
        obs = []
        for gw, r in g.rx.items():
            if gw in excluded or gw not in self.gws:
                continue
            if r.range_m is not None and r.range_m > 0:
                rr, sd, walls = nlos_correct(float(r.range_m), float(r.range_sd or 1.0), r.rssi, self.p,
                                             self._rssi_ref(gw, float(r.range_m), r.rssi))
                o = RangeObs(gw, self.gws[gw], rr, sd, r.src or 'tof')
                o.walls = walls
                o.raw = float(r.range_m)
                obs.append(o)
            elif r.rssi is not None and self.p.rssi_ranging:
                d, sd = rssi_to_range(float(r.rssi), self.p)
                obs.append(RangeObs(gw, self.gws[gw], d, sd, 'rssi'))
        return obs

    def _rssi_ref(self, gw: str, r: float, rssi: Optional[float]) -> Optional[float]:
        """Each gateway's line-of-sight RSSI at 1 m, learnt from the data: the strongest signals for their
        range (90th percentile) are the ones with no wall in the way. If the robot never has a clear line
        to a gateway this underestimates the walls, i.e. corrects less: the safe side."""
        if rssi is None or not self.p.rssi_autoref:
            return None
        h = self.rssi_hist.setdefault(gw, deque(maxlen=400))
        h.append(float(rssi) + 10.0 * self.p.rssi_exponent * math.log10(max(r, 1.0)))
        if len(h) < 30:
            return None
        return float(np.percentile(np.array(h), 90))

    def _single_ping_sigma(self, obs: list) -> Optional[float]:
        """How well this ping's ranges alone pin the robot down (2 or more ranges), at the current estimate."""
        if self.ekf is None or len(obs) < 2:
            return None
        H, W = [], []
        for o in obs:
            dz = self.p.robot_height_m - (o.pos[2] if o.pos.shape[0] > 2 else 0.0)
            d = self.ekf.x - o.pos[:2]
            h = max(math.sqrt(float(d @ d) + dz * dz), 1e-6)
            H.append(d / h)
            W.append(1.0 / o.sigma ** 2)
        H, W = np.array(H), np.array(W)
        N = (H.T * W) @ H
        if np.linalg.cond(N) > 1e6:
            return None
        return float(math.sqrt(max(np.linalg.eigvalsh(np.linalg.inv(N)))))

    def _gps(self, e: float, n: float) -> tuple:
        lat, lon, _ = self.cal.enu_to_gps([e, n, self.cal.t[2]])
        return lat, lon

    # --------------------------------------------------------------- one ping
    def process(self, g: PingGroup, report: Optional[dict], votes: int, of: int, state: str,
                excluded: set = frozenset()) -> dict:
        """g: the receptions of one ping. report: its decoded content if the vote confirmed it."""
        g.done = True
        self.groups = {s: x for s, x in self.groups.items() if not x.done and x.t0 >= g.t0 - 30}
        self.last_seq = g.seq
        t = g.t0
        obs = self._obs(g, excluded)
        R2 = self.cal.R[:2, :2]

        rep_enu = None
        if report is not None:
            self.role = report.get('role', self.role)
            loc = np.array([report['x'], report['y'], 0.0])
            rep_enu = self.cal.local_to_enu(loc)[:2]

        fix = trilaterate(obs, self.p.robot_height_m,
                          x0=None if self.ekf is None else self.ekf.x) if len(obs) >= 3 else Fix(False)
        self.last_fix = fix if fix.ok else self.last_fix

        # ---- EKF: initialise, predict, correct with each range
        if self.ekf is None:
            if fix.ok and fix.raim != 'fail':
                self.ekf = RangeEKF(fix.e, fix.n, max(fix.sigma_m, 0.5))
            elif rep_enu is not None:
                sd0 = (report.get('pose_sd_m') or 0.3) + self.cal.calib_rms_m + 1.0
                self.ekf = RangeEKF(rep_enu[0], rep_enu[1], sd0)
        else:
            if report is not None and self.last_local is not None:
                d_local = np.array([report['x'], report['y']]) - self.last_local
                step = float(np.linalg.norm(d_local))
                dt = max(0.5, (t - (self.last_t or t)) * self.time_scale)   # mission seconds
                self.static = self.static + 1 if step < 0.05 else 0
                if step / dt > self.p.max_speed_mps:
                    # a robot cannot teleport: its own pose jumped (SLAM relocalised or slipped). Do not
                    # follow it; let the gateways say where it went.
                    self.jumps += 1
                    # it physically moved at most max_speed x dt: stay confident, so that the gateways see
                    # at once that the robot's own pose and the measurement no longer agree
                    if self.p.jump_motion < 0:      # (v9.0 behaviour, for comparisons)
                        self.ekf.predict(np.zeros(2), np.array([step, step]) + self.p.max_speed_mps * dt)
                    else:
                        self.ekf.predict(np.zeros(2), np.array([1.0, 1.0]) * (self.p.jump_motion * self.p.max_speed_mps * dt))
                else:
                    # once the robot's own pose has been caught drifting, its displacement counts much less
                    k = self.p.drift_distrust if self.drifting else 1.0
                    self.ekf.predict_move(R2 @ d_local, step, self.p.noise_per_m * k, self.p.noise_floor_m)
            else:
                dt = max(0.0, (t - (self.last_t or t)) * self.time_scale)
                s = self.p.noise_floor_m + self.p.unknown_motion_mps * dt
                self.ekf.predict(np.zeros(2), np.array([s, s]))
        updates = []
        if self.ekf is not None:
            for o in sorted(obs, key=lambda o: o.sigma):
                infl = 1.0 if self.drifting else self.p.corr_inflation     # own pose unreliable: ranges lead
                # standing still, each ping repeats the same wall errors: it adds almost nothing new
                infl *= math.sqrt(1.0 + min(self.static, self.p.static_cap))
                if len(obs) < 3:
                    infl *= self.p.few_gw_inflation   # no spare range: a wall bias cannot be seen, trust less
                updates.append(self.ekf.update_range(o.gw, o.pos, o.r, o.sigma * infl, self.p.robot_height_m))
            # Wall errors are the same from one ping to the next (same walls in the way): averaging many pings
            # does not remove them. The filter must not claim to be better than one ping's 3-sphere fix
            # (x floor_vs_fix), or its error bars lie and every small bias looks like a SLAM drift.
            floor = self._single_ping_sigma(obs) if self.p.floor_vs_fix > 0 else None
            if floor is not None and self.ekf.sigma_m < self.p.floor_vs_fix * floor:
                self.ekf.P = self.ekf.P * (self.p.floor_vs_fix * floor / self.ekf.sigma_m) ** 2
        if report is not None:
            self.last_local = np.array([report['x'], report['y']])
            self.last_report = report
        self.last_t = t

        # ---- reported vs measured
        consistency = {'state': 'n/a'}
        if self.ekf is not None and rep_enu is not None and self.ekf.updates > 0:
            sd_rep = (report.get('pose_sd_m') or 0.3) + self.cal.calib_rms_m
            dist = float(np.linalg.norm(rep_enu - self.ekf.x))
            m = self.ekf.mahalanobis_to(rep_enu, np.eye(2) * sd_rep ** 2)
            raw = m > self.p.agree_sigma
            # accusing the robot's SLAM needs a real 3-sphere fix that disagrees too: with only two gateways
            # (one down or ignored) the filter leans on odometry and on biased ranges, and a "drift" would
            # often be the gateways' own error
            fix_ok = fix.ok and fix.raim != 'fail' and fix.cov is not None
            if not self.p.drift_needs_fix:
                pass
            elif raw and fix_ok:
                dv = rep_enu - np.array([fix.e, fix.n])
                S = fix.cov[:2, :2] + np.eye(2) * sd_rep ** 2
                raw = float(np.sqrt(max(0.0, dv @ np.linalg.solve(S, dv)))) > self.p.agree_sigma
            elif raw:
                raw = False
            self.drift_run = self.drift_run + 1 if raw else 0
            self.agree_run = self.agree_run + 1 if not raw else 0
            if not self.drifting and self.drift_run >= self.p.drift_confirm:
                self.drifting = True                   # several pings in a row: a real slip, not noise
                self.drift_dist = dist
                # the last few calibration pairs already had the slipped pose: they say nothing about the
                # entrance calibration (the in-mission frame check must not blame it for a SLAM slip)
                self.pairs = self.pairs[:max(0, len(self.pairs) - (self.p.drift_confirm + 2))]
                if len(self.pairs) >= self.p.calib_min_pairs:
                    self.calib_check = self._calibration_check() or self.calib_check
                # if the filter followed the slipping odometry for a while (a slow drift), re-acquire from the
                # spheres; if it did not follow it (a jump it rejected), it is right: keep it
                if fix.ok and fix.raim != 'fail' and fix.cov is not None:
                    if self.ekf.mahalanobis_to(np.array([fix.e, fix.n]), fix.cov[:2, :2]) > self.p.agree_sigma:
                        self.ekf.x = np.array([fix.e, fix.n])
                        self.ekf.P = fix.cov[:2, :2] * 1.5 ** 2
                else:
                    self.ekf.P = self.ekf.P + np.eye(2) * dist ** 2
            elif (self.drifting and self.agree_run >= self.p.drift_confirm
                  and dist < max(self.p.drift_clear_m, 0.5 * getattr(self, 'drift_dist', 0.0))):
                # "agrees again" needs the robot's own pose to really come back (a relocalisation), not just
                # a wider error ellipse on the gateways' side
                self.drifting = False
            consistency = {'state': 'drift' if self.drifting else 'agree', 'dist_m': round(dist, 2),
                           'sigmas': round(m, 2)}

        # ---- calibration pairs: good fix + confirmed pose
        if (report is not None and report.get('pose_valid', True) and not self.drifting and fix.ok
                and fix.raim == 'pass'
                and fix.hdop <= self.p.calib_max_hdop and fix.sigma_m <= self.p.calib_max_sigma_m):
            self.pairs.append((np.array([report['x'], report['y']]), np.array([fix.e, fix.n]),
                               1.0 / max(fix.sigma_m, 0.1) ** 2))
            if len(self.pairs) > 300:
                self.pairs = self.pairs[::2]           # keep the spread, halve the count
            if len(self.pairs) >= self.p.calib_min_pairs and len(self.pairs) % 3 == 0:
                self.calib_check = self._calibration_check() or self.calib_check

        self.processed += 1
        payload = self._payload(t, report, votes, of, state, obs, updates, fix, rep_enu, consistency)
        if self.ekf is not None:
            self.history.append((t, float(self.ekf.x[0]), float(self.ekf.x[1]), self.ekf.sigma_m))
        self.last_payload = payload
        return payload

    def _calibration_check(self) -> Optional[dict]:
        loc = np.array([[p[0][0], p[0][1], 0.0] for p in self.pairs])
        enu = np.array([[p[1][0], p[1][1], self.cal.t[2]] for p in self.pairs])
        w = np.array([p[2] for p in self.pairs])
        spread = float(np.max(np.linalg.norm(loc[:, :2] - loc[:, :2].mean(axis=0), axis=1)))
        if spread < self.p.calib_min_spread_m:
            return None
        R, tr, _ = planar_alignment(loc, enu, w)
        pred = loc @ R.T + tr
        rms = float(np.sqrt(np.mean(np.sum((pred - enu)[:, :2] ** 2, axis=1))))
        yaw_new = math.atan2(R[1, 0], R[0, 0])
        dyaw = math.degrees((yaw_new - self.cal.yaw_rad + math.pi) % (2 * math.pi) - math.pi)
        # how far the entrance calibration and the in-mission one disagree over the robot's area
        diffs = [float(np.linalg.norm((R @ p + tr - self.cal.local_to_enu(p))[:2])) for p in loc]
        return {'pairs': len(self.pairs), 'rms_m': round(rms, 3), 'dyaw_deg': round(dyaw, 2),
                'max_shift_m': round(max(diffs), 3), 'mean_shift_m': round(sum(diffs) / len(diffs), 3),
                'verdict': 'holds' if (sum(diffs) / len(diffs) < self.p.calib_tol_m
                                       and abs(dyaw) < self.p.calib_tol_deg) else 'drifted'}

    def _payload(self, t, report, votes, of, state, obs, updates, fix, rep_enu, consistency) -> dict:
        upd = {u.gw: u for u in updates}
        ranges = []
        for o in obs:
            glat, glon = self._gps(o.pos[0], o.pos[1])
            dz = self.p.robot_height_m - (o.pos[2] if o.pos.shape[0] > 2 else 0.0)
            u = upd.get(o.gw)
            ranges.append({'gw': o.gw, 'lat': round(glat, 7), 'lon': round(glon, 7), 'range_m': round(o.r, 2),
                           'raw_m': round(getattr(o, 'raw', o.r), 2), 'walls': round(getattr(o, 'walls', 0.0), 1),
                           'horiz_m': round(math.sqrt(max(o.r * o.r - dz * dz, 0.0)), 2),
                           'sigma_m': round(o.sigma, 2), 'src': o.src,
                           'residual_m': None if not fix.ok else round(fix.residuals.get(o.gw, 0.0), 2),
                           'accepted': None if u is None else u.accepted})
        out = {'robot_id': self.robot_id, 'role': (report or {}).get('role', self.role or 'ROBOT'),
               'timestamp': t, 'votes': votes, 'of': of, 'state': state, 'ranges': ranges,
               'consistency': consistency, 'calibration_check': self.calib_check}
        if report is not None:
            rlat, rlon = self._gps(rep_enu[0], rep_enu[1])
            out['reported'] = {'lat': round(rlat, 7), 'lon': round(rlon, 7), 'x': round(report['x'], 3),
                               'y': round(report['y'], 3), 'pose_sd_m': report.get('pose_sd_m')}
            for k in ('phase', 'mission_id', 'mission_ack', 'done', 'total', 'battery_pct', 'odo_m', 'stuck',
                      'emergency', 'last_beacon', 'seq'):
                out[k] = report.get(k)
            out['heading'] = round(report['yaw'] + self.cal.yaw_rad, 4)
        if fix.ok:
            flat, flon = self._gps(fix.e, fix.n)
            out['fix'] = {'lat': round(flat, 7), 'lon': round(flon, 7), 'sigma_m': round(fix.sigma_m, 2),
                          'hdop': round(fix.hdop, 2), 'raim': fix.raim, 'chi2': round(fix.chi2, 2)}
        if self.ekf is not None:
            mlat, mlon = self._gps(self.ekf.x[0], self.ekf.x[1])
            out['measured'] = {'lat': round(mlat, 7), 'lon': round(mlon, 7), 'sigma_m': round(self.ekf.sigma_m, 2),
                               'ellipse': self.ekf.ellipse(), 'ranges_used': self.ekf.updates}
            out['lat'], out['lon'] = out['measured']['lat'], out['measured']['lon']
            out['sigma_m'] = out['measured']['sigma_m']
            out['source'] = 'gateways'
        elif report is not None:
            out['lat'], out['lon'] = out['reported']['lat'], out['reported']['lon']
            out['sigma_m'] = (report.get('pose_sd_m') or 0.3) + self.cal.calib_rms_m
            out['source'] = 'reported'
        return out
