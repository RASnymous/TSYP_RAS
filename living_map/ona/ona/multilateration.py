"""
Where is the robot? Three gateways, three spheres.

Each ONA gateway measures its distance r_i to the robot. The robot lies on a
sphere of radius r_i around gateway i. Three spheres meet in two points that
are mirror images across the plane of the gateways; the robot drives on the
floor, its height is known, so on the floor the spheres become three circles
that meet in one point.

GPS works the same way with satellites, with one difference: a GPS receiver
measures one-way travel times with a cheap clock, so its clock error is a 4th
unknown and it needs 4 satellites. The gateways use two-way ranging (a ping and
its echo, like a radar), the clock error cancels, and 3 gateways are enough.

Solver: weighted least squares by Gauss-Newton on the slant ranges, started
from the closed-form linear solution, with Huber re-weighting (a wall between
the robot and a gateway makes that range too long, never too short).

Integrity (RAIM, as in aviation GPS): with n ranges and 2 unknowns there are
n - 2 spare measurements. The weighted residuals must fit a chi-square law with
n - 2 degrees of freedom; if they do not, one range is wrong. With 3 gateways
the fault is detected; with 4 or more the bad gateway is also identified and
excluded.

DOP (dilution of precision): how the gateways' geometry turns range errors into
position errors. Gateways all on one side of the building give a large HDOP
(poor fix); gateways around it give HDOP close to 1.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from .kalman import chi2_threshold


@dataclass
class RangeObs:
    gw: str
    pos: np.ndarray        # gateway antenna, ENU (e, n, u) metres
    r: float               # measured slant range (m)
    sigma: float           # 1-sigma of that range (m)
    src: str = 'tof'       # 'tof' two-way time of flight, 'rssi' path-loss estimate


@dataclass
class Fix:
    ok: bool
    e: float = float('nan')
    n: float = float('nan')
    cov: Optional[np.ndarray] = None
    sigma_m: float = float('inf')     # 1-sigma radius of the largest error axis
    hdop: float = float('inf')
    residuals: dict = field(default_factory=dict)   # gw -> measured - computed (m)
    chi2: float = 0.0
    dof: int = 0
    raim: str = 'n/a'                 # 'pass', 'fail', 'excluded:<gw>', 'n/a'
    used: list = field(default_factory=list)
    iterations: int = 0
    reason: str = ''

    def as_dict(self) -> dict:
        return {'ok': self.ok, 'e': round(self.e, 3), 'n': round(self.n, 3),
                'sigma_m': round(self.sigma_m, 3) if math.isfinite(self.sigma_m) else None,
                'hdop': round(self.hdop, 2) if math.isfinite(self.hdop) else None,
                'residuals': {k: round(v, 3) for k, v in self.residuals.items()},
                'chi2': round(self.chi2, 3), 'dof': self.dof, 'raim': self.raim, 'used': self.used,
                'reason': self.reason}


def _horizontal(obs: RangeObs, robot_h: float) -> float:
    dz = robot_h - (obs.pos[2] if obs.pos.shape[0] > 2 else 0.0)
    return math.sqrt(max(obs.r * obs.r - dz * dz, 0.0))


def linear_guess(obs: Sequence[RangeObs], robot_h: float) -> Optional[np.ndarray]:
    """Closed form: subtract the first circle equation from the others -> linear system."""
    if len(obs) < 3:
        return None
    p0 = obs[0].pos[:2]
    r0 = _horizontal(obs[0], robot_h)
    A, b = [], []
    for o in obs[1:]:
        p = o.pos[:2]
        r = _horizontal(o, robot_h)
        A.append(2 * (p - p0))
        b.append(p @ p - p0 @ p0 - r * r + r0 * r0)
    A, b = np.array(A), np.array(b)
    if np.linalg.matrix_rank(A, tol=1e-6) < 2:
        return None
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return sol


def two_circle_candidates(a: RangeObs, b: RangeObs, robot_h: float) -> list:
    """The (0, 1 or 2) floor points at the right distance from two gateways."""
    pa, pb = a.pos[:2], b.pos[:2]
    ra, rb = _horizontal(a, robot_h), _horizontal(b, robot_h)
    d = float(np.linalg.norm(pb - pa))
    if d < 1e-6:
        return []
    ra_, rb_ = ra, rb
    if d > ra + rb:              # circles apart: take the best compromise on the line
        k = ra / (ra + rb) if ra + rb > 0 else 0.5
        return [pa + (pb - pa) * k]
    if d < abs(ra - rb):         # one inside the other
        big, small = (pa, pb) if ra > rb else (pb, pa)
        u = (small - big) / d
        return [big + u * max(ra_, rb_)]
    x = (d * d + ra * ra - rb * rb) / (2 * d)
    y = math.sqrt(max(ra * ra - x * x, 0.0))
    u = (pb - pa) / d
    v = np.array([-u[1], u[0]])
    base = pa + u * x
    return [base + v * y, base - v * y] if y > 1e-6 else [base]


def _solve(obs: Sequence[RangeObs], robot_h: float, x0: np.ndarray, max_iter: int, huber_k: float):
    x = np.array(x0, dtype=float)
    w_rob = np.ones(len(obs))
    it = 0
    for it in range(1, max_iter + 1):
        H, f, W = [], [], []
        for i, o in enumerate(obs):
            dz = robot_h - (o.pos[2] if o.pos.shape[0] > 2 else 0.0)
            d = x - o.pos[:2]
            h = math.sqrt(float(d @ d) + dz * dz)
            h = max(h, 1e-6)
            H.append(d / h)
            f.append(o.r - h)
            W.append(w_rob[i] / (o.sigma ** 2))
        H, f, W = np.array(H), np.array(f), np.array(W)
        HtW = H.T * W
        N = HtW @ H
        if np.linalg.cond(N) > 1e12:
            return None, None, it
        dx = np.linalg.solve(N, HtW @ f)
        x = x + dx
        # Huber re-weighting once the solution has settled a little
        if len(obs) > 2 and it >= 2:
            z = np.abs(f) / np.array([o.sigma for o in obs])
            w_rob = np.where(z <= huber_k, 1.0, huber_k / np.maximum(z, 1e-9))
        if float(np.linalg.norm(dx)) < 1e-5:
            break
    return x, w_rob, it


def trilaterate(obs: Sequence[RangeObs], robot_h: float = 0.2, x0: Optional[Sequence[float]] = None,
                max_iter: int = 30, p_fa: float = 0.01, huber_k: float = 2.5, raim_exclude: bool = True) -> Fix:
    obs = [o for o in obs if o is not None and math.isfinite(o.r) and o.r > 0 and o.sigma > 0]
    if len(obs) < 2:
        return Fix(False, reason=f'{len(obs)} range(s): need 3 (2 with a prior)')
    if len(obs) == 2 and x0 is None:
        return Fix(False, reason='2 ranges and no prior: two possible points')
    start = None if x0 is None else np.asarray(x0, dtype=float)[:2]
    if start is None:
        start = linear_guess(obs, robot_h)
        if start is None:
            start = np.mean([o.pos[:2] for o in obs], axis=0)
    elif len(obs) == 2:
        cands = two_circle_candidates(obs[0], obs[1], robot_h)
        if cands:
            start = min(cands, key=lambda c: float(np.linalg.norm(c - start)))
    x, w_rob, it = _solve(obs, robot_h, start, max_iter, huber_k)
    if x is None:
        return Fix(False, reason='gateways in a line: geometry cannot fix the position', iterations=it)
    fix = _finish(obs, robot_h, x, w_rob, p_fa, it)
    if fix.raim == 'fail' and raim_exclude and len(obs) >= 4:
        best = None
        for sub in itertools.combinations(range(len(obs)), len(obs) - 1):
            sobs = [obs[i] for i in sub]
            sx, sw, sit = _solve(sobs, robot_h, x, max_iter, huber_k)
            if sx is None:
                continue
            f2 = _finish(sobs, robot_h, sx, sw, p_fa, sit)
            if f2.raim == 'pass' and (best is None or f2.chi2 < best.chi2):
                bad = [o.gw for i, o in enumerate(obs) if i not in sub][0]
                f2.raim = f'excluded:{bad}'
                best = f2
        if best is not None:
            return best
    return fix


def _finish(obs, robot_h, x, w_rob, p_fa, it) -> Fix:
    H, W, res = [], [], {}
    chi2 = 0.0
    for i, o in enumerate(obs):
        dz = robot_h - (o.pos[2] if o.pos.shape[0] > 2 else 0.0)
        d = x - o.pos[:2]
        h = max(math.sqrt(float(d @ d) + dz * dz), 1e-6)
        H.append(d / h)
        W.append(1.0 / o.sigma ** 2)
        res[o.gw] = o.r - h
        chi2 += (res[o.gw] / o.sigma) ** 2
    H, W = np.array(H), np.array(W)
    try:
        cov = np.linalg.inv((H.T * W) @ H)
        g = np.linalg.inv(H.T @ H)
        hdop = math.sqrt(max(0.0, float(np.trace(g))))
        sig = math.sqrt(max(0.0, float(max(np.linalg.eigvalsh(cov)))))
    except np.linalg.LinAlgError:
        return Fix(False, float(x[0]), float(x[1]), reason='singular geometry', iterations=it)
    dof = len(obs) - 2
    if dof <= 0:
        raim = 'n/a'
    else:
        raim = 'pass' if chi2 <= chi2_threshold(dof, p_fa) else 'fail'
    # a fix on the wrong side of a line of gateways is possible with 2 ranges only
    return Fix(True, float(x[0]), float(x[1]), cov, sig, hdop, res, chi2, dof, raim, [o.gw for o in obs], it)


def hdop_map(gws_enu: Sequence[np.ndarray], xs, ys, robot_h: float = 0.2) -> np.ndarray:
    """HDOP over a grid (for planning where to put the gateways)."""
    out = np.full((len(ys), len(xs)), np.inf)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            H = []
            for g in gws_enu:
                d = np.array([x - g[0], y - g[1]])
                dz = robot_h - (g[2] if len(g) > 2 else 0.0)
                h = math.sqrt(float(d @ d) + dz * dz)
                H.append(d / max(h, 1e-6))
            H = np.array(H)
            try:
                out[j, i] = math.sqrt(float(np.trace(np.linalg.inv(H.T @ H))))
            except np.linalg.LinAlgError:
                pass
    return out
