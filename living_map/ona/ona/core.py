"""
The Outside Network Area (ONA): the only bridge between the zone and the world.

    3 LoRa gateways --(serial / UDP, one JSON line per frame heard)--> OnaCore
        1. re-check every frame's signature with the ONA's own key
        2. 2-of-3 vote per (type, origin, seq)                      quorum.py
        3. robots: 3 range spheres + EKF + frame translation        tracker.py
        4. persistent queue, most important first                   store.py
        5. LTE, else satellite, to the Command Post                 uplink.py
    Command Post --(mission dispatch)--> OnaCore --> signed MISSION frames --> gateways --> Executor

OnaCore holds the logic only: no sockets, no threads, an injectable clock. The
I/O (serial ports, UDP, HTTP polling) is in server.py. That makes every rule
here testable without a radio or a network.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from . import lmb2
from .geodesy import AnchorPoint, LocalToGpsCalibrator
from .quorum import CONFIRMED, CONFLICT, PENDING, QuorumVoter
from .store import PersistentQueue
from .tracker import Reception, RobotTrack, TrackerParams

HAZARD_KINDS = {'VICTIM', 'GAS', 'RADIATION', 'THERMAL', 'STRUCTURAL', 'OBSTRUCTION'}
URGENT_KINDS = {'VICTIM', 'GAS', 'RADIATION', 'THERMAL'}
# mine scenario: resources are marked and forwarded, but always last (never over satellite)
RESOURCE_KINDS = {'PHOSPHATE', 'GOLD', 'GEMSTONE'}


# ---------------------------------------------------------------------- config
@dataclass
class OnaConfig:
    key: bytes = lmb2.KEY_DEMO
    net_id: int = lmb2.NET_DEFAULT
    quorum: int = 2
    gateways: dict = field(default_factory=dict)        # gw -> ENU np.array(3)
    gateways_gps: dict = field(default_factory=dict)    # gw -> (lat, lon, alt)
    calibrator: Optional[LocalToGpsCalibrator] = None
    tracker: TrackerParams = field(default_factory=TrackerParams)
    unconfirmed_hold_s: float = 4.0     # a record one gateway reported is shown as unconfirmed after this
    strict: bool = False                # True: never show unconfirmed records
    status_period_s: float = 2.0
    conflict_hold_s: float = 10.0       # a 1-vs-1 disagreement must last this long to raise an alert
    mission_retx_s: float = 8.0
    mission_max_tx: int = 12
    time_scale: float = 1.0
    executor_id: Optional[int] = None   # robot id to brief (None = any Executor)
    name: str = 'ONA'
    site: Optional[dict] = None         # e.g. the Gafsa mine: {'id', 'name', 'plan', 'underground'} for the dashboard

    @staticmethod
    def from_dict(d: dict) -> 'OnaConfig':
        cfg = OnaConfig()
        cfg.key = lmb2.parse_key(d.get('key', 'demo'))
        cfg.net_id = int(d.get('net_id', lmb2.NET_DEFAULT))
        cfg.quorum = int(d.get('quorum', 2))
        cfg.unconfirmed_hold_s = float(d.get('unconfirmed_hold_s', cfg.unconfirmed_hold_s))
        cfg.strict = bool(d.get('strict', False))
        cfg.time_scale = float(d.get('time_scale', 1.0))
        cfg.executor_id = d.get('executor_id')
        cfg.name = d.get('name', 'ONA')
        cfg.site = d.get('site')
        a = d.get('anchor', {'lat': 36.8065, 'lon': 10.1815, 'alt': 10.0, 'map_yaw_deg': 0.0})
        if d.get('entrance_anchors'):
            anchors = [AnchorPoint(np.array(x['local'], dtype=float), x['lat'], x['lon'], x.get('alt', 0.0),
                                   x.get('weight', 1.0)) for x in d['entrance_anchors']]
            cfg.calibrator = LocalToGpsCalibrator(anchors, planar=True)
        else:
            cfg.calibrator = LocalToGpsCalibrator.from_anchor_yaw(a['lat'], a['lon'], a.get('alt', 0.0),
                                                                  a.get('map_yaw_deg', 0.0), planar=True)
        tp = d.get('tracker', {})
        cfg.tracker = TrackerParams(**{k: v for k, v in tp.items() if k in TrackerParams.__dataclass_fields__})
        cal = cfg.calibrator
        for gw, g in d.get('gateways', {}).items():
            if 'local' in g:
                enu = cal.local_to_enu(np.array(g['local'], dtype=float))
                lat, lon, alt = cal.enu_to_gps(enu)
            else:
                lat, lon, alt = g['lat'], g['lon'], g.get('alt', cal.ref_anchor.alt)
                enu = cal.gps_to_enu(lat, lon, alt)
            cfg.gateways[gw] = np.asarray(enu, dtype=float)
            cfg.gateways_gps[gw] = (lat, lon, alt)
        return cfg

    @staticmethod
    def load(path: str) -> 'OnaConfig':
        with open(path, encoding='utf-8') as f:
            return OnaConfig.from_dict(json.load(f))


def seq_newer(a: int, b: int) -> bool:
    """True when sequence a is newer than b (16-bit serial-number arithmetic, as LMB2)."""
    d = (a - b) & 0xFFFF
    return 0 < d < 0x8000


@dataclass
class RecordEntry:
    origin: int
    seq: int
    rec: dict
    state: str
    votes: int
    gateways: list
    forwarded: bool = False
    conflict: bool = False
    rssi: Optional[float] = None
    snr: Optional[float] = None
    hops: int = 0
    relay: Optional[int] = None
    first_t: float = 0.0
    expired: bool = False


# ---------------------------------------------------------------------- core
class OnaCore:
    def __init__(self, cfg: OnaConfig, queue: PersistentQueue, clock: Callable[[], float] = time.time,
                 downlink: Optional[Callable[[list], None]] = None, log: Optional[Callable[[str], None]] = None):
        self.cfg = cfg
        self.q = queue
        self.clock = clock
        self.downlink = downlink or (lambda frames: None)
        self.log = log or (lambda s: None)
        self.voter = QuorumVoter(list(cfg.gateways.keys()) or ['gw1', 'gw2', 'gw3'], cfg.quorum)
        self.records: dict = {}             # origin -> RecordEntry (latest confirmed version)
        self.hints: dict = {}               # origin -> {gw: ('silent'|'alive', t)}
        self.silent: dict = {}              # origin -> bool (last forwarded)
        self.expired: dict = {}             # (origin, seq) -> gateways that saw it age out
        self.tracks: dict = {}              # robot id -> RobotTrack
        self.gw_stats: dict = {g: {} for g in self.voter.gateways}
        self.gw_rx: dict = {g: 0 for g in self.voter.gateways}
        self.mission: Optional[dict] = None
        self.mission_seq = 0
        self.last_dashboard_mission = None
        self.last_status = 0.0
        self.events_ring: list = []
        self.t_start = clock()
        self.wall_start = time.time()
        self.alerts = 0
        self.conflicts_open: dict = {}
        tp = cfg.tracker
        self.robot_h_enu = float(cfg.calibrator.t[2]) + tp.robot_height_m

    # ---------------------------------------------------------------- helpers
    def _gws_active(self) -> list:
        return [g for g in self.voter.gateways if not self.voter.score(g).quarantined]

    def _excluded(self) -> set:
        return {g for g in self.voter.gateways if self.voter.score(g).quarantined}

    def event(self, level: str, event: str, text: str, priority: int = 2, **extra) -> None:
        """Operator-visible event: audit table + Command Post feed."""
        e = self.q.audit(event, level=level, text=text, **extra)
        payload = {'t': e['ts'], 'level': level, 'event': event, 'text': text, **extra}
        self.events_ring.append(payload)
        self.events_ring = self.events_ring[-40:]
        if level == 'alert':
            self.alerts += 1
        self.q.push(payload, '/api/ona-event', priority=priority)
        self.log(f'[{level.upper()}] {event}: {text}')

    # ---------------------------------------------------------------- input
    def handle_line(self, line, default_gw: Optional[str] = None, t_rx: Optional[float] = None) -> None:
        """One JSON line from a gateway (string or dict)."""
        if isinstance(line, (bytes, bytearray)):
            line = line.decode('utf-8', 'replace')
        if isinstance(line, str):
            line = line.strip()
            if not line or line[0] != '{':
                return
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                return
        else:
            obj = line
        gw = str(obj.get('gw') or obj.get('gateway_id') or default_gw or 'gw?')
        now = float(t_rx if t_rx is not None else (obj.get('t') or self.clock()))
        self.voter.score(gw).last_heard = self.clock()
        self.gw_stats.setdefault(gw, {})
        self.gw_rx[gw] = self.gw_rx.get(gw, 0) + 1
        ev = obj.get('ev')
        if ev == 'hb':
            if obj.get('sim_speed'):
                self.cfg.time_scale = float(obj['sim_speed'])     # a simulator running faster than real time
            self.gw_stats[gw].update({k: obj[k] for k in ('known', 'rx_ok', 'rx_bad', 'digests', 'duty', 'rejected',
                                                          'reject_reasons') if k in obj})
            self.gw_stats[gw]['hb_t'] = self.clock()
            return
        if ev == 'reject':
            rr = self.gw_stats[gw].setdefault('air_rejects', {})
            reason = obj.get('reason', '?')
            rr[reason] = rr.get(reason, 0) + 1
            if reason == 'bad_mac':
                st = self.gw_stats[gw]
                st['spoof_pending'] = st.get('spoof_pending', 0) + 1
                if self.clock() - st.get('spoof_t', -1e9) >= 60.0:      # at most one alert a minute per gateway
                    n, st['spoof_pending'], st['spoof_t'] = st['spoof_pending'], 0, self.clock()
                    self.event('warn', 'SPOOF_ON_AIR', f'{gw} heard {n} frame{"s" if n > 1 else ""} with a bad '
                               f'signature on the radio (forged or tampered, rejected)', priority=2, gw=gw, count=n)
            return
        if obj.get('frame'):
            try:
                frame = bytes.fromhex(str(obj['frame']))
                tname, dec = lmb2.decode_any(frame, self.cfg.key, self.cfg.net_id)
            except (ValueError, lmb2.DecodeError) as e:
                reason = getattr(e, 'reason', 'bad_hex')
                self.voter.invalid(gw, reason, self.clock())
                sc = self.voter.score(gw)
                self.q.audit('REJECTED', gw=gw, reason=reason)
                if sc.quarantined and 'quarantine_announced' not in self.gw_stats[gw]:
                    self.gw_stats[gw]['quarantine_announced'] = True
                    self.event('alert', 'GATEWAY_QUARANTINED', f'{gw} passed on {sc.invalid} frames that fail the '
                               f'ONA signature check: its reports are ignored', gw=gw, priority=3)
                return
            if tname == 'record':
                # vote on the signed content (signature checked above by the ONA itself)
                self._on_record(gw, dec, ('R', lmb2.record_signed_fields(dec)), obj, now)
                return
            body = ('F', lmb2.sealed_part(frame))
            if tname == 'robot':
                self._on_robot(gw, dec, body, obj, now)
            elif tname == 'mission':
                self._on_mission_heard(gw, dec)
            return
        if 'beacon_id' in obj and 'kind' in obj and 'gps' in obj:     # JSON-only gateway (older firmware)
            rec = dict(obj)
            rec['kind'] = str(rec['kind']).upper()
            rec['_hdr'] = {'relay': obj.get('relay'), 'hops': obj.get('hops', 0), 'silent': bool(obj.get('silent'))}
            try:
                body = ('R', lmb2.record_signed_fields(rec))
            except (KeyError, TypeError, ValueError):
                self.voter.invalid(gw, 'bad_json', self.clock())
                return
            self._on_record(gw, rec, body, obj, now)

    # ---------------------------------------------------------------- records
    def _on_record(self, gw: str, rec: dict, body, line: dict, now: float) -> None:
        origin, seq = int(rec['beacon_id']), int(rec['seq'])
        hdr = rec.get('_hdr', {})
        if hdr.get('silent') or line.get('silent'):
            self.hints.setdefault(origin, {})[gw] = ('silent', now)
        elif (hdr.get('alive') or line.get('silent') is False
              or (hdr.get('relay') == origin and not hdr.get('hops'))):      # the beacon itself just spoke
            self.hints.setdefault(origin, {})[gw] = ('alive', now)
        rec = dict(rec)
        rec['_rx'] = {'gw': gw, 'rssi': line.get('rssi'), 'snr': line.get('snr'), 'hops': hdr.get('hops', 0),
                      'relay': hdr.get('relay'), 't': now}
        if line.get('eff_conf') is not None:          # the gateway's own clock (simulators run faster)
            rec['_rx'].update(eff_conf=float(line['eff_conf']), age_s=line.get('age_s'))
        if line.get('ev') == 'expired':
            self.expired.setdefault((origin, seq), set()).add(gw)
        for ev in self.voter.offer(gw, ('R', origin, seq), body, rec, now=self.clock()):
            v = ev.vote
            if ev.kind == 'confirmed':
                self._confirmed_record(origin, seq, v.meta.get(v.winner, rec), v, now)
            elif ev.kind == 'more_votes':
                ent = self.records.get(origin)
                if ent and ent.seq == seq:
                    ent.votes = v.count(v.winner)
                    ent.gateways = v.gateways()
                    self._forward_record(ent, 'votes', now, priority=1)
            elif ev.kind == 'disagree':
                self.event('alert', 'GATEWAY_DISAGREES', f'{ev.gw} reported a different signed version of beacon '
                           f'#{origin} seq {seq} than the other gateways', gw=ev.gw, beacon_id=origin, seq=seq,
                           priority=3)
                sc = self.voter.score(ev.gw)
                if sc.quarantined and 'quarantine_announced' not in self.gw_stats.setdefault(ev.gw, {}):
                    self.gw_stats[ev.gw]['quarantine_announced'] = True
                    self.event('alert', 'GATEWAY_QUARANTINED', f'{ev.gw} is ignored from now on: {sc.note}',
                               gw=ev.gw, priority=3)
            elif ev.kind == 'conflict':
                # often the next gateway settles it within seconds: only a conflict that lasts is an alert
                self.conflicts_open.setdefault(('R', origin, seq), self.clock())
        self._check_silence(origin, now)
        ent = self.records.get(origin)
        if (ent is not None and ent.seq == seq and ent.state == CONFIRMED and not ent.expired
                and len(self.expired.get((origin, seq), ())) >= self.cfg.quorum):
            ent.expired = True                          # aged out, by the gateways' clocks too
            self._forward_record(ent, 'expired', now, priority=1)

    def _confirmed_record(self, origin: int, seq: int, rec: dict, vote, now: float) -> None:
        ent = self.records.get(origin)
        if ent is not None and ent.state == CONFIRMED and ent.seq != seq and not seq_newer(seq, ent.seq):
            return                                        # an older version confirmed late: keep the newer
        if ent is None or not ent.forwarded:
            ev = 'new'
        elif ent.seq != seq or ent.state != CONFIRMED:
            ev = 'update'
        else:
            ev = 'votes'
        rx = rec.get('_rx', {})
        ent = RecordEntry(origin, seq, rec, CONFIRMED, vote.count(vote.winner), vote.gateways(),
                          forwarded=ent.forwarded if ent else False, rssi=rx.get('rssi'), snr=rx.get('snr'),
                          hops=rx.get('hops') or 0, relay=rx.get('relay'), first_t=now)
        self.records[origin] = ent
        kind = rec['kind']
        prio = 3 if kind in URGENT_KINDS else (2 if kind in HAZARD_KINDS or kind == 'SEARCHED' else 1)
        self._forward_record(ent, ev, now, priority=prio)
        if ev == 'new' and kind in HAZARD_KINDS:
            self.q.audit('CONFIRMED', beacon_id=origin, seq=seq, kind=kind, votes=ent.votes, gateways=ent.gateways)
        elif ev == 'new' and kind in RESOURCE_KINDS:
            what = (f'{rec.get("severity", 0) / 4.0:.1f} % P2O5' if kind == 'PHOSPHATE'
                    else 'sample to confirm')
            self.event('info', 'RESOURCE_MARKED', f'{kind.lower()} marked by beacon #{origin} ({what})',
                       priority=1, beacon_id=origin, kind=kind)
        elif ev == 'new' and kind == 'SEARCHED':
            self.event('info', 'AREA_SEARCHED', f'an area was searched ({rec.get("severity", 0)} % seen by the '
                       f'thermal camera), beacon #{origin}', priority=2, beacon_id=origin, kind=kind)

    def _forward_record(self, ent: RecordEntry, ev: str, now: float, priority: int = 1) -> None:
        rec = ent.rec
        rx = rec.get('_rx', {})
        if ev == 'expired':
            eff = 0.0
        elif rx.get('eff_conf') is not None:
            eff = rx['eff_conf']
        else:
            eff = lmb2.eff_conf(rec, now) if 'ttl_s' in rec else rec.get('confidence', 1.0)
        age = rx.get('age_s') if rx.get('age_s') is not None else max(0, round(now - rec.get('ts', now)))
        n_gw = len(self.voter.gateways)
        state = 'conflict' if ent.conflict else ('confirmed' if ent.state == CONFIRMED else 'unconfirmed')
        g = rec.get('gps', {})
        p = {
            'beacon_id': ent.origin, 'event_type': rec['kind'].lower(), 'value': rec.get('severity', 0),
            'lat': g.get('lat'), 'lon': g.get('lon'), 'timestamp': rec.get('ts'),
            'ttl': max(0, min(20, round(20 * eff))), 'gateway_id': '+'.join(ent.gateways) or 'ona',
            'rssi': ent.rssi, 'v': 2, 'ev': ev, 'kind': rec['kind'], 'seq': ent.seq,
            'severity': rec.get('severity', 0), 'confidence': rec.get('confidence'), 'eff_conf': round(eff, 3),
            'err_m': g.get('err_m'), 'gps_src': g.get('src', 'slam'), 'next': rec.get('next'),
            'ttl_s': rec.get('ttl_s'), 'half_life_s': rec.get('half_life_s'),
            'age_s': age, 'hops': ent.hops, 'relay': ent.relay,
            'snr': ent.snr, 'silent': bool(self.silent.get(ent.origin)), 'stale': bool(rec.get('stale')),
            'retracted': bool(rec.get('retracted')), 'time_scale': self.cfg.time_scale,
            'ona': {'state': state, 'votes': ent.votes, 'of': n_gw, 'quorum': self.cfg.quorum,
                    'gateways': ent.gateways},
        }
        self.q.push(p, '/api/beacon', priority=priority, coalesce_key=f'rec:{ent.origin}')
        ent.forwarded = True

    def _check_silence(self, origin: int, now: float) -> None:
        ent = self.records.get(origin)
        if ent is None:
            return
        hs = self.hints.get(origin, {})
        n_sil = sum(1 for h, t in hs.values() if h == 'silent')
        n_alv = sum(1 for h, t in hs.values() if h == 'alive')
        silent = n_sil > 0 and n_sil >= n_alv
        if silent != bool(self.silent.get(origin)):
            self.silent[origin] = silent
            if silent:
                self.event('warn', 'BEACON_LOST', f'beacon #{origin} ({ent.rec["kind"]}) is silent: probably '
                           f'destroyed. Its record is kept.', beacon_id=origin, priority=2)
            self._forward_record(ent, 'silent', now, priority=2)

    # ---------------------------------------------------------------- robots
    def _track(self, rid: int) -> RobotTrack:
        tr = self.tracks.get(rid)
        if tr is None:
            tp = TrackerParams(**{k: getattr(self.cfg.tracker, k) for k in TrackerParams.__dataclass_fields__})
            tp.robot_height_m = self.robot_h_enu
            tr = self.tracks[rid] = RobotTrack(rid, self.cfg.calibrator, self.cfg.gateways, tp)
        return tr

    def _on_robot(self, gw: str, dec: dict, body, line: dict, now: float) -> None:
        rid, seq = dec['robot_id'], dec['seq']
        tr = self._track(rid)
        if not self.voter.score(gw).quarantined:
            rng = line.get('range_m')
            tr.add_reception(seq, Reception(gw, self.clock(), None if rng is None else float(rng),
                                            line.get('range_sd'), line.get('rssi'), line.get('snr'),
                                            line.get('range_src', 'tof')))
        for ev in self.voter.offer(gw, ('P', rid, seq), body, dec, now=self.clock()):
            if ev.kind == 'disagree':
                self.event('alert', 'GATEWAY_DISAGREES', f'{ev.gw} reported a different position report from '
                           f'robot #{rid} than the other gateways', gw=ev.gw, robot_id=rid, priority=3)

    def _process_pings(self) -> None:
        n_active = len(self._gws_active())
        now = self.clock()
        for rid, tr in self.tracks.items():
            for g in tr.ready(now, n_active):
                v = self.voter.votes.get(('P', rid, g.seq))
                state = v.state if v else PENDING
                report = v.meta.get(v.winner) if (v and v.state == CONFIRMED) else None
                votes = v.count(v.winner) if (v and v.winner is not None) else (v.count() if v else 0)
                tr.time_scale = max(1e-3, float(self.cfg.time_scale or 1.0))
                p = tr.process(g, report, votes, len(self.voter.gateways), state.lower(), self._excluded())
                if 'lat' not in p:
                    continue
                self.q.push(p, '/api/robot-status', priority=2, coalesce_key=f'robot:{rid}')
                if (report or {}).get('role') == 'EXECUTOR' or tr.role == 'EXECUTOR':
                    self.q.push({'lat': p['lat'], 'lon': p['lon'], 'heading': p.get('heading', 0.0),
                                 'status': str(p.get('phase') or 'navigating').lower(), 'timestamp': time.time(),
                                 'robot_id': rid, 'sigma_m': p.get('sigma_m'), 'source': p.get('source')},
                                '/api/executor-status', priority=2, coalesce_key='executor')
                self._robot_events(rid, tr, p, report)

    def _robot_events(self, rid: int, tr: RobotTrack, p: dict, report: Optional[dict]) -> None:
        st = self.gw_stats.setdefault(f'robot{rid}', {})
        cons = p.get('consistency', {}).get('state')
        if cons and cons != st.get('cons'):
            if cons == 'drift':
                self.event('warn', 'SLAM_DRIFT', f'robot #{rid}: its own position and the gateways\' measurement '
                           f'differ by {p["consistency"]["dist_m"]} m. Trust the gateways.', robot_id=rid)
            elif cons == 'agree' and st.get('cons') == 'drift':
                self.event('info', 'SLAM_OK', f'robot #{rid}: own position agrees with the gateways again',
                           robot_id=rid, priority=1)
            st['cons'] = cons
        if report is not None:
            if report.get('emergency') and not st.get('emergency'):
                self.event('alert', 'ROBOT_EMERGENCY', f'robot #{rid} reports an emergency', robot_id=rid,
                           priority=3)
            st['emergency'] = bool(report.get('emergency'))
            m = self.mission
            if (m and report.get('mission_id') == m['mission_id'] and report.get('mission_ack')
                    and rid not in m['acked_by']):
                m['acked_by'].append(rid)
                self.event('info', 'MISSION_ACK', f'robot #{rid} ({report.get("role", "?").lower()}) received '
                           f'briefing {m["mission_id"]}: {len(m["targets"])} target(s)', robot_id=rid,
                           mission_id=m['mission_id'], priority=3)
            if m and report.get('mission_id') == m['mission_id']:
                m['progress'] = {'done': report.get('done'), 'total': report.get('total'),
                                 'phase': report.get('phase')}

    def _on_mission_heard(self, gw: str, dec: dict) -> None:
        m = self.mission
        if m is None or dec['mission_id'] != m['mission_id']:
            self.event('alert', 'UNKNOWN_BRIEFING', f'{gw} heard a signed mission briefing the ONA did not send '
                       f'(mission {dec["mission_id"]})', gw=gw, priority=3)

    # ---------------------------------------------------------------- downlink
    def dispatch_mission(self, targets, return_to_exit: bool = True, abort: bool = False,
                         robot_id: Optional[int] = None, source: str = 'command post') -> dict:
        """Command Post -> ONA -> gateways -> Executor: a signed briefing."""
        self.mission_seq += 1
        mid = (int(self.clock()) & 0x7FFF) or 1
        if self.mission and self.mission['mission_id'] == mid:
            mid = (mid + 1) & 0x7FFF or 1
        rid = robot_id if robot_id is not None else self.cfg.executor_id
        frames = lmb2.encode_mission(mid, targets, self.cfg.key, self.cfg.net_id, seq=self.mission_seq,
                                     robot_id=lmb2.NONE16 if rid is None else int(rid),
                                     return_to_exit=return_to_exit, abort=abort, in_order=True)
        self.mission = {'mission_id': mid, 'seq': self.mission_seq, 'targets': [int(t) for t in targets],
                        'frames': frames, 'created': self.clock(), 'last_tx': 0.0, 'tx_count': 0, 'acked_by': [],
                        'return_to_exit': return_to_exit, 'abort': abort, 'robot_id': rid, 'source': source,
                        'progress': None}
        self.event('info', 'MISSION_DISPATCH', f'briefing {mid} for the Executor: targets '
                   f'{" -> ".join("#" + str(t) for t in targets) or "(none)"}'
                   f'{", then back to the exit" if return_to_exit else ""} ({len(frames)} signed frame(s))',
                   mission_id=mid, targets=[int(t) for t in targets], priority=3)
        self._tx_mission(force=True)
        return self.mission

    def _tx_mission(self, force: bool = False) -> None:
        m = self.mission
        if m is None:
            return
        now = self.clock()
        if m['acked_by'] and not force:
            return                         # the Executor confirmed it: stop repeating
        if not force and (now - m['last_tx'] < self.cfg.mission_retx_s or m['tx_count'] >= self.cfg.mission_max_tx):
            return
        m['last_tx'] = now
        m['tx_count'] += 1
        self.downlink(m['frames'])
        self.q.audit('MISSION_TX', mission_id=m['mission_id'], tx=m['tx_count'])

    def on_dashboard_mission(self, m) -> Optional[dict]:
        """The dashboard's latest dispatched mission (GET /api/mission-dispatch/latest)."""
        if not m or not isinstance(m, dict):
            return None
        stamp = m.get('dispatched_at') or m.get('received_at')
        if stamp is None or stamp == self.last_dashboard_mission:
            return None
        first = self.last_dashboard_mission is None
        self.last_dashboard_mission = stamp
        try:
            if first and float(stamp) < self.wall_start - 5.0:
                self.q.audit('MISSION_SKIPPED', reason='dispatched before the ONA started', dispatched_at=stamp)
                return None                 # an old mission left on the dashboard: never replay it
        except (TypeError, ValueError):
            pass
        ids = []
        for w in m.get('waypoints', []):
            b = w.get('beacon_id') if isinstance(w, dict) else w
            try:
                ids.append(int(b))
            except (TypeError, ValueError):
                continue
        return self.dispatch_mission(ids, return_to_exit=bool(m.get('return_to_exit', True)),
                                     abort=bool(m.get('abort', False)), source='dashboard')

    # ---------------------------------------------------------------- periodic
    def tick(self, link_state: str = 'LTE', force_status: bool = False) -> None:
        now = self.clock()
        self._link_change(link_state)
        self._process_pings()
        if not self.cfg.strict:
            for v in self.voter.pending(self.cfg.unconfirmed_hold_s, now):
                if v.key[0] != 'R' or v.announced:
                    continue
                v.announced = 1
                origin, seq = v.key[1], v.key[2]
                ent = self.records.get(origin)
                if ent is not None and ent.state == CONFIRMED and not seq_newer(seq, ent.seq):
                    continue
                body = next(iter(v.bodies))
                rec = v.meta.get(body)
                if rec is None:
                    continue
                rx = rec.get('_rx', {})
                pend = RecordEntry(origin, seq, rec, PENDING, v.count(), v.gateways(), rssi=rx.get('rssi'),
                                   snr=rx.get('snr'), hops=rx.get('hops') or 0, relay=rx.get('relay'), first_t=now)
                if ent is None or ent.state != CONFIRMED:
                    self.records[origin] = pend
                # the Command Post keeps a confirmed version on the map and shows this one as pending
                self._forward_record(pend, 'new' if ent is None else 'update', rx.get('t', now), priority=1)
                self.q.audit('UNCONFIRMED', beacon_id=origin, seq=seq, gateways=pend.gateways)
        for key, t0 in list(self.conflicts_open.items()):
            v = self.voter.votes.get(key)
            if v is None or v.state != CONFLICT:
                del self.conflicts_open[key]
            elif now - t0 >= self.cfg.conflict_hold_s:
                del self.conflicts_open[key]
                origin, seq = key[1], key[2]
                self.event('alert', 'RECORD_CONFLICT', f'beacon #{origin} seq {seq}: {len(v.bodies)} different '
                           f'signed versions and no majority after {self.cfg.conflict_hold_s:.0f} s (a gateway is '
                           f'lying, or the mission key has leaked)', beacon_id=origin, seq=seq,
                           gateways=v.gateways(), priority=3)
                ent = self.records.get(origin)
                if ent:
                    ent.conflict = True
                    self._forward_record(ent, 'conflict', now, priority=3)
        self.voter.sweep(now)
        self._tx_mission()
        period = self.cfg.status_period_s if link_state != 'SATELLITE' else max(10.0, self.cfg.status_period_s)
        if force_status or now - self.last_status >= period:
            self.last_status = now
            self.push_status(link_state)

    def _link_change(self, link_state: str) -> None:
        """Tell the Command Post when the uplink switches (this event goes over the satellite too)."""
        prev = getattr(self, 'link_seen', None)
        if link_state == prev or (prev is None and link_state == 'NONE'):
            return
        self.link_seen = link_state
        if prev is None:
            return
        if link_state == 'SATELLITE':
            self.event('warn', 'LINK_CHANGE', 'LTE lost: only hazards, alerts and the Executor position go out, '
                       'over the satellite (Iridium SBD, 340 bytes a message). The rest waits in the queue.',
                       link=link_state, prev=prev, priority=3)
        elif link_state == 'NONE':
            self.event('alert', 'LINK_CHANGE', 'no uplink at all: everything waits in the on-disk queue',
                       link=link_state, prev=prev, priority=3)
        else:
            self.event('info', 'LINK_CHANGE', f'{link_state} back: everything queued during the outage goes out, '
                       f'most important first', link=link_state, prev=prev, priority=3)

    # ---------------------------------------------------------------- status
    def status(self, link_state: str = 'LTE', uplink=None) -> dict:
        now = self.clock()
        gws = []
        for gw in self.voter.gateways:
            sc = self.voter.score(gw).as_dict()
            lat, lon, alt = self.cfg.gateways_gps.get(gw, (None, None, None))
            st = self.gw_stats.get(gw, {})
            last = self.voter.score(gw).last_heard
            gws.append({**sc, 'lat': lat, 'lon': lon, 'alt': alt,
                        'alive': bool(last and now - last < 15), 'last_heard_s': round(now - last, 1) if last else None,
                        'lines': self.gw_rx.get(gw, 0), 'known': st.get('known'), 'duty': st.get('duty'),
                        'air_rejects': st.get('air_rejects', {})})
        cal = self.cfg.calibrator
        checks = [t.calib_check for t in self.tracks.values() if t.calib_check]
        m = self.mission
        return {
            'name': self.cfg.name, 't': time.time(), 'uptime_s': round(now - self.t_start, 1),
            'link': {'state': link_state, **self.q.counts()},
            'gateways': gws, 'votes': {k: v for k, v in self.voter.summary().items() if k != 'scores'},
            'calibration': {'rms_m': round(cal.calib_rms_m, 3), 'yaw_deg': round(math.degrees(cal.yaw_rad), 2),
                            'anchor': {'lat': cal.ref_anchor.lat, 'lon': cal.ref_anchor.lon},
                            'in_mission_check': checks[-1] if checks else None},
            'robots': [{'robot_id': rid, 'role': t.role, 'pings': t.processed,
                        'consistency': (t.last_payload or {}).get('consistency', {}).get('state')}
                       for rid, t in self.tracks.items()],
            'mission': None if m is None else {k: m[k] for k in ('mission_id', 'seq', 'targets', 'tx_count',
                                                                  'acked_by', 'return_to_exit', 'progress',
                                                                  'source')},
            'records': {'confirmed': sum(1 for e in self.records.values() if e.state == CONFIRMED),
                        'unconfirmed': sum(1 for e in self.records.values() if e.state != CONFIRMED),
                        'conflicts': sum(1 for e in self.records.values() if e.conflict)},
            'alerts': self.alerts, 'events': self.events_ring[-12:], 'quorum': self.cfg.quorum,
            'site': self.cfg.site,
        }

    def push_status(self, link_state: str = 'LTE') -> None:
        st = self.status(link_state)
        self.q.push(st, '/api/ona-status', priority=1, coalesce_key='ona-status')
        for g in st['gateways']:
            self.q.push({'gateway_id': g['gw'], 'status': 'online' if g['alive'] else 'offline',
                         'timestamp': time.time(), 'known': g.get('known'), 'rx_ok': g['lines'],
                         'rejected': g['invalid'] + sum(g['air_rejects'].values()),
                         'reject_reasons': g['air_rejects'], 'duty': g.get('duty'), 'lat': g['lat'], 'lon': g['lon'],
                         'agreement': g['agreement'], 'quarantined': g['quarantined'], 'via': 'ona'},
                        '/api/network-health', priority=1, coalesce_key=f'health:{g["gw"]}')
