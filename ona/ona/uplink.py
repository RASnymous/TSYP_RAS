"""
GALERIA - The Living Map
The ONA's link to the distant Command Post: LTE first, satellite as a backup.

Origin: ona.py (GALERIA team), "4. Liaisons montantes". LinkType, the
UplinkTransport interface, SimulatedTransport and choose_link are that code.
v9 adds a real transport to the Command Post dashboard (HTTP), a link model
for demos (LTE outages, satellite on/off), the satellite limits (Iridium SBD:
340-byte messages, a few seconds of latency, only urgent traffic) and the
thread that empties the persistent queue whenever a link is up.

Rule kept from the original design: only the ONA has both a LoRa side (its
gateways) and a network side (LTE / satellite). The robots only have LoRa and
the Command Post has no LoRa radio.
"""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from enum import Enum
from typing import Callable, Optional

from .store import PersistentQueue

MQTT_TOPIC_UP = 'galeria/uplink'
MQTT_TOPIC_DOWN = 'galeria/mission'
SBD_MAX_BYTES = 340          # Iridium Short Burst Data, mobile-originated message


class LinkType(Enum):
    LTE = 'LTE'
    SATELLITE = 'SATELLITE'
    NONE = 'NONE'


# ---------------------------------------------------------------------------
# original interface
# ---------------------------------------------------------------------------
class UplinkTransport:
    """The two modems. Replace with real drivers (LTE AT commands, Iridium SBD)."""

    def lte_available(self) -> bool:
        raise NotImplementedError

    def satellite_available(self) -> bool:
        raise NotImplementedError

    def send(self, endpoint: str, payload: dict, via: LinkType) -> bool:
        raise NotImplementedError

    # original name, kept
    def send_mqtt_tls(self, topic: str, payload: dict, via: LinkType) -> bool:
        return self.send(topic, payload, via)


class SimulatedTransport(UplinkTransport):
    """Test double: prints what it would send."""

    def __init__(self, lte_up: bool = True, sat_up: bool = True, log: Optional[Callable] = print):
        self._lte_up, self._sat_up = lte_up, sat_up
        self.sent = []
        self.log = log

    def lte_available(self) -> bool:
        return self._lte_up

    def satellite_available(self) -> bool:
        return self._sat_up

    def send(self, endpoint: str, payload: dict, via: LinkType) -> bool:
        self.sent.append((endpoint, payload, via))
        if self.log:
            self.log(f'[{via.value}] -> {endpoint} {json.dumps(payload)[:160]}')
        return True


def choose_link(transport: UplinkTransport) -> LinkType:
    """LTE as the main link, satellite (Iridium SBD) as the backup."""
    if transport.lte_available():
        return LinkType.LTE
    if transport.satellite_available():
        return LinkType.SATELLITE
    return LinkType.NONE


# ---------------------------------------------------------------------------
# v9: link model + HTTP transport to the Command Post
# ---------------------------------------------------------------------------
def parse_windows(spec: str) -> list:
    """'60-150,300-320' -> [(60, 150), (300, 320)] seconds after start."""
    out = []
    for part in (spec or '').split(','):
        part = part.strip()
        if not part:
            continue
        a, b = part.split('-')
        out.append((float(a), float(b)))
    return out


class LinkModel:
    """Which links are up, as a function of time (for demos and tests).

    lte / sat: True (up), False (never), plus outage windows in seconds after start."""

    def __init__(self, lte: bool = True, sat: bool = True, lte_outages: str = '', sat_outages: str = '',
                 clock: Callable[[], float] = time.time):
        self.clock = clock
        self.t0 = clock()
        self.lte, self.sat = lte, sat
        self.lte_out = parse_windows(lte_outages)
        self.sat_out = parse_windows(sat_outages)
        self.forced = {}          # link -> bool, set by the operator (ONA console)

    def _up(self, name: str, base: bool, outs: list) -> bool:
        if name in self.forced:
            return self.forced[name]
        if not base:
            return False
        t = self.clock() - self.t0
        return not any(a <= t < b for a, b in outs)

    def lte_up(self) -> bool:
        return self._up('lte', self.lte, self.lte_out)

    def sat_up(self) -> bool:
        return self._up('sat', self.sat, self.sat_out)


def compact_for_satellite(endpoint: str, payload: dict) -> dict:
    """Keep what the Command Post needs, drop what it can rebuild (fits one SBD message)."""
    # (the Command Post rebuilds event_type, value, ttl and eff_conf from kind, severity, confidence,
    #  timestamp and half-life)
    keep = {'/api/beacon': ('beacon_id', 'kind', 'seq', 'lat', 'lon', 'severity', 'timestamp', 'v', 'ev',
                            'confidence', 'err_m', 'next', 'ttl_s', 'half_life_s', 'stale', 'retracted',
                            'silent', 'ona'),
            '/api/robot-status': ('robot_id', 'role', 'phase', 'lat', 'lon', 'heading', 'sigma_m', 'timestamp',
                                  'mission_id', 'done', 'total', 'consistency', 'source', 'emergency'),
            '/api/executor-status': ('lat', 'lon', 'heading', 'status', 'timestamp')}.get(endpoint)
    if keep is None:
        return payload
    out = {k: payload[k] for k in keep if k in payload}
    if 'ona' in out and isinstance(out['ona'], dict):
        o = out['ona']
        out['ona'] = {k: o[k] for k in ('state', 'votes', 'of', 'quorum') if k in o}
    if isinstance(out.get('next'), dict):
        out['next'] = {k: out['next'][k] for k in ('id', 'dist_m') if k in out['next']}
    for k in ('stale', 'retracted', 'silent'):
        if out.get(k) is False:
            del out[k]
    out['via'] = 'sat'
    return out


class CommandPostHttp(UplinkTransport):
    """Posts to the Command Post dashboard (Node.js) over HTTP.

    Link availability comes from a LinkModel (the radio side of LTE / satellite);
    an HTTP error counts as a failed send and the message stays queued."""

    def __init__(self, base_url: str, links: LinkModel, timeout_s: float = 3.0):
        self.base = base_url.rstrip('/')
        self.links = links
        self.timeout = timeout_s
        self.last_error = ''
        self.bytes_sent = {'LTE': 0, 'SATELLITE': 0}
        self.unsupported = set()

    def lte_available(self) -> bool:
        return self.links.lte_up()

    def satellite_available(self) -> bool:
        return self.links.sat_up()

    def _post(self, endpoint: str, payload: dict) -> bool:
        data = json.dumps(payload, separators=(',', ':')).encode()
        req = urllib.request.Request(self.base + endpoint, data=data, headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                r.read()
                ok = 200 <= r.status < 300
        except urllib.error.HTTPError as e:
            self.last_error = f'HTTP {e.code} on {endpoint}'
            if e.code == 404:             # an older Command Post without this endpoint: drop, don't block
                self.unsupported.add(endpoint)
                return True
            return e.code == 400          # the dashboard refused it: never retry a malformed message
        except Exception as e:  # noqa: BLE001 (network down, refused, timeout)
            self.last_error = f'{type(e).__name__}: {e}'
            return False
        if ok:
            self.last_error = ''
        return ok

    def send(self, endpoint: str, payload: dict, via: LinkType) -> bool:
        if via == LinkType.SATELLITE:
            payload = compact_for_satellite(endpoint, payload)
            size = len(json.dumps(payload, separators=(',', ':')).encode())
            if size > SBD_MAX_BYTES:
                self.last_error = f'{size} B > SBD limit {SBD_MAX_BYTES} B'
        ok = self._post(endpoint, payload)
        if ok:
            self.bytes_sent[via.value] = self.bytes_sent.get(via.value, 0) + len(json.dumps(payload))
        return ok

    def get(self, endpoint: str) -> Optional[object]:
        """Downlink (Command Post -> ONA), over whichever link is up."""
        if choose_link(self) == LinkType.NONE:
            return None
        try:
            with urllib.request.urlopen(self.base + endpoint, timeout=self.timeout) as r:
                return json.loads(r.read().decode() or 'null')
        except Exception as e:  # noqa: BLE001
            self.last_error = f'{type(e).__name__}: {e}'
            return None


class Uplink:
    """Empties the persistent queue over the best link available.

    LTE: everything, most important first. Satellite: only priority >= sat_min_priority,
    at most one message every sat_interval_s (an SBD modem is slow and billed per message)."""

    def __init__(self, queue: PersistentQueue, transport: UplinkTransport, audit: Callable = None,
                 sat_min_priority: int = 2, sat_interval_s: float = 1.0, period_s: float = 0.25,
                 batch: int = 40):
        self.q = queue
        self.t = transport
        self.audit = audit or (lambda *a, **k: None)
        self.sat_min_priority = sat_min_priority
        self.sat_interval_s = sat_interval_s
        self.period_s = period_s
        self.batch = batch
        self.state = LinkType.NONE
        self._stop = threading.Event()
        self._last_sat = 0.0
        self._last_state = None
        self._fail_until = 0.0
        self.sent = 0
        self.failed = 0

    def flush_once(self) -> int:
        """One pass (the original flush_queue). Returns the number of messages sent."""
        link = choose_link(self.t)
        self.state = link
        if link != self._last_state:
            self.audit('LINK', link=link.value)
            self._last_state = link
        if link == LinkType.NONE:
            return 0
        now = time.time()
        if now < self._fail_until:
            return 0
        n = 0
        if link == LinkType.SATELLITE:
            if now - self._last_sat < self.sat_interval_s:
                return 0
            rows = self.q.pending(limit=1, min_priority=self.sat_min_priority)
        else:
            rows = self.q.pending(limit=self.batch)
        for row_id, endpoint, payload, prio in rows:
            if self.t.send(endpoint, payload, via=link):
                self.q.mark_sent(row_id, link.value)
                self.sent += 1
                n += 1
                if link == LinkType.SATELLITE:
                    self._last_sat = time.time()
            else:
                self.q.mark_failed(row_id)
                self.failed += 1
                self._fail_until = time.time() + 1.0      # back off a second, keep the message
                break
        return n

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                self.flush_once()
            except Exception as e:  # noqa: BLE001 - the uplink thread must never die
                self.audit('UPLINK_ERROR', error=str(e))
                time.sleep(1.0)
            self._stop.wait(self.period_s)

    def start(self) -> threading.Thread:
        th = threading.Thread(target=self.run, name='ona-uplink', daemon=True)
        th.start()
        return th

    def stop(self) -> None:
        self._stop.set()
