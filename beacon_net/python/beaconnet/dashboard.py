"""
beaconnet.dashboard - map beacon records onto the Command Post data contract.

The dashboard contract (POST /api/beacon) predates LMB2:
    {beacon_id, event_type, value, lat, lon, timestamp, ttl (0..20), gateway_id, rssi}
We keep every contract field meaningful for an old dashboard, and add the
full v2 record next to it (the v2 dashboard uses those):

    event_type  kind in lower case ("victim", "gas", ...)
    value       severity (0..255, 0..100 recommended)
    ttl         20 x effective confidence  -> old dashboards fade correctly
    v           2
    kind, seq, severity, confidence, eff_conf, err_m, gps_src, next{id,dist_m,bearing_deg},
    ttl_s, half_life_s, age_s, hops, relay, silent, stale, retracted, ev
"""
from __future__ import annotations

import json
import threading
import time
import queue
import urllib.request
from typing import Optional

from . import proto


def payload_from_gateway_line(d: dict, gateway_id: str) -> dict:
    """A gateway JSON event line (record + rx meta) -> /api/beacon body."""
    gps = d.get("gps") or {}
    eff = d.get("eff_conf")
    if eff is None:
        eff = d.get("confidence", 1.0)
    if d.get("ev") == "expired":
        eff = 0.0
    kind = str(d.get("kind", "WAYPOINT"))
    return {
        # --- contract fields
        "beacon_id": d.get("beacon_id"),
        "event_type": kind.lower(),
        "value": d.get("severity", 0),
        "lat": gps.get("lat"),
        "lon": gps.get("lon"),
        "timestamp": d.get("ts", time.time()),
        "ttl": max(0, min(20, round(20 * float(eff)))),
        "gateway_id": gateway_id,
        "rssi": d.get("rssi"),
        # --- LMB2 extras
        "v": 2,
        "ev": d.get("ev", "new"),
        "kind": kind,
        "seq": d.get("seq"),
        "severity": d.get("severity", 0),
        "confidence": d.get("confidence"),
        "eff_conf": round(float(eff), 3),
        "err_m": gps.get("err_m"),
        "gps_src": gps.get("src", "slam"),
        "next": d.get("next"),
        "ttl_s": d.get("ttl_s"),
        "half_life_s": d.get("half_life_s"),
        "age_s": d.get("age_s"),
        "hops": d.get("hops"),
        "relay": d.get("relay"),
        "snr": d.get("snr"),
        "silent": bool(d.get("silent", False)),
        "stale": bool(d.get("stale", False)),
        "retracted": bool(d.get("retracted", False)),
    }


def payload_from_record(rec: proto.Record, gateway_id: str, now: Optional[int] = None,
                        ev: str = "new", **meta) -> dict:
    """A Record known locally (e.g. the Writer's own provisioning) -> /api/beacon body."""
    now = int(time.time()) if now is None else now
    d = proto.record_to_dict(rec, now=now)
    d["ev"] = ev
    d.update(meta)
    return payload_from_gateway_line(d, gateway_id)


class Poster:
    """Fire-and-forget JSON POSTs on a worker thread (never blocks the caller)."""

    def __init__(self, server: str, log=print):
        self.server = server.rstrip("/")
        self.log = log
        self.q: "queue.Queue[tuple]" = queue.Queue(maxsize=1000)
        self.sent = 0
        self.failed = 0
        threading.Thread(target=self._run, daemon=True).start()

    def post(self, path: str, body: dict) -> None:
        try:
            self.q.put_nowait((path, body))
        except queue.Full:
            self.failed += 1

    def flush(self, timeout: float = 5.0) -> None:
        end = time.time() + timeout
        while not self.q.empty() and time.time() < end:
            time.sleep(0.05)
        time.sleep(0.1)

    def _run(self) -> None:
        warned = False
        while True:
            path, body = self.q.get()
            req = urllib.request.Request(self.server + path, data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"})
            try:
                urllib.request.urlopen(req, timeout=3).read()
                self.sent += 1
                warned = False
            except Exception as e:  # noqa: BLE001 - network errors of any kind
                self.failed += 1
                if not warned:
                    self.log(f"[dashboard] POST {path} failed: {e} (is the Command Post running?)")
                    warned = True
