"""
GALERIA - The Living Map
Partie 6 : ONA (Outside Network Area)

Rôle double :
    1. Traducteur de coordonnées (voir coord_transform.py, section 5)
    2. Passerelle radio UNIQUE entre l'intérieur (LoRa) et l'extérieur
       (MQTT/TLS vers le poste de commandement).

Flux (sens montant, robot -> poste de commandement) :
    Réception LoRa -> vérification (CRC, HMAC) -> dédoublonnage
    -> traduction locale -> GPS -> mise en file persistante (SQLite)
    -> envoi MQTT/TLS dès qu'un lien est disponible (LTE, sinon satellite)

Flux (sens descendant, poste de commandement -> Executor) :
    Réception mission signée (MQTT) -> retransmission radio (LoRa)

Règle stricte : seule l'ONA possède à la fois une interface LoRa et une
interface réseau (LTE/Satellite). Les robots n'ont que du LoRa, le poste
de commandement n'a aucune radio LoRa.
"""

from __future__ import annotations
import hashlib
import hmac
import json
import queue
import sqlite3
import struct
import threading
import time
import zlib
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from ekf import LocalToGpsCalibrator

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
HMAC_KEY = b"replace-with-mission-key"   # clé partagée robots <-> ONA
DB_PATH = "ona_queue.sqlite3"
MQTT_TOPIC_UP = "galeria/uplink"
MQTT_TOPIC_DOWN = "galeria/mission"


class LinkType(Enum):
    LTE = "LTE"
    SATELLITE = "SATELLITE"
    NONE = "NONE"


# ---------------------------------------------------------------------------
# 1. Message balise/robot reçu en LoRa : vérification CRC + HMAC
#
# Contrainte radio : trame complète (payload + CRC + HMAC) <= 41 octets.
# Pour tenir cette taille, chaque champ est quantifié au minimum utile :
#   - kind        : enum 1 octet au lieu d'une chaîne "VICTIM"/"GAS"/...
#   - position    : entiers 16 bits signés en CENTIMETRES (portée ±327 m,
#                   résolution 1 cm, largement suffisant pour une balise)
#   - confidence  : 1 octet (0-255) au lieu d'un float 4 octets
#   - dist_anchor : 1 octet, en mètres (0-255 m entre deux recalages)
#   - timestamp   : uint32 (secondes Unix), pas besoin d'un double 8 octets
#   - CRC16 (au lieu de CRC32) + HMAC tronqué à 6 octets
# ---------------------------------------------------------------------------

class BeaconKind(Enum):
    VICTIM = 0
    GAS = 1
    OBSTRUCTION = 2


# format binaire du corps du message (avant CRC+HMAC)
# B=beacon_id(0-255) B=kind h=x_cm h=y_cm h=z_cm B=severity B=confidence*255
# B=next_id B=dist_anchor_m H=ttl_s I=timestamp_unix
_STRUCT_FMT = "!BB hhh BBBB H I"

_CRC16_POLY_TABLE = None  # calculé une fois, à la volée (évite une dépendance externe)


def _crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    """CRC16-CCITT, moins coûteux en octets que CRC32 (2 octets au lieu de 4)."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


@dataclass
class BeaconMessage:
    beacon_id: int             # 0-255
    kind: str                  # "VICTIM" | "GAS" | "OBSTRUCTION"
    local_x: float             # m (quantifié en cm sur le fil)
    local_y: float
    local_z: float
    severity: int              # 0-255
    confidence: float          # 0.0-1.0 (quantifié sur 1 octet)
    next_id: int                # 0-255
    dist_since_anchor_m: float  # arrondi au mètre sur le fil
    ttl_s: int                  # 0-65535 s
    timestamp: float            # secondes Unix (tronqué à la seconde)

    def to_bytes(self) -> bytes:
        """Sérialisation compacte : corps binaire fixe, sans champ de longueur."""
        return struct.pack(
            _STRUCT_FMT,
            self.beacon_id & 0xFF,
            BeaconKind[self.kind].value,
            int(round(self.local_x * 100)),   # m -> cm, int16 signé
            int(round(self.local_y * 100)),
            int(round(self.local_z * 100)),
            self.severity & 0xFF,
            int(round(self.confidence * 255)) & 0xFF,
            self.next_id & 0xFF,
            min(int(round(self.dist_since_anchor_m)), 255),
            self.ttl_s & 0xFFFF,
            int(self.timestamp) & 0xFFFFFFFF,
        )


def build_wire_frame(msg: BeaconMessage) -> bytes:
    """Trame radio complète : payload + CRC16 + HMAC-SHA256 tronqué à 6 octets."""
    payload = msg.to_bytes()
    crc = _crc16_ccitt(payload)
    frame_wo_mac = payload + struct.pack("!H", crc)
    mac = hmac.new(HMAC_KEY, frame_wo_mac, hashlib.sha256).digest()[:6]
    return frame_wo_mac + mac


def verify_and_parse_frame(frame: bytes) -> Optional[BeaconMessage]:
    """Vérifie HMAC puis CRC16 ; retourne None si la trame est corrompue ou falsifiée."""
    if len(frame) < 6:
        return None

    mac_received = frame[-6:]
    frame_wo_mac = frame[:-6]
    mac_expected = hmac.new(HMAC_KEY, frame_wo_mac, hashlib.sha256).digest()[:6]
    if not hmac.compare_digest(mac_received, mac_expected):
        return None  # authentification échouée -> trame rejetée

    payload, crc_received = frame_wo_mac[:-2], struct.unpack("!H", frame_wo_mac[-2:])[0]
    if _crc16_ccitt(payload) != crc_received:
        return None  # intégrité échouée -> trame rejetée

    (beacon_id, kind_code, x_cm, y_cm, z_cm, severity, conf_q,
     next_id, dist_m, ttl, ts) = struct.unpack(_STRUCT_FMT, payload)

    return BeaconMessage(
        beacon_id=beacon_id,
        kind=BeaconKind(kind_code).name,
        local_x=x_cm / 100.0,
        local_y=y_cm / 100.0,
        local_z=z_cm / 100.0,
        severity=severity,
        confidence=conf_q / 255.0,
        next_id=next_id,
        dist_since_anchor_m=float(dist_m),
        ttl_s=ttl,
        timestamp=float(ts),
    )


# ---------------------------------------------------------------------------
# 2. Dédoublonnage (plusieurs relais gossip peuvent livrer le même message)
# ---------------------------------------------------------------------------

class Deduplicator:
    def __init__(self, window_s: float = 300.0):
        self._seen: dict[int, float] = {}   # beacon_id -> dernier timestamp accepté
        self._window_s = window_s
        self._lock = threading.Lock()

    def is_duplicate(self, msg: BeaconMessage) -> bool:
        with self._lock:
            now = time.time()
            # purge des entrées trop anciennes
            self._seen = {k: v for k, v in self._seen.items() if now - v < self._window_s}
            last = self._seen.get(msg.beacon_id)
            if last is not None and abs(msg.timestamp - last) < 1.0:
                return True
            self._seen[msg.beacon_id] = msg.timestamp
            return False


# ---------------------------------------------------------------------------
# 3. File persistante SQLite (ne rien perdre en cas de coupure réseau)
# ---------------------------------------------------------------------------

class PersistentQueue:
    def __init__(self, db_path: str = DB_PATH):
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._lock = threading.Lock()
        self._conn.execute("""
            CREATE TABLE IF NOT EXISTS uplink_queue (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                payload_json TEXT NOT NULL,
                created_at REAL NOT NULL,
                sent INTEGER NOT NULL DEFAULT 0
            )
        """)
        self._conn.commit()

    def push(self, payload: dict) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT INTO uplink_queue (payload_json, created_at, sent) VALUES (?, ?, 0)",
                (json.dumps(payload), time.time()),
            )
            self._conn.commit()

    def pending(self, limit: int = 50) -> list[tuple[int, dict]]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT id, payload_json FROM uplink_queue WHERE sent = 0 "
                "ORDER BY id ASC LIMIT ?", (limit,)
            ).fetchall()
        return [(row_id, json.loads(pj)) for row_id, pj in rows]

    def mark_sent(self, row_id: int) -> None:
        with self._lock:
            self._conn.execute("UPDATE uplink_queue SET sent = 1 WHERE id = ?", (row_id,))
            self._conn.commit()


# ---------------------------------------------------------------------------
# 4. Liaisons montantes : LTE en principal, Iridium/satellite en secours
# ---------------------------------------------------------------------------

class UplinkTransport:
    """
    Abstraction des deux modems. À remplacer par les vrais pilotes
    (AT commands LTE, SBD Iridium). Ici : interface + logique de bascule.
    """

    def lte_available(self) -> bool:
        raise NotImplementedError

    def satellite_available(self) -> bool:
        raise NotImplementedError

    def send_mqtt_tls(self, topic: str, payload: dict, via: LinkType) -> bool:
        raise NotImplementedError


class SimulatedTransport(UplinkTransport):
    """Implémentation factice pour tests / démo (Phase 1 = simulation)."""

    def __init__(self, lte_up: bool = True, sat_up: bool = True):
        self._lte_up = lte_up
        self._sat_up = sat_up

    def lte_available(self) -> bool:
        return self._lte_up

    def satellite_available(self) -> bool:
        return self._sat_up

    def send_mqtt_tls(self, topic: str, payload: dict, via: LinkType) -> bool:
        print(f"[{via.value}] -> MQTT/TLS topic={topic} payload={payload}")
        return True  # simuler un envoi réussi


def choose_link(transport: UplinkTransport) -> LinkType:
    """LTE en lien principal, satellite (Iridium SBD) en secours."""
    if transport.lte_available():
        return LinkType.LTE
    if transport.satellite_available():
        return LinkType.SATELLITE
    return LinkType.NONE


# ---------------------------------------------------------------------------
# 5. Passerelle ONA complète
# ---------------------------------------------------------------------------

class ONAGateway:
    def __init__(self, calibrator: LocalToGpsCalibrator, transport: UplinkTransport,
                 db_path: str = DB_PATH):
        self.calibrator = calibrator
        self.transport = transport
        self.dedup = Deduplicator()
        self.pqueue = PersistentQueue(db_path)
        self.audit_log: list[dict] = []   # journal d'audit (chaque message traversant l'ONA)

    # --- Sens montant : LoRa -> traitement -> file -> MQTT -----------------

    def on_lora_frame_received(self, frame: bytes) -> None:
        """Point d'entrée appelé par le driver radio (callback IRQ / thread lecture LoRa)."""
        msg = verify_and_parse_frame(frame)
        if msg is None:
            self._audit("REJECTED", reason="CRC/HMAC invalide")
            return

        if self.dedup.is_duplicate(msg):
            self._audit("DUPLICATE", beacon_id=msg.beacon_id)
            return

        gps = self.calibrator.local_to_gps(
            __import__("numpy").array([msg.local_x, msg.local_y, msg.local_z]),
            dist_since_last_anchor_m=msg.dist_since_anchor_m,
        )

        uplink_payload = {
            "beacon_id": msg.beacon_id,
            "kind": msg.kind,
            "gps": gps,
            "severity": msg.severity,
            "confidence": msg.confidence,
            "next": {"id": msg.next_id},
            "ttl_s": msg.ttl_s,
            "received_at": time.time(),
        }

        self.pqueue.push(uplink_payload)
        self._audit("QUEUED", beacon_id=msg.beacon_id, gps=gps)

    def flush_queue(self) -> None:
        """À appeler périodiquement : tente d'envoyer tout ce qui est en attente."""
        link = choose_link(self.transport)
        if link == LinkType.NONE:
            self._audit("NO_LINK", info="LTE et satellite indisponibles, message conservé en file")
            return

        for row_id, payload in self.pqueue.pending():
            ok = self.transport.send_mqtt_tls(MQTT_TOPIC_UP, payload, via=link)
            if ok:
                self.pqueue.mark_sent(row_id)
                self._audit("SENT", via=link.value, beacon_id=payload.get("beacon_id"))

    # --- Sens descendant : mission signée -> LoRa vers Executor ------------

    def forward_mission_to_executor(self, mission_signed: dict, lora_send_fn) -> None:
        """
        mission_signed : {"mission_id":..., "graph":[...], "signature": "..."}
        lora_send_fn   : fonction fournie par le driver radio pour émettre en LoRa.
        """
        frame = json.dumps(mission_signed).encode()
        lora_send_fn(frame)
        self._audit("MISSION_FORWARDED", mission_id=mission_signed.get("mission_id"))

    # --- Journal d'audit -----------------------------------------------------

    def _audit(self, event: str, **kwargs) -> None:
        entry = {"ts": time.time(), "event": event, **kwargs}
        self.audit_log.append(entry)
        print(f"[AUDIT] {entry}")


# ---------------------------------------------------------------------------
# Exemple d'utilisation (simulation)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import numpy as np
    from ekf import AnchorPoint

    anchors = [
        AnchorPoint(np.array([0.0, 0.0, 0.0]), lat=34.4310, lon=8.7840, alt=310.0),
        AnchorPoint(np.array([5.0, 0.0, 0.0]), lat=34.4310, lon=8.78406, alt=310.1),
        AnchorPoint(np.array([0.0, 5.0, 0.0]), lat=34.43105, lon=8.7840, alt=310.0),
        AnchorPoint(np.array([5.0, 5.0, 0.2]), lat=34.43105, lon=8.78406, alt=310.2),
    ]
    calib = LocalToGpsCalibrator(anchors)
    transport = SimulatedTransport(lte_up=True, sat_up=True)
    ona = ONAGateway(calib, transport)

    # Simuler la réception d'une balise "VICTIM" déposée par le Writer
    msg = BeaconMessage(
        beacon_id=7, kind="VICTIM",
        local_x=12.3, local_y=7.8, local_z=0.0,
        severity=15, confidence=0.92,
        next_id=6, dist_since_anchor_m=42.0,
        ttl_s=7200, timestamp=time.time(),
    )
    frame = build_wire_frame(msg)
    print(f"Trame radio construite : {len(frame)} octets "
          f"({'OK' if len(frame) <= 41 else 'DEPASSEMENT'} <= 41 octets)")

    ona.on_lora_frame_received(frame)
    ona.on_lora_frame_received(frame)  # doit être détecté comme doublon
    ona.flush_queue()