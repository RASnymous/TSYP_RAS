"""
GALERIA - The Living Map
Store and forward: nothing is lost when the uplink to the Command Post is down.

Origin: ona.py (GALERIA team), "3. File persistante SQLite" and the audit log.
The queue is that code; v9 adds, without changing the idea:

* a priority per message (victims and hazards first, trail waypoints later) so
  that a narrow satellite link carries the important messages first;
* coalescing: a robot's position is only worth its latest value, so a new one
  replaces an unsent older one instead of queueing a replay;
* the endpoint each message goes to, attempts and the link that carried it;
* the audit log is written to the same database (table `audit`).

Everything is in one SQLite file (default ona_queue.sqlite3). Delete the file to
start a mission with an empty queue.
"""
from __future__ import annotations

import json
import sqlite3
import threading
import time
from typing import Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS uplink_queue (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    payload_json TEXT NOT NULL,
    created_at REAL NOT NULL,
    sent INTEGER NOT NULL DEFAULT 0
);
CREATE TABLE IF NOT EXISTS audit (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL NOT NULL,
    event TEXT NOT NULL,
    detail_json TEXT NOT NULL
);
"""
# v9 columns, added to an existing (original) queue file if needed
V9_COLUMNS = [('endpoint', "TEXT NOT NULL DEFAULT '/api/beacon'"), ('priority', 'INTEGER NOT NULL DEFAULT 1'),
              ('coalesce_key', 'TEXT'), ('attempts', 'INTEGER NOT NULL DEFAULT 0'), ('sent_at', 'REAL'),
              ('link', 'TEXT'), ('size', 'INTEGER NOT NULL DEFAULT 0')]


class PersistentQueue:
    def __init__(self, db_path: str = 'ona_queue.sqlite3'):
        self.path = db_path
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._lock = threading.Lock()
        with self._lock:
            self._conn.executescript(SCHEMA)
            have = {r[1] for r in self._conn.execute('PRAGMA table_info(uplink_queue)')}
            for name, decl in V9_COLUMNS:
                if name not in have:
                    self._conn.execute(f'ALTER TABLE uplink_queue ADD COLUMN {name} {decl}')
            self._conn.execute('CREATE INDEX IF NOT EXISTS q_pending ON uplink_queue(sent, priority, id)')
            self._conn.execute('CREATE INDEX IF NOT EXISTS q_coalesce ON uplink_queue(coalesce_key, sent)')
            self._conn.commit()

    # ---------------------------------------------------------------- queue
    def push(self, payload: dict, endpoint: str = '/api/beacon', priority: int = 1,
             coalesce_key: Optional[str] = None) -> int:
        data = json.dumps(payload, separators=(',', ':'))
        now = time.time()
        with self._lock:
            if coalesce_key is not None:
                row = self._conn.execute('SELECT id FROM uplink_queue WHERE coalesce_key=? AND sent=0 '
                                         'ORDER BY id DESC LIMIT 1', (coalesce_key,)).fetchone()
                if row:
                    self._conn.execute('UPDATE uplink_queue SET payload_json=?, created_at=?, size=?, priority=? '
                                       'WHERE id=?', (data, now, len(data), priority, row[0]))
                    self._conn.commit()
                    return row[0]
            cur = self._conn.execute('INSERT INTO uplink_queue (payload_json, created_at, sent, endpoint, priority, '
                                     'coalesce_key, size) VALUES (?, ?, 0, ?, ?, ?, ?)',
                                     (data, now, endpoint, priority, coalesce_key, len(data)))
            self._conn.commit()
            return cur.lastrowid

    def pending(self, limit: int = 50, min_priority: int = 0) -> list:
        """[(id, endpoint, payload, priority)], most important first, oldest first within a priority."""
        with self._lock:
            rows = self._conn.execute('SELECT id, endpoint, payload_json, priority FROM uplink_queue '
                                      'WHERE sent=0 AND priority>=? ORDER BY priority DESC, id ASC LIMIT ?',
                                      (min_priority, limit)).fetchall()
        return [(r[0], r[1], json.loads(r[2]), r[3]) for r in rows]

    def mark_sent(self, row_id: int, link: str = '') -> None:
        with self._lock:
            self._conn.execute('UPDATE uplink_queue SET sent=1, sent_at=?, link=? WHERE id=?',
                               (time.time(), link, row_id))
            self._conn.commit()

    def mark_failed(self, row_id: int) -> None:
        with self._lock:
            self._conn.execute('UPDATE uplink_queue SET attempts=attempts+1 WHERE id=?', (row_id,))
            self._conn.commit()

    def counts(self) -> dict:
        with self._lock:
            pend = self._conn.execute('SELECT COUNT(*), COALESCE(SUM(size),0) FROM uplink_queue WHERE sent=0'
                                      ).fetchone()
            sent = self._conn.execute('SELECT link, COUNT(*) FROM uplink_queue WHERE sent=1 GROUP BY link').fetchall()
            oldest = self._conn.execute('SELECT MIN(created_at) FROM uplink_queue WHERE sent=0').fetchone()[0]
        return {'pending': pend[0], 'pending_bytes': pend[1], 'sent_by_link': {k or '?': n for k, n in sent},
                'oldest_pending_s': round(time.time() - oldest, 1) if oldest else 0.0}

    # ---------------------------------------------------------------- audit log
    def audit(self, event: str, **detail) -> dict:
        entry = {'ts': time.time(), 'event': event, **detail}
        with self._lock:
            self._conn.execute('INSERT INTO audit (ts, event, detail_json) VALUES (?, ?, ?)',
                               (entry['ts'], event, json.dumps(detail, default=str, separators=(',', ':'))))
            self._conn.commit()
        return entry

    def audit_tail(self, n: int = 30) -> list:
        with self._lock:
            rows = self._conn.execute('SELECT ts, event, detail_json FROM audit ORDER BY id DESC LIMIT ?',
                                      (n,)).fetchall()
        return [{'ts': r[0], 'event': r[1], **json.loads(r[2])} for r in rows]

    def close(self) -> None:
        with self._lock:
            self._conn.close()
