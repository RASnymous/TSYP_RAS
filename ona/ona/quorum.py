"""
2-of-3: a message counts once two gateways heard exactly the same thing.

Every LMB2 frame is already signed (HMAC), and the ONA re-checks that
signature itself. The vote adds what a signature cannot:

* A gateway is a computer with the mission key in it. If one is faulty,
  hacked, or fed by an attacker on its USB cable, it can make up a correctly
  signed record. With the vote, no single gateway can put anything on the
  Command Post's map.
* An attacker with a stolen key who transmits close to one gateway is heard
  by that gateway only: the record never reaches a quorum.
* Two different signed versions of the same record (same beacon, same
  sequence number) should never exist. If they do, the vote shows which one
  the majority heard and names the gateway that disagrees.

What is compared: the "sealed part" of the frame (type, network, the 30 signed
bytes and the MAC). Relay id and hop count legitimately differ between copies
and are left out. Gateways that only send decoded JSON are compared on the
canonical signed fields instead.

Vote states for one (type, origin, seq):
    PENDING      heard by fewer than `quorum` gateways (shown as "unconfirmed n/3")
    CONFIRMED    `quorum` gateways reported identical bytes
    CONFLICT     two different versions were reported and none has a majority
"""
from __future__ import annotations

import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Optional

PENDING, CONFIRMED, CONFLICT = 'PENDING', 'CONFIRMED', 'CONFLICT'


@dataclass
class GatewayScore:
    gw: str
    reports: int = 0          # votes cast
    agreed: int = 0           # votes that ended on the confirmed version
    disagreed: int = 0        # votes for a version the majority did not hear
    invalid: int = 0          # frames that failed the ONA's own signature check
    solo: int = 0             # versions nobody else ever reported (coverage or invention)
    last_heard: float = 0.0
    quarantined: bool = False
    note: str = ''

    def as_dict(self) -> dict:
        return {'gw': self.gw, 'reports': self.reports, 'agreed': self.agreed, 'disagreed': self.disagreed,
                'invalid': self.invalid, 'solo': self.solo, 'quarantined': self.quarantined, 'note': self.note,
                'agreement': round(self.agreed / max(1, self.agreed + self.disagreed), 3)}


@dataclass
class Vote:
    key: tuple
    first_seen: float
    bodies: dict = field(default_factory=dict)      # body -> set(gw)
    meta: dict = field(default_factory=dict)        # body -> first decoded object
    state: str = PENDING
    winner: Optional[bytes] = None
    confirmed_at: Optional[float] = None
    announced: int = 0                              # votes count last reported upstream
    solo_counted: bool = False

    def count(self, body=None) -> int:
        if body is None:
            return max((len(s) for s in self.bodies.values()), default=0)
        return len(self.bodies.get(body, ()))

    def gateways(self, body=None) -> list:
        b = self.winner if body is None else body
        return sorted(self.bodies.get(b, ())) if b is not None else sorted({g for s in self.bodies.values() for g in s})


@dataclass
class VoteEvent:
    kind: str            # 'confirmed', 'more_votes', 'conflict', 'disagree', 'pending'
    vote: Vote
    body: Optional[bytes] = None
    obj: Optional[dict] = None
    gw: Optional[str] = None
    detail: str = ''


class QuorumVoter:
    def __init__(self, gateways, quorum: int = 2, solo_after_s: float = 900.0, max_keys: int = 20000,
                 quarantine_after: int = 3):
        self.gateways = list(gateways)
        self.quorum = int(quorum)
        self.solo_after_s = solo_after_s
        self.max_keys = max_keys
        self.quarantine_after = quarantine_after
        self.votes: 'OrderedDict[tuple, Vote]' = OrderedDict()
        self.scores = {g: GatewayScore(g) for g in self.gateways}
        self.stats = {'confirmed': 0, 'conflicts': 0, 'disagreements': 0, 'invalid': 0, 'reports': 0}

    def score(self, gw: str) -> GatewayScore:
        if gw not in self.scores:
            self.scores[gw] = GatewayScore(gw)
            self.gateways.append(gw)
        return self.scores[gw]

    def invalid(self, gw: str, reason: str, now: Optional[float] = None) -> None:
        s = self.score(gw)
        s.invalid += 1
        s.last_heard = now or time.time()
        self.stats['invalid'] += 1
        self._maybe_quarantine(s, f'{s.invalid} frames with a bad signature ({reason})')

    def _maybe_quarantine(self, s: GatewayScore, why: str) -> None:
        if not s.quarantined and (s.disagreed + s.invalid) >= self.quarantine_after:
            s.quarantined = True
            s.note = why

    def release(self, gw: str) -> None:
        s = self.score(gw)
        s.quarantined = False
        s.note = 'released by the operator'
        s.disagreed = s.invalid = 0

    def offer(self, gw: str, key: tuple, body, obj: Optional[dict] = None, now: Optional[float] = None) -> list:
        """One gateway reports one version. Returns a list of VoteEvent."""
        now = time.time() if now is None else now
        s = self.score(gw)
        s.last_heard = now
        events = []
        if s.quarantined:
            return [VoteEvent('ignored', self.votes.get(key) or Vote(key, now), body, obj, gw, 'gateway quarantined')]
        v = self.votes.get(key)
        if v is None:
            v = Vote(key, now)
            self.votes[key] = v
            if len(self.votes) > self.max_keys:
                self.votes.popitem(last=False)
        else:
            self.votes.move_to_end(key)
        holders = v.bodies.setdefault(body, set())
        if gw in holders:
            return []                      # the same gateway repeating itself does not count twice
        if any(gw in hs for b, hs in v.bodies.items() if b != body):
            # this gateway already voted for another version of the same key
            events.append(VoteEvent('disagree', v, body, obj, gw, 'gateway changed its version'))
        holders.add(gw)
        if obj is not None and body not in v.meta:
            v.meta[body] = obj
        s.reports += 1
        self.stats['reports'] += 1

        if v.state == CONFIRMED:
            if body == v.winner:
                s.agreed += 1
                events.append(VoteEvent('more_votes', v, body, v.meta.get(body), gw))
            else:
                s.disagreed += 1
                self.stats['disagreements'] += 1
                self._maybe_quarantine(s, f'{s.disagreed} versions the majority did not hear')
                events.append(VoteEvent('disagree', v, body, obj, gw,
                                        f'{gw} reported a version of {key} that {len(v.bodies[v.winner])} '
                                        f'other gateway(s) did not hear'))
            return events

        n = len(holders)
        if n >= self.quorum:
            v.state = CONFIRMED
            v.winner = body
            v.confirmed_at = now
            self.stats['confirmed'] += 1
            for g in holders:
                self.score(g).agreed += 1
            for b, hs in v.bodies.items():
                if b != body:
                    for g in hs:
                        sc = self.score(g)
                        sc.disagreed += 1
                        self.stats['disagreements'] += 1
                        self._maybe_quarantine(sc, f'{sc.disagreed} versions the majority did not hear')
                        events.append(VoteEvent('disagree', v, b, v.meta.get(b), g,
                                                f'{g} reported a version of {key} the majority did not hear'))
            events.append(VoteEvent('confirmed', v, body, v.meta.get(body), gw))
            return events
        if len(v.bodies) > 1:
            if v.state != CONFLICT:
                self.stats['conflicts'] += 1
            v.state = CONFLICT
            events.append(VoteEvent('conflict', v, body, obj, gw,
                                    f'{len(v.bodies)} different signed versions of {key}, none with a majority'))
            return events
        events.append(VoteEvent('pending', v, body, obj, gw))
        return events

    def sweep(self, now: Optional[float] = None) -> list:
        """Count 'solo' reports: versions only one gateway ever heard, after solo_after_s."""
        now = time.time() if now is None else now
        out = []
        for v in self.votes.values():
            if v.state == PENDING and not v.solo_counted and now - v.first_seen > self.solo_after_s:
                v.solo_counted = True
                for b, hs in v.bodies.items():
                    for g in hs:
                        self.score(g).solo += 1
                out.append(v)
        return out

    def pending(self, older_than_s: float, now: Optional[float] = None) -> list:
        now = time.time() if now is None else now
        return [v for v in self.votes.values() if v.state == PENDING and now - v.first_seen >= older_than_s]

    def summary(self) -> dict:
        states = {PENDING: 0, CONFIRMED: 0, CONFLICT: 0}
        for v in self.votes.values():
            states[v.state] += 1
        return {'quorum': self.quorum, 'gateways': len(self.gateways), **states, **self.stats,
                'scores': [self.scores[g].as_dict() for g in self.gateways]}
