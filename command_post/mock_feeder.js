// mock_feeder.js - fakes a live mission for the dashboard (no robot, no radio).
// Posts exactly what python/beaconnet/gateway_bridge.py posts from a real LoRa
// gateway: the contract fields + the full LMB2 record (v: 2).
// Zero dependencies (Node's http module) - works on any Node version.
//
//   node mock_feeder.js                         (posts to http://localhost:3000)
//   TARGET=http://192.168.1.20:3000 node mock_feeder.js
//   SPEED=30 node mock_feeder.js                (mission time runs 30x; default 20)
//   ONA=0 node mock_feeder.js                   (one gateway, no Outside Network Area)
//
// Storyline (mission time): the Writer lays a chain of beacons through a
// winding corridor, one every ~45 s: EXIT, trail, GAS (fades in minutes),
// VICTIM, RADIATION, OBSTRUCTION (valid for days), COLLAPSE RISK ... Each
// record points to the previous beacon (the way out). Mid-mission beacon #6
// is crushed: its neighbours report it LOST? while its record stays on the
// map (replicated). The victim record is updated (severity 15 -> 40). The
// gas event ages out and its beacon becomes a stale route marker. A spoofed
// frame is rejected by gw1. gw2 drops out ~20 s every minute. The Executor
// walks the chain.
// ONA (on by default): three gateways around the building vote 2-of-3 on
// every record; the Writer and the Executor are positioned by the gateways'
// distance spheres; gw3 starts lying at 18 min and gets ignored; a forged
// victim heard by one gateway stays "unconfirmed"; the Executor's SLAM slips
// at 20 min (drift alert, back at 24); LTE fails at 30 min (satellite) and
// comes back at 36. Dispatch a mission: the ONA briefs and the Executor acks.
// The real thing: python -m ona + python -m ona.zonesim (see ../ona/README.md).
const http = require('http');
const { URL } = require('url');

const TARGET = process.env.TARGET || 'http://localhost:3000';
const ONA = process.env.ONA !== '0';
const SPEED = Number(process.env.SPEED) || 20;
const ANCHOR = { lat: 36.8065, lon: 10.1815 };
const M_LAT = 111320;
const M_LON = 111320 * Math.cos(ANCHOR.lat * Math.PI / 180);

const KIND = {                                     // half-life / TTL defaults (s), as beacon_proto.c
  WAYPOINT: [43200, 86400], VICTIM: [3600, 7200], GAS: [600, 3600], RADIATION: [21600, 43200],
  THERMAL: [1200, 3600], OBSTRUCTION: [172800, 604800], STRUCTURAL: [86400, 259200], EXIT: [172800, 604800],
};
const PLAN = ['EXIT', 'WAYPOINT', 'WAYPOINT', 'GAS', 'WAYPOINT', 'WAYPOINT', 'VICTIM', 'WAYPOINT',
  'THERMAL', 'RADIATION', 'WAYPOINT', 'OBSTRUCTION', 'WAYPOINT', 'STRUCTURAL', 'WAYPOINT', 'WAYPOINT'];
const SEV = { VICTIM: 15, GAS: 60, RADIATION: 70, THERMAL: 55, OBSTRUCTION: 40, STRUCTURAL: 65 };
const SEGMENTS = [[4, 0], [4, 90], [3, 0], [3, -90], [99, 0]];   // corridor: count, heading (deg from east)

function post(path, body) {
  return new Promise((resolve) => {
    const u = new URL(path, TARGET);
    const data = JSON.stringify(body);
    const req = http.request({
      hostname: u.hostname, port: u.port || 80, path: u.pathname, method: 'POST',
      headers: { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) },
    }, (res) => { res.resume(); res.on('end', resolve); });
    req.on('error', (e) => { console.error(`Failed to POST ${path}: ${e.message}`); resolve(); });
    req.write(data); req.end();
  });
}

const t0 = Date.now();
const missionS = () => (Date.now() - t0) / 1000 * SPEED;      // mission clock, seconds
const unix0 = Math.floor(Date.now() / 1000);
const beacons = [];      // { id, kind, x, y, rec }
let pos = { x: 2, y: 0 }, seg = 0, inSeg = 0, rejects = 0;

function latlon(x, y) { return { lat: +(ANCHOR.lat + y / M_LAT).toFixed(7), lon: +(ANCHOR.lon + x / M_LON).toFixed(7) }; }

function effConf(b) {
  const age = missionS() - b.bornS;
  if (age > b.ttl) return 0;
  return b.conf * Math.pow(2, -age / b.hl);
}

function payload(b, ev) {
  const p = payloadV8(b, ev);
  if (!ONA) return p;
  const gws = onaVoters(b);
  return { ...p, gateway_id: gws.join('+'), ona: { state: 'confirmed', votes: gws.length, of: 3, quorum: 2, gateways: gws } };
}
function payloadV8(b, ev) {
  const { lat, lon } = latlon(b.x, b.y);
  const eff = ev === 'expired' ? 0 : effConf(b);
  const hops = Math.max(0, Math.floor((b.id - 1) / 2));
  return {
    beacon_id: b.id, event_type: b.kind.toLowerCase(), value: b.sev, lat, lon,
    timestamp: unix0 + Math.round(b.bornS), ttl: Math.max(0, Math.min(20, Math.round(20 * eff))),
    gateway_id: 'gw1', rssi: hops === 0 ? -70 - b.id * 4 : null,
    v: 2, ev, kind: b.kind, seq: b.seq, severity: b.sev, confidence: b.conf, eff_conf: +eff.toFixed(3),
    err_m: b.err, gps_src: 'slam', next: b.next, ttl_s: b.ttl, half_life_s: b.hl,
    age_s: Math.round(missionS() - b.bornS), hops, relay: hops ? b.id - 1 : b.id, snr: hops ? null : 9.5,
    silent: !!b.silent, stale: !!b.stale, retracted: false, time_scale: SPEED,
  };
}

function dropNext() {
  const i = beacons.length;
  if (i >= PLAN.length) return false;
  if (i > 0) {
    if (inSeg >= SEGMENTS[seg][0]) { seg += 1; inSeg = 0; }
    const step = 8.5 + (Math.random() - 0.5) * 2;
    const h = SEGMENTS[seg][1] * Math.PI / 180;
    pos = { x: pos.x + step * Math.cos(h), y: pos.y + step * Math.sin(h) };
  }
  inSeg += 1;
  const kind = PLAN[i];
  const prev = beacons[i - 1];
  const b = {
    id: i + 1, kind, x: pos.x, y: pos.y, seq: 1, sev: SEV[kind] || 0,
    conf: kind === 'WAYPOINT' || kind === 'EXIT' ? 1 : +(0.8 + Math.random() * 0.2).toFixed(2),
    hl: KIND[kind][0], ttl: KIND[kind][1], bornS: missionS(),
    err: +(0.3 + 0.03 * (i * 8.5)).toFixed(1), next: null,
  };
  if (prev) {
    const dx = prev.x - b.x, dy = prev.y - b.y;
    b.next = { id: prev.id, dist_m: +Math.hypot(dx, dy).toFixed(1),
      bearing_deg: +(((90 - Math.atan2(dy, dx) * 180 / Math.PI) + 360) % 360).toFixed(1) };
  }
  beacons.push(b);
  post('/api/beacon', payload(b, 'new'));
  console.log(`[mock] t=${Math.round(missionS())}s drop #${b.id} ${kind}`);
  return true;
}

// ---- timeline (mission seconds) ------------------------------------------------
const events = [
  { at: 12 * 60, run: () => { const v = beacons.find((b) => b.kind === 'VICTIM'); if (v) { v.seq = 2; v.sev = 40; v.bornS = missionS(); post('/api/beacon', payload(v, 'update')); console.log('[mock] victim updated'); } } },
  { at: 15 * 60, run: () => { const b = beacons[5]; if (b) { b.dead = true; console.log('[mock] beacon #6 crushed'); } } },
  { at: 23 * 60, run: () => { const b = beacons[5]; if (b) { b.silent = true; post('/api/beacon', payload(b, 'silent')); console.log('[mock] #6 reported silent'); } } },
  { at: 9 * 60, run: () => { rejects += 1; console.log('[mock] spoofed frame rejected'); } },
  { at: 27 * 60, run: () => { rejects += 1; } },
];

let nextDropS = 0;
setInterval(() => {
  const now = missionS();
  if (now >= nextDropS && beacons.length < PLAN.length) { dropNext(); nextDropS = now + 45; }
  for (const e of events) if (!e.done && now >= e.at) { e.done = true; e.run(); }
  // adaptive aging: an event whose confidence fell under 5 % is re-issued by its
  // beacon as a STALE route marker (new seq, waypoint lifetime)
  for (const b of beacons) {
    if (!b.stale && !b.dead && b.kind !== 'WAYPOINT' && effConf(b) < 0.05) {
      b.stale = true; b.seq += 1; b.conf = 1; b.bornS = now;
      b.hl = KIND.WAYPOINT[0]; b.ttl = KIND.WAYPOINT[1];
      post('/api/beacon', payload(b, 'update'));
      console.log(`[mock] #${b.id} ${b.kind} aged out -> route marker`);
    }
  }
}, 250);

// anti-entropy style refresh: a random known record re-reported now and then
setInterval(() => {
  if (!beacons.length) return;
  const b = beacons[Math.floor(Math.random() * beacons.length)];
  post('/api/beacon', payload(b, 'dump'));
}, 5000);

// Executor walks the chain inwards, then back out
let execIdx = 0, execDir = 1, execT = 0;
setInterval(() => {
  if (beacons.length < 2) return;
  const a = beacons[Math.min(execIdx, beacons.length - 1)];
  const b = beacons[Math.min(execIdx + execDir, beacons.length - 1)] || a;
  execT += 0.12;
  if (execT >= 1) {
    execT = 0; execIdx += execDir;
    if (execIdx >= beacons.length - 1) execDir = -1;
    if (execIdx <= 0) execDir = 1;
  }
  const x = a.x + (b.x - a.x) * execT, y = a.y + (b.y - a.y) * execT;
  const heading = Math.atan2(b.y - a.y, b.x - a.x);
  post('/api/executor-status', { ...latlon(x - 0.6, y - 0.6), heading: +heading.toFixed(2), status: 'navigating', timestamp: Date.now() / 1000 });
}, 1000);

// Gateway heartbeats every 4 s. gw2 drops out for ~20 s every minute.
setInterval(() => {
  if (ONA) return;                     // the ONA reports the three gateways (below)
  post('/api/network-health', { gateway_id: 'gw1', status: 'online', timestamp: Date.now() / 1000,
    known: beacons.length, rx_ok: beacons.length * 7, rejected: rejects, reject_reasons: rejects ? { bad_mac: rejects } : {}, duty: 0.004 });
  const sec = new Date().getSeconds();
  if (sec < 40) post('/api/network-health', { gateway_id: 'gw2', status: 'online', timestamp: Date.now() / 1000, known: Math.max(0, beacons.length - 2) });
}, 4000);

// =====================================================================================
// Outside Network Area mock (the real one: ../ona, python -m ona)
// =====================================================================================
const GW = { gw1: { x: -5, y: -6, z: 2.0 }, gw2: { x: 75, y: -6, z: 2.2 }, gw3: { x: 36, y: 44, z: 2.0 } };
const ona = {
  liar: false, quarantined: false, forged: false, drift: 0, driftOn: false, link: 'LTE',
  mission: null, lastDispatch: 0, events: [], seq: 0, ack: false, score: { gw1: [0, 0], gw2: [0, 0], gw3: [0, 0] },
};
const gw2Up = () => new Date().getSeconds() < 40;
function onaVoters() {
  const v = ['gw1'];
  if (gw2Up()) v.push('gw2');
  if (!ona.quarantined) v.push('gw3');
  for (const g of v) ona.score[g][0] += 1;
  return v;
}
function gauss() { return Math.sqrt(-2 * Math.log(Math.random() + 1e-12)) * Math.cos(2 * Math.PI * Math.random()); }
function onaEvent(level, event, text, extra = {}) {
  const e = { t: Date.now() / 1000, level, event, text, ...extra };
  ona.events.push(e); if (ona.events.length > 12) ona.events.shift();
  if (ona.link === 'NONE') return;
  post('/api/ona-event', e);
  console.log(`[mock-ona] ${level.toUpperCase()} ${event}: ${text}`);
}
// what the three gateways measure to a robot standing at (x, y)
function sphereReport(id, role, x, y, heading, extra) {
  const ranges = [];
  for (const [g, p] of Object.entries(GW)) {
    if (g === 'gw2' && !gw2Up()) continue;
    if (g === 'gw3' && ona.quarantined) continue;
    const dz = 0.3 - p.z;
    const truth = Math.hypot(x - p.x, y - p.y, dz);
    const walls = Math.min(4, Math.floor(truth / 18));
    const raw = truth + walls * 0.6 + gauss() * 0.25;
    const r = raw - walls * 0.6;
    ranges.push({ gw: g, ...latlon(p.x, p.y), range_m: +r.toFixed(2), raw_m: +raw.toFixed(2), walls,
      horiz_m: +Math.sqrt(Math.max(0, r * r - dz * dz)).toFixed(2), sigma_m: +(0.3 + walls * 0.25).toFixed(2),
      src: 'tof', residual_m: +(gauss() * 0.15).toFixed(2), accepted: true });
  }
  const sig = ranges.length >= 3 ? 0.35 : 0.6;
  const mx = x + gauss() * sig * 0.5, my = y + gauss() * sig * 0.5;
  const rep = role === 'EXECUTOR' ? { x: x + ona.drift, y: y - ona.drift * 0.6 } : { x: x + gauss() * 0.1, y: y + gauss() * 0.1 };
  const dist = Math.hypot(rep.x - mx, rep.y - my);
  const ang = Math.atan2(my - GW.gw3.y, mx - GW.gw3.x);
  return {
    robot_id: id, role, timestamp: Date.now() / 1000, votes: ranges.length, of: 3, state: 'confirmed', ranges,
    consistency: { state: ona.driftOn && role === 'EXECUTOR' ? 'drift' : 'agree', dist_m: +dist.toFixed(2), sigmas: +(dist / 0.5).toFixed(2) },
    reported: { ...latlon(rep.x, rep.y), x: +rep.x.toFixed(3), y: +rep.y.toFixed(3), pose_sd_m: 0.2 },
    heading: +heading.toFixed(3), seq: ++ona.seq, battery_pct: Math.max(20, 100 - Math.round(missionS() / 60)),
    odo_m: Math.round(missionS() * 0.25), stuck: false, emergency: false,
    measured: { ...latlon(mx, my), sigma_m: sig, ellipse: { a_m: sig, b_m: +(sig * 0.7).toFixed(2), angle_rad: +ang.toFixed(3) }, ranges_used: ranges.length },
    fix: { ...latlon(mx, my), sigma_m: sig, hdop: ranges.length >= 3 ? 1.2 : 2.4, raim: ranges.length >= 3 ? 'pass' : 'n/a', chi2: 0.8 },
    ...latlon(mx, my), sigma_m: sig, source: 'gateways', ...extra,
  };
}
if (ONA) {
  const onaT = [
    { at: 10 * 60, run: () => {   // a forged victim, heard by gw2 only: never confirmed
      const fx = 30, fy = 20;
      post('/api/beacon', { beacon_id: 77, event_type: 'victim', value: 90, ...latlon(fx, fy), timestamp: Math.floor(Date.now() / 1000), ttl: 20,
        gateway_id: 'gw2', v: 2, ev: 'new', kind: 'VICTIM', seq: 1, severity: 90, confidence: 1, eff_conf: 1, err_m: 0.5,
        gps_src: 'slam', next: null, ttl_s: 7200, half_life_s: 3600, age_s: 0, hops: 0, relay: 77, silent: false, stale: false,
        retracted: false, time_scale: SPEED, ona: { state: 'unconfirmed', votes: 1, of: 3, quorum: 2, gateways: ['gw2'] } });
      console.log('[mock-ona] forged victim #77 from gw2 only -> unconfirmed');
    } },
    { at: 18 * 60, run: () => {
      ona.liar = true;
      for (const id of [4, 7, 10]) {
        onaEvent('alert', 'GATEWAY_DISAGREES', `gw3 reported a different signed version of beacon #${id} seq 1 than the other gateways`, { gw: 'gw3', beacon_id: id, seq: 1 });
        ona.score.gw3[1] += 1;
      }
      ona.quarantined = true;
      onaEvent('alert', 'GATEWAY_QUARANTINED', 'gw3 is ignored from now on: 3 versions the majority did not hear', { gw: 'gw3' });
    } },
    { at: 20 * 60, run: () => { ona.drift = 2.6; ona.driftOn = true;
      onaEvent('warn', 'SLAM_DRIFT', "robot #1: its own position and the gateways' measurement differ by 3.1 m. Trust the gateways.", { robot_id: 1 }); } },
    { at: 24 * 60, run: () => { ona.drift = 0.2; ona.driftOn = false;
      onaEvent('info', 'SLAM_OK', 'robot #1: own position agrees with the gateways again', { robot_id: 1 }); } },
    { at: 30 * 60, run: () => { onaEvent('warn', 'LINK_CHANGE', 'LTE lost: only hazards, alerts and the Executor position go out, over the satellite (Iridium SBD, 340 bytes a message). The rest waits in the queue.', { link: 'SATELLITE', prev: 'LTE' }); ona.link = 'SATELLITE'; } },
    { at: 36 * 60, run: () => { ona.link = 'LTE'; onaEvent('info', 'LINK_CHANGE', 'LTE back: flushing 41 queued message(s), most important first', { link: 'LTE', prev: 'SATELLITE' }); } },
  ];
  setInterval(() => {
    const now = missionS();
    for (const e of onaT) if (!e.done && now >= e.at) { e.done = true; e.run(); }
    if (ona.drift > 0.2 && !ona.driftOn) ona.drift *= 0.9;
  }, 250);

  // robots: the Writer at the head of the chain, the Executor on its walk (positions from the spheres)
  let lastExec = null;
  setInterval(() => {
    if (ona.link !== 'LTE') return;          // on satellite the ONA only sends the essentials
    if (beacons.length) {
      const last = beacons[beacons.length - 1];
      const h = SEGMENTS[Math.min(seg, SEGMENTS.length - 1)][1] * Math.PI / 180;
      const k = Math.min(1, ((missionS() - last.bornS) / 45));
      const done = beacons.length >= PLAN.length;
      const wx = done ? beacons[0].x + 1.5 : last.x + Math.cos(h) * 8.5 * k;
      const wy = done ? beacons[0].y + 1.5 : last.y + Math.sin(h) * 8.5 * k;
      post('/api/robot-status', sphereReport(2, 'WRITER', wx, wy, h, { phase: done ? 'DONE' : 'FOLLOW', done: beacons.length, total: PLAN.length, last_beacon: last.id }));
    }
    if (lastExec) {
      const m = ona.mission;
      post('/api/robot-status', sphereReport(1, 'EXECUTOR', lastExec.x, lastExec.y, lastExec.h,
        { phase: m ? (m.done >= m.targets.length ? 'RETURN' : 'GOTO') : 'WAIT', mission_id: m ? m.id : 0, mission_ack: !!(m && ona.ack),
          done: m ? m.done : 0, total: m ? m.targets.length : 0, last_beacon: beacons[Math.min(execIdx, beacons.length - 1)]?.id || 0 }));
    }
  }, 2000);
  // keep the Executor's true position for the spheres (the loop above posts executor-status)
  setInterval(() => {
    if (beacons.length < 2) return;
    const a = beacons[Math.min(execIdx, beacons.length - 1)];
    const b = beacons[Math.min(execIdx + execDir, beacons.length - 1)] || a;
    lastExec = { x: a.x + (b.x - a.x) * execT - 0.6, y: a.y + (b.y - a.y) * execT - 0.6, h: Math.atan2(b.y - a.y, b.x - a.x) };
  }, 500);

  // briefing: a mission dispatched on the dashboard is signed, sent by the gateways and acknowledged
  setInterval(() => {
    const u = new URL('/api/mission-dispatch/latest', TARGET);
    http.get({ hostname: u.hostname, port: u.port || 80, path: u.pathname }, (res) => {
      let body = ''; res.on('data', (c) => { body += c; });
      res.on('end', () => {
        let m = null; try { m = JSON.parse(body); } catch (_) { return; }
        if (!m || !m.waypoints || !m.dispatched_at || m.dispatched_at <= ona.lastDispatch) return;
        if (!ona.lastDispatch && m.dispatched_at < unix0) { ona.lastDispatch = m.dispatched_at; return; }
        ona.lastDispatch = m.dispatched_at;
        const id = 1000 + Math.floor(Math.random() * 60000);
        if (m.abort) {
          ona.mission = ona.mission ? { ...ona.mission, id, targets: [], done: 0 } : null; ona.ack = false;
          onaEvent('warn', 'MISSION_DISPATCH', `ABORT briefing ${id}: the Executor is called back to the exit (1 signed frame)`, { mission_id: id, targets: [] });
          setTimeout(() => { ona.ack = true; onaEvent('info', 'MISSION_ACK', `robot #1 (executor) received briefing ${id}: abort`, { robot_id: 1, mission_id: id }); }, 3000);
          return;
        }
        const targets = m.waypoints.map((w) => Number(w.beacon_id)).filter((x) => x >= 0 && x <= 65535);
        ona.mission = { id, targets, done: 0 }; ona.ack = false;
        onaEvent('info', 'MISSION_DISPATCH', `briefing ${id} for the Executor: targets ${targets.map((t) => '#' + t).join(' -> ')}, then back to the exit (1 signed frame(s))`, { mission_id: id, targets });
        setTimeout(() => { ona.ack = true; onaEvent('info', 'MISSION_ACK', `robot #1 (executor) received briefing ${id}: ${targets.length} target(s)`, { robot_id: 1, mission_id: id }); }, 3000);
        const tick = setInterval(() => { if (!ona.mission || ona.mission.id !== id) return clearInterval(tick);
          ona.mission.done += 1; if (ona.mission.done >= targets.length) clearInterval(tick); }, 12000);
      });
    }).on('error', () => {});
  }, 2000);

  // ONA status (2 s) + gateway health with positions and agreement
  setInterval(() => {
    if (ona.link !== 'LTE') return;
    const now = Date.now() / 1000;
    const gws = Object.entries(GW).map(([g, p]) => {
      const [rep, dis] = ona.score[g];
      const alive = g !== 'gw2' || gw2Up();
      return { gw: g, reports: rep + dis, agreed: rep, disagreed: dis, invalid: 0, solo: 0, quarantined: g === 'gw3' && ona.quarantined,
        note: g === 'gw3' && ona.quarantined ? '3 versions the majority did not hear' : '', agreement: rep + dis ? +(rep / (rep + dis)).toFixed(3) : 1,
        ...latlon(p.x, p.y), alt: 10 + p.z, alive, last_heard_s: alive ? 0.5 : 12, lines: rep * 3, known: beacons.length, duty: 0.004, air_rejects: {} };
    });
    for (const g of gws) {
      post('/api/network-health', { gateway_id: g.gw, status: g.alive ? 'online' : 'offline', timestamp: now, known: g.known, rx_ok: g.lines,
        rejected: g.gw === 'gw1' ? rejects : 0, reject_reasons: g.gw === 'gw1' && rejects ? { bad_mac: rejects } : {}, duty: g.duty,
        lat: g.lat, lon: g.lon, agreement: g.agreement, quarantined: g.quarantined, via: 'ona' });
    }
    const m = ona.mission;
    post('/api/ona-status', {
      name: 'ONA-mock', t: now, uptime_s: Math.round(now - unix0),
      link: { state: 'LTE', pending: 0, pending_bytes: 0, sent_by_link: { LTE: ona.seq * 4 }, oldest_pending_s: 0 },
      gateways: gws, votes: { quorum: 2, gateways: 3, confirmed: beacons.length, conflicts: ona.liar ? 3 : 0 },
      calibration: { rms_m: 0.04, yaw_deg: 0, anchor: ANCHOR,
        in_mission_check: missionS() > 300 ? { pairs: Math.round(missionS() / 4), rms_m: 0.41, dyaw_deg: 0.6, max_shift_m: 0.7, mean_shift_m: 0.42, verdict: 'holds' } : null },
      robots: [{ robot_id: 1, role: 'EXECUTOR' }, { robot_id: 2, role: 'WRITER' }],
      mission: m ? { mission_id: m.id, seq: 1, targets: m.targets, tx_count: 1, acked_by: ona.ack ? [1] : [], return_to_exit: true,
        progress: { done: m.done, total: m.targets.length, phase: m.done >= m.targets.length ? 'RETURN' : 'GOTO' }, source: 'command post' } : null,
      records: { confirmed: beacons.length, unconfirmed: missionS() > 600 ? 1 : 0, conflicts: 0 },
      alerts: ona.events.filter((e) => e.level === 'alert').length, events: ona.events, quorum: 2,
    });
  }, 2000);
}

console.log(`Mock mission running at ${SPEED}x, sending LMB2 records to ${TARGET} ...`);
