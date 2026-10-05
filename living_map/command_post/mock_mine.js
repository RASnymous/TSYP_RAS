// mock_mine.js - the GAFSA MINE scenario on the dashboard, without ROS, without the ONA.
// It replays the ready-made mine mission (../writer_robot/missions/demo_mine.json: what
// the Writer recorded in the simulated room-and-pillar panel) the way the Outside Network
// Area would post it: every record confirmed by the gateways, the robots' positions,
// the victim search area by area, resources, then the Executor briefed from the dashboard.
// Zero dependencies (Node's http module).
//
//   node server.js --demo-mine                 (= npm run demo:mine: server + this feeder)
//   node mock_mine.js                          (against a dashboard already running)
//   SPEED=8 node mock_mine.js                  (mission time runs 8x; default 4)
//   TARGET=http://192.168.1.20:3000 node mock_mine.js
//
// Storyline: the Writer enters at the portal and explores the panel (about 7 min of mission
// time): pins appear as it drops beacons - radon in the dead end, the rich phosphate layer,
// the trapped miner, the fire, the cracked roof, bad air - and each area turns green when the
// thermal camera has searched it. Then the Executor waits at the portal for a briefing: select
// targets on the map and press "Dispatch mission" (a minute of mission time), else it goes for
// the trapped miner first and then every danger. "Call the Executor back" sends it home.
const http = require('http');
const fs = require('fs');
const path = require('path');
const { URL } = require('url');

const TARGET = process.env.TARGET || 'http://localhost:3000';
const SPEED = Number(process.env.SPEED) || 4;
const MISSION = process.env.MISSION || path.join(__dirname, '..', 'writer_robot', 'missions', 'demo_mine.json');
const ONA_CFG = path.join(__dirname, '..', 'ona', 'ona_config_gazebo_mine.json');
const WAIT_S = 60;                       // mission seconds the Executor waits for a briefing
const SITE = { id: 'gafsa_mine', name: 'Gafsa phosphate basin: simulated room-and-pillar panel (Metlaoui area)',
  plan: 'sites/gafsa_mine.geojson', underground: true };

let mission;
try { mission = JSON.parse(fs.readFileSync(MISSION, 'utf8')); } catch (e) {
  console.error(`[mock-mine] cannot read the mine mission ${MISSION}: ${e.message}`); process.exit(1);
}
let gwLocal = { gw1: [-1.0, 0.0, 1.8], gw2: [9.4, 6.8, 1.8], gw3: [23.0, -6.8, 1.8] };
try {
  const c = JSON.parse(fs.readFileSync(ONA_CFG, 'utf8'));
  gwLocal = Object.fromEntries(Object.entries(c.gateways).map(([k, v]) => [k, v.local]));
  if (c.site) Object.assign(SITE, c.site);
} catch (_) { /* defaults */ }

const A = mission.anchor;
const M_LAT = 111320; const M_LON = 111320 * Math.cos(A.lat * Math.PI / 180);
const ll = (x, y) => ({ lat: +(A.lat + y / M_LAT).toFixed(7), lon: +(A.lon + x / M_LON).toFixed(7) });
const HALF = { WAYPOINT: [43200, 86400], VICTIM: [3600, 7200], GAS: [600, 3600], RADIATION: [21600, 43200],
  THERMAL: [1200, 3600], OBSTRUCTION: [172800, 604800], STRUCTURAL: [86400, 259200], EXIT: [172800, 604800],
  PHOSPHATE: [259200, 604800], GOLD: [259200, 604800], GEMSTONE: [259200, 604800], SEARCHED: [7200, 43200] };

function post(p, body) {
  return new Promise((resolve) => {
    const u = new URL(p, TARGET); const data = JSON.stringify(body);
    const req = http.request({ hostname: u.hostname, port: u.port || 80, path: u.pathname, method: 'POST',
      headers: { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) } },
    (res) => { res.resume(); res.on('end', resolve); });
    req.on('error', (e) => { console.error(`[mock-mine] POST ${p}: ${e.message}`); resolve(); });
    req.write(data); req.end();
  });
}
function get(p) {
  return new Promise((resolve) => {
    const u = new URL(p, TARGET);
    http.get({ hostname: u.hostname, port: u.port || 80, path: u.pathname }, (res) => {
      let s = ''; res.on('data', (c) => { s += c; }); res.on('end', () => { try { resolve(JSON.parse(s)); } catch (_) { resolve(null); } });
    }).on('error', () => resolve(null));
  });
}

// ------------------------------------------------------------ the recorded mission
const t0rec = mission.started;
const B = mission.beacons.map((e) => ({
  id: e.id, kind: e.record.kind, rec: e.record, drop: e.drop, ev: e.event, detail: e.detail,
  t: Math.max(0, e.record.ts - t0rec), next: e.record.next ? e.record.next.id : null,
})).sort((a, b) => a.t - b.t || a.id - b.id);
const byId = new Map(B.map((b) => [b.id, b]));
const writerEnd = B[B.length - 1].t + 20;

const t0 = Date.now();
const unix0 = Math.floor(Date.now() / 1000);
const missionS = () => (Date.now() - t0) / 1000 * SPEED;
const posted = new Set();
const gws = Object.keys(gwLocal);

function payload(b) {
  const r = b.rec; const [hl, ttl] = HALF[r.kind] || [3600, 7200];
  const age = Math.max(0, missionS() - b.t);
  const eff = (r.confidence ?? 1) * Math.pow(2, -age / hl);
  const votes = b.id % 7 === 3 ? 2 : 3;                    // now and then one gateway is out of range
  const g = gws.slice(0, votes);
  return {
    beacon_id: b.id, event_type: r.kind.toLowerCase(), value: r.severity || 0, lat: r.gps.lat, lon: r.gps.lon,
    timestamp: unix0 + Math.round(b.t / SPEED), ttl: Math.max(0, Math.min(20, Math.round(20 * eff))),
    gateway_id: g.join('+'), rssi: -95 - (b.id % 9), v: 2, ev: 'new', kind: r.kind, seq: r.seq || 1,
    severity: r.severity || 0, confidence: r.confidence ?? 1, eff_conf: +eff.toFixed(3), err_m: r.gps.err_m,
    gps_src: 'slam', next: r.next || null, ttl_s: ttl, half_life_s: hl, age_s: Math.round(age),
    hops: Math.min(9, Math.floor(b.id / 6)), relay: b.id, snr: 4.5, silent: false, stale: false, retracted: false,
    time_scale: SPEED, ona: { state: 'confirmed', votes, of: 3, quorum: 2, gateways: g },
  };
}
const KIND_EVENT = { PHOSPHATE: 'RESOURCE_MARKED', GOLD: 'RESOURCE_MARKED', GEMSTONE: 'RESOURCE_MARKED', SEARCHED: 'AREA_SEARCHED' };
function onaEvent(level, event, text, extra = {}) {
  return post('/api/ona-event', { t: Date.now() / 1000, level, event, text, ...extra });
}

// ------------------------------------------------------------ robots
function gwRanges(x, y) {
  // underground: a gateway measures a distance only straight down a gallery (here: within 12 m)
  const out = [];
  for (const [g, p] of Object.entries(gwLocal)) {
    const d = Math.hypot(x - p[0], y - p[1]);
    if (d > 12) continue;
    const L = ll(p[0], p[1]);
    out.push({ gw: g, lat: L.lat, lon: L.lon, range_m: +Math.hypot(d, p[2] - 0.3).toFixed(2), horiz_m: +d.toFixed(2),
      sigma_m: 0.5, src: 'tof', walls: 0, raw_m: +d.toFixed(2), accepted: true });
  }
  return out;
}
let seq = 0;
function robotStatus(role, id, x, y, yaw, extra) {
  const p = ll(x, y); const ranges = gwRanges(x, y); seq += 1;
  const meas = ranges.length ? { lat: p.lat, lon: p.lon, sigma_m: ranges.length > 1 ? 0.45 : 0.8,
    ellipse: { a_m: ranges.length > 1 ? 0.45 : 0.9, b_m: 0.4, angle_rad: yaw } } : undefined;
  return post('/api/robot-status', { robot_id: id, role, seq, lat: p.lat, lon: p.lon, heading: yaw,
    sigma_m: meas ? meas.sigma_m : 1.5, source: meas ? 'gateways' : 'robot', votes: ranges.length, of: 3,
    measured: meas, ranges, reported: { lat: p.lat, lon: p.lon, x, y },
    consistency: meas ? { state: 'agree', dist_m: 0.3, sigmas: 0.6 } : {}, battery_pct: 90, ...extra });
}
function along(pts, s) {               // point at arc length s along a polyline
  for (let i = 0; i + 1 < pts.length; i++) {
    const [ax, ay] = pts[i]; const [bx, by] = pts[i + 1]; const L = Math.hypot(bx - ax, by - ay);
    if (s <= L) { const k = L ? s / L : 0; return [ax + k * (bx - ax), ay + k * (by - ay), Math.atan2(by - ay, bx - ax)]; }
    s -= L;
  }
  const e = pts[pts.length - 1]; return [e[0], e[1], 0];
}
const plen = (pts) => pts.slice(1).reduce((s, p, i) => s + Math.hypot(p[0] - pts[i][0], p[1] - pts[i][1]), 0);

// the Writer: between the drop points, in the order it dropped them
function writerAt(t) {
  let i = 0;
  while (i + 1 < B.length && B[i + 1].t <= t) i++;
  const a = B[i]; const b = B[Math.min(i + 1, B.length - 1)];
  const k = b.t > a.t ? Math.min(1, (t - a.t) / (b.t - a.t)) : 1;
  return [a.drop[0] + k * (b.drop[0] - a.drop[0]), a.drop[1] + k * (b.drop[1] - a.drop[1]),
    Math.atan2(b.drop[1] - a.drop[1], b.drop[0] - a.drop[0]) || 0];
}

// the Executor: beacon to beacon along the tree (the "next" links)
function chain(id) { const out = []; let c = id; while (c != null && byId.has(c) && out.length < 200) { out.push(c); c = byId.get(c).next; } return out; }
function treePath(a, b) {
  const ca = chain(a); const cb = chain(b); const sb = new Set(cb);
  const k = ca.findIndex((x) => sb.has(x));
  return [...ca.slice(0, k + 1), ...cb.slice(0, cb.indexOf(ca[k])).reverse()];
}
const ex = { phase: 'WAIT', at: B[0].id, targets: [], done: 0, total: 0, leg: null, s: 0, treatUntil: 0,
  mission: null, ack: false, started: null, x: 0, y: 0, yaw: 0, abort: false };
function planTargets(ids) {
  ex.targets = ids.filter((i) => byId.has(i)); ex.total = ex.targets.length; ex.done = 0; nextLeg();
}
function nextLeg() {
  const goal = ex.targets[0];
  const ids = goal != null ? treePath(ex.at, goal) : chain(ex.at);
  const pts = [[ex.x, ex.y], ...ids.map((i) => byId.get(i).drop)];
  ex.leg = { ids, pts, len: plen(pts) }; ex.s = 0;
  ex.phase = goal != null ? 'GOTO' : 'RETURN';
}
async function briefingCheck(t) {
  const m = await get('/api/mission-dispatch/latest');
  if (!m || !m.dispatched_at || (ex.mission && m.dispatched_at <= ex.mission.dispatched_at)) return;
  if (m.dispatched_at * 1000 < t0) return;          // from before this demo
  ex.mission = m; ex.mission.id = 15000 + Math.floor(Math.random() * 999); ex.ack = false;
  await onaEvent('info', 'MISSION_DISPATCH', m.abort ? `call-back #${ex.mission.id} signed, sent by the gateways`
    : `briefing #${ex.mission.id}: ${m.waypoints.map((w) => '#' + w.beacon_id).join(' -> ')} signed, sent by the gateways`);
  setTimeout(() => { ex.ack = true; onaEvent('info', 'MISSION_ACK', `the Executor acknowledged briefing #${ex.mission.id}`); }, 2500);
  if (m.abort) { ex.targets = []; ex.abort = true; nextLeg(); return; }
  planTargets(m.waypoints.map((w) => Number(w.beacon_id)));
  if (m.return_to_exit === false) ex.stay = true;
}
function autoTargets() {
  const kinds = ['VICTIM', 'GAS', 'RADIATION', 'THERMAL', 'STRUCTURAL'];
  const ids = B.filter((b) => kinds.includes(b.kind)).sort((a, b) => kinds.indexOf(a.kind) - kinds.indexOf(b.kind) || a.id - b.id);
  return ids.map((b) => b.id);
}

// ------------------------------------------------------------ main loop
let lastStatus = 0; let lastRobot = 0; let lastPoll = 0;
console.log(`[mock-mine] ${B.length} beacons from ${path.basename(MISSION)}, mission time x${SPEED} -> ${TARGET}`);
async function tick() {
  const t = missionS(); const now = Date.now();
  for (const b of B) {
    if (b.t <= t && !posted.has(b.id)) {
      posted.add(b.id);
      await post('/api/beacon', payload(b));
      if (KIND_EVENT[b.kind]) {
        const what = b.kind === 'PHOSPHATE' ? `${(b.rec.severity / 4).toFixed(1)} % P2O5` : (b.detail || '');
        await onaEvent('info', KIND_EVENT[b.kind], b.kind === 'SEARCHED' ? `area searched: ${b.detail || ''}`
          : `${b.kind.toLowerCase()} marked by beacon #${b.id} (${what})`, { beacon_id: b.id, kind: b.kind });
      }
    }
  }
  if (now - lastStatus > 2000) {
    lastStatus = now;
    for (const g of gws) {
      await post('/api/network-health', { gateway_id: g, status: 'online', via: 'ona', known: posted.size,
        rx_ok: posted.size * 3, rejected: 0, ...ll(gwLocal[g][0], gwLocal[g][1]), alive: true, agreement: 1 });
    }
    await post('/api/ona-status', {
      name: 'ONA-Gafsa-mine (mock)', site: SITE, quorum: 2, link: { state: 'LTE', pending: 0, sent_by_link: { LTE: posted.size } },
      gateways: gws.map((g) => ({ gw: g, ...ll(gwLocal[g][0], gwLocal[g][1]), alt: 300 + gwLocal[g][2], alive: true, agreement: 1,
        reports: posted.size, agreed: posted.size, disagreed: 0, invalid: 0, quarantined: false })),
      records: { confirmed: posted.size, unconfirmed: 0, conflicts: 0 }, calibration: { rms_m: 0 },
      mission: ex.mission && !ex.mission.abort ? { mission_id: ex.mission.id, targets: ex.mission.waypoints.map((w) => w.beacon_id),
        return_to_exit: ex.mission.return_to_exit !== false, tx_count: 2, acked_by: ex.ack ? [1] : [],
        progress: { done: ex.done, total: ex.total, phase: ex.phase } } : null,
    });
  }
  if (now - lastRobot > 1000 / Math.min(4, SPEED)) {
    lastRobot = now;
    if (t < writerEnd) {
      const [x, y, yaw] = writerAt(t);
      await robotStatus('WRITER', 2, x, y, yaw, { phase: t < writerEnd - 25 ? 'FOLLOW' : 'HOME', odo_m: Math.round(t * 0.42),
        total: 9, done: B.filter((b) => b.kind === 'SEARCHED' && posted.has(b.id)).length });
    } else if (t < writerEnd + 6) {
      await robotStatus('WRITER', 2, 0, 0, Math.PI, { phase: 'DONE', total: 9, done: 9 });
    } else {
      if (ex.started == null) {
        ex.started = t; ex.x = B[0].drop[0]; ex.y = B[0].drop[1];
        console.log('[mock-mine] the Executor waits at the portal for a briefing (Dispatch mission on the dashboard)');
      }
      if (now - lastPoll > 2000) { lastPoll = now; await briefingCheck(t); }
      if (ex.phase === 'WAIT' && t - ex.started > WAIT_S) {
        await onaEvent('info', 'NO_BRIEFING', 'no briefing within a minute: the Executor goes for the trapped miner first, then every danger');
        planTargets(autoTargets());
      }
      const dt = 1 / Math.min(4, SPEED) * SPEED;
      if (ex.phase === 'GOTO' || ex.phase === 'RETURN') {
        ex.s = Math.min(ex.leg.len, ex.s + 0.6 * dt);
        [ex.x, ex.y, ex.yaw] = along(ex.leg.pts, ex.s);
        if (ex.s >= ex.leg.len) {
          ex.at = ex.leg.ids[ex.leg.ids.length - 1] ?? ex.at;
          if (ex.phase === 'GOTO') { ex.phase = 'TREAT'; ex.treatUntil = t + 4; } else ex.phase = 'DONE';
        }
      } else if (ex.phase === 'TREAT' && t >= ex.treatUntil) {
        ex.done += 1; ex.targets.shift();
        if (!ex.targets.length && ex.stay) ex.phase = 'DONE'; else nextLeg();
      }
      await robotStatus('EXECUTOR', 1, ex.x, ex.y, ex.yaw, { phase: ex.phase, mission_id: ex.mission ? ex.mission.id : null,
        mission_ack: ex.ack, done: ex.done, total: ex.total, last_beacon: ex.at, odo_m: Math.round(ex.s) });
      await post('/api/executor-status', { lat: ll(ex.x, ex.y).lat, lon: ll(ex.x, ex.y).lon, heading: ex.yaw, status: ex.phase.toLowerCase() });
    }
  }
}
(async function loop() { await tick(); setTimeout(loop, 250); })();
