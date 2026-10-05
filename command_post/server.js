// =====================================================================
//  Living Map - Command Post server
//  Express (HTTP API) + Socket.io (live push) + static dashboard
//
//  Data contract (do not change - the robot side posts exactly this):
//    POST /api/beacon            -> store (latest per beacon, 200 beacons) + emit "beacon:new"
//    POST /api/executor-status   -> emit "executor:update"
//    POST /api/network-health    -> emit "network:health"
//    GET  /api/beacons           -> last 200 beacons (latest record of each)
//    POST /api/mission-dispatch  -> store latestMission, 200 OK
//  Extras (additive, harmless):
//    GET  /api/mission-dispatch/latest, GET /api/state, GET /api/health
//  from the Outside Network Area (ona/, python -m ona):
//    POST /api/robot-status      -> robot position: gateways' spheres + EKF, its own pose, ranges
//    POST /api/ona-status        -> link (LTE / satellite), queue, votes, gateways, calibration
//    POST /api/ona-event         -> alerts and decisions (lying gateway, SLAM drift, briefing ack...)
//
//  Run:   npm start            (real data)
//         npm run demo         (server + built-in mock feeder)
//         npm run demo:mine    (server + the Gafsa mine mission replayed: mock_mine.js)
//         SITE=gafsa_mine npm start   (the mine plan without the ONA, e.g. with beacon_record_node)
// =====================================================================
const express = require('express');
const http = require('http');
const path = require('path');
const os = require('os');
const { Server } = require('socket.io');

const PORT = Number(process.env.PORT) || 3000;
const MAX_BEACONS = 200;
const MAX_EVENTS = 100;      // recent events, to backfill the feed of a new dashboard tab
// gateways the dashboard should expect to hear from (shown grey until they do)
const EXPECTED_GATEWAYS = (process.env.GATEWAYS || 'gw1,gw2,gw3')
  .split(',').map((s) => s.trim()).filter(Boolean);

// a named site: its plan is drawn on the dashboard (the ONA sends the same in its status)
const SITES = {
  gafsa_mine: { id: 'gafsa_mine', name: 'Gafsa phosphate basin: simulated room-and-pillar panel (Metlaoui area)',
    plan: 'sites/gafsa_mine.geojson', underground: true },
};
const DEMO_MINE = process.argv.includes('--demo-mine');
const site = SITES[process.env.SITE || (DEMO_MINE ? 'gafsa_mine' : '')] || null;

const app = express();
const server = http.createServer(app);
const io = new Server(server, { cors: { origin: '*' } });

app.use(express.json({ limit: '1mb' }));
app.use((req, res, next) => {            // allow the robot laptop to POST from anywhere
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
  res.setHeader('Access-Control-Allow-Methods', 'GET,POST,OPTIONS');
  if (req.method === 'OPTIONS') return res.sendStatus(204);
  next();
});
app.use(express.static(path.join(__dirname, 'public')));
app.use('/vendor/leaflet',
  express.static(path.join(__dirname, 'node_modules', 'leaflet', 'dist')));

// ------------------------------------------------------------ in-memory state
const beacons = new Map();   // beacon_id -> latest event (Map order = recency)
const events = [];           // last MAX_EVENTS events (updates included)
let executor = null;         // last executor status
const health = {};           // gateway_id -> last health message
let latestMission = null;
const robots = {};           // robot_id -> last robot-status (from the ONA)
let ona = null;              // last ONA status
const onaEvents = [];        // last MAX_EVENTS ONA events
const nowS = () => Date.now() / 1000;
const num = (v) => { const n = Number(v); return Number.isFinite(n) ? n : null; };

// ------------------------------------------------------------ endpoints
app.post('/api/beacon', (req, res) => {
  const b = req.body || {};
  const lat = num(b.lat); const lon = num(b.lon);
  if (lat === null || lon === null) {
    return res.status(400).json({ ok: false, error: 'lat and lon are required numbers' });
  }
  const beacon = {
    ...b,
    lat, lon,
    beacon_id: b.beacon_id ?? `auto-${Date.now()}`,
    ttl: num(b.ttl) ?? 20,
    timestamp: num(b.timestamp) ?? nowS(),
    received_at: nowS(),
  };
  // a compact record that came over the satellite link: rebuild what was left out
  if (beacon.v === 2 && beacon.kind) {
    if (beacon.event_type == null) beacon.event_type = String(beacon.kind).toLowerCase();
    if (beacon.value == null) beacon.value = beacon.severity ?? 0;
    if (beacon.eff_conf == null && beacon.half_life_s) {
      const age = Math.max(0, nowS() - beacon.timestamp) * (num(beacon.time_scale) || 1);
      beacon.age_s = beacon.age_s ?? Math.round(age);
      beacon.eff_conf = +((num(beacon.confidence) ?? 1) * Math.pow(2, -age / beacon.half_life_s)).toFixed(3);
      beacon.ttl = Math.max(0, Math.min(20, Math.round(20 * beacon.eff_conf)));
    }
  }
  // one entry per beacon: an update (new seq, aging, "lost") replaces the old one
  const key = String(beacon.beacon_id);
  // a version only one ONA gateway reported never hides a version two gateways confirmed:
  // it is kept next to it as "pending" until it is confirmed (or forgotten)
  const prev = beacons.get(key);
  const confirmed = (x) => !x.ona || x.ona.state === 'confirmed';
  if (prev && confirmed(prev) && beacon.ona && beacon.ona.state === 'unconfirmed') {
    const kept = { ...prev, pending: { seq: beacon.seq, severity: beacon.severity, ona: beacon.ona, received_at: beacon.received_at } };
    beacons.set(key, kept);
    io.emit('beacon:new', { ...kept, ev: 'pending' });
    log('beacon', `#${key} unconfirmed newer version (seq ${beacon.seq}, ${beacon.ona.votes}/${beacon.ona.of}) kept aside`);
    return res.json({ ok: true, stored: beacons.size, pending: true });
  }
  beacons.delete(key);
  beacons.set(key, beacon);
  while (beacons.size > MAX_BEACONS) beacons.delete(beacons.keys().next().value);
  events.push(beacon);
  while (events.length > MAX_EVENTS) events.shift();
  io.emit('beacon:new', beacon);
  const v2 = beacon.v === 2 ? ` seq ${beacon.seq} conf ${beacon.eff_conf}${beacon.silent ? ' LOST?' : ''}${beacon.stale ? ' stale' : ''}` +
    (beacon.ona ? ` [${beacon.ona.state} ${beacon.ona.votes}/${beacon.ona.of}]` : '') : '';
  log('beacon', `#${beacon.beacon_id} ${beacon.event_type} v=${beacon.value}${v2} via ${beacon.gateway_id || '?'}`);
  res.json({ ok: true, stored: beacons.size });
});

app.post('/api/executor-status', (req, res) => {
  const e = req.body || {};
  const lat = num(e.lat); const lon = num(e.lon);
  if (lat === null || lon === null) {
    return res.status(400).json({ ok: false, error: 'lat and lon are required numbers' });
  }
  executor = { ...e, lat, lon, heading: num(e.heading) ?? 0, received_at: nowS() };
  io.emit('executor:update', executor);
  res.json({ ok: true });
});

app.post('/api/network-health', (req, res) => {
  const h = req.body || {};
  if (!h.gateway_id) return res.status(400).json({ ok: false, error: 'gateway_id required' });
  health[h.gateway_id] = { ...h, received_at: nowS() };
  io.emit('network:health', health[h.gateway_id]);
  res.json({ ok: true });
});

app.get('/api/beacons', (_req, res) => res.json([...beacons.values()]));

// ------------------------------------------------------------ Outside Network Area
app.post('/api/robot-status', (req, res) => {
  const r = req.body || {};
  const lat = num(r.lat); const lon = num(r.lon);
  if (r.robot_id == null || lat === null || lon === null) {
    return res.status(400).json({ ok: false, error: 'robot_id, lat and lon are required' });
  }
  let rr = { ...r, lat, lon, received_at: nowS() };
  if (r.via === 'sat') {
    // satellite summary: keep the last full picture (battery, own pose...), but no stale spheres
    const prev = robots[r.robot_id] || {};
    const sd = num(r.sigma_m);
    rr = { ...prev, ...rr, ranges: [], reported: undefined, fix: undefined,
      measured: r.source === 'gateways' && sd !== null ? { lat, lon, sigma_m: sd, ellipse: { a_m: sd, b_m: sd, angle_rad: 0 } } : undefined };
  }
  robots[r.robot_id] = rr;
  io.emit('robot:update', robots[r.robot_id]);
  res.json({ ok: true });
});

app.post('/api/ona-status', (req, res) => {
  ona = { ...(req.body || {}), received_at: nowS() };
  io.emit('ona:status', ona);
  res.json({ ok: true });
});

app.post('/api/ona-event', (req, res) => {
  const e = { ...(req.body || {}), received_at: nowS() };
  if (!e.event) return res.status(400).json({ ok: false, error: 'event required' });
  onaEvents.push(e);
  while (onaEvents.length > MAX_EVENTS) onaEvents.shift();
  io.emit('ona:event', e);
  log('ona', `${String(e.level || 'info').toUpperCase()} ${e.event}: ${e.text || ''}`);
  res.json({ ok: true });
});

app.post('/api/mission-dispatch', (req, res) => {
  const m = req.body || {};
  // {abort: true} with no waypoints calls the Executor back to the exit (through the ONA)
  if (!Array.isArray(m.waypoints) || (m.waypoints.length === 0 && !m.abort)) {
    return res.status(400).json({ ok: false, error: 'waypoints must be a non-empty array' });
  }
  latestMission = { ...m, dispatched_at: num(m.dispatched_at) ?? nowS(), received_at: nowS() };
  io.emit('mission:dispatched', latestMission);
  log('mission', m.abort ? 'ABORT: the Executor is called back to the exit' :
    `dispatched ${m.waypoints.length} waypoint(s): ` + m.waypoints.map((w) => `#${w.beacon_id}`).join(' -> ') +
    (m.return_to_exit === false ? ' (stay there)' : ''));
  res.status(200).json({ ok: true });
});

app.get('/api/mission-dispatch/latest', (_req, res) => res.json(latestMission));

// one call to hydrate the dashboard on load / reconnect
app.get('/api/state', (_req, res) => res.json({
  beacons: [...beacons.values()], events, executor, health, latestMission,
  robots, ona, onaEvents, site: (ona && ona.site) || site,
  expectedGateways: EXPECTED_GATEWAYS, serverTime: nowS(),
}));
app.get('/api/health', (_req, res) => res.json({ ok: true, beacons: beacons.size, uptime_s: process.uptime() }));

// ------------------------------------------------------------ misc
io.on('connection', (socket) => {
  log('ws', `dashboard connected (${io.engine.clientsCount} open)`);
  socket.on('disconnect', () => log('ws', `dashboard disconnected (${io.engine.clientsCount} open)`));
});

function log(tag, msg) {
  const t = new Date().toTimeString().slice(0, 8);
  console.log(`[${t}] ${tag.padEnd(7)} ${msg}`);
}

function lanAddresses() {
  const out = [];
  for (const list of Object.values(os.networkInterfaces())) {
    for (const a of list || []) if (a.family === 'IPv4' && !a.internal) out.push(a.address);
  }
  return out;
}

server.listen(PORT, '0.0.0.0', () => {
  console.log('==============================================================');
  console.log(' Living Map - Command Post');
  console.log(`   Dashboard : http://localhost:${PORT}`);
  for (const ip of lanAddresses()) console.log(`   On LAN    : http://${ip}:${PORT}   <- robot posts here`);
  console.log(`   Gateways  : ${EXPECTED_GATEWAYS.join(', ')}`);
  if (site) console.log(`   Site      : ${site.name}`);
  console.log('==============================================================');
  if (process.argv.includes('--demo')) {
    process.env.TARGET = process.env.TARGET || `http://127.0.0.1:${PORT}`;
    require('./mock_feeder.js');
  } else if (DEMO_MINE) {
    process.env.TARGET = process.env.TARGET || `http://127.0.0.1:${PORT}`;
    require('./mock_mine.js');
  }
});
