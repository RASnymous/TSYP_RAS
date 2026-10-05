/* =====================================================================
   Living Map - Command Post dashboard (plain JS + Leaflet + Socket.io)
   Understands both the original contract (event_type/value/ttl 0..20) and
   the LMB2 beacon records forwarded by the LoRa gateway bridge (v: 2):
   kinds, severity, confidence half-life, next-hop route, lost/stale flags.
   the Outside Network Area (ONA) - three gateways, 2-of-3 vote on every
   record, robots positioned by the gateways' distance spheres (like GPS),
   LTE / satellite uplink, signed briefings to the Executor.
   Sites: the ONA (or SITE=gafsa_mine on the server) can name a site. An
   underground site (the Gafsa mine) gets its mine plan as the map, mine
   wording, measured values decoded from each record's severity, the victim
   search by area, and the resources (phosphate seams, finds) on their own layer.
   ===================================================================== */
(() => {
  'use strict';

  // ------------------------------------------------------------ config
  const CFG = {
    center: [36.8065, 10.1815],   // Tunis default - the map jumps to the first beacon anyway
    zoom: 19,
    ttlMax: 20,
    ttlStepS: 6,           // v1 client-side aging: TTL drops by 1 every N seconds since last heard
    gatewayTimeoutS: 10,   // gateway goes red if silent for this long
    execStaleS: 6,
    feedMax: 60,
    beaconMax: 200,
    trailMax: 150,
    liveMin: 0.05,         // below this effective confidence a record is "aged out"
    robotStaleS: 10,       // ONA robot status older than this: greyed
    onaStaleS: 12,         // ONA status older than this: "not heard"
    ellipseK: 2.45,        // 1-sigma ellipse x 2.45 = 95 % area
  };
  const GW_PALETTE = ['#fbbf24', '#2dd4bf', '#818cf8', '#f472b6', '#a3e635'];
  const ROLES = {
    EXECUTOR: { label: 'Executor', color: '#22d3ee' },
    WRITER:   { label: 'Writer',   color: '#e2e8f0' },
  };
  const roleOf = (r) => ROLES[String(r || '').toUpperCase()] || { label: 'Robot', color: '#94a3b8' };
  const TYPES = {
    victim:      { label: 'Victim',      color: '#ec4899', icon: '✚', hazard: false },
    gas:         { label: 'Gas',         color: '#a3e635', icon: '☁', hazard: true },
    radiation:   { label: 'Radiation',   color: '#ef4444', icon: '☢', hazard: true },
    thermal:     { label: 'Thermal',     color: '#f97316', icon: '🔥', hazard: true },
    fire:        { label: 'Thermal',     color: '#f97316', icon: '🔥', hazard: true },
    structural:  { label: 'Collapse risk', color: '#eab308', icon: '⚠', hazard: true },
    obstruction: { label: 'Obstruction', color: '#a8a29e', icon: '▦', hazard: true },
    exit:        { label: 'Exit',        color: '#22c55e', icon: '⇱', hazard: false },
    waypoint:    { label: 'Trail',       color: '#3b82f6', icon: '◆', hazard: false },
    // the mine: resources are marked, never avoided; a searched area is a victim-search marker
    phosphate:   { label: 'Phosphate seam', color: '#d4a373', icon: '▤', hazard: false, resource: true },
    gold:        { label: 'Gold find',      color: '#facc15', icon: '✦', hazard: false, resource: true },
    gemstone:    { label: 'Gemstone find',  color: '#2dd4bf', icon: '❖', hazard: false, resource: true },
    searched:    { label: 'Area searched',  color: '#86efac', icon: '✓', hazard: false, search: true },
  };
  // underground, the same kinds are said the way a mine rescue team says them
  const MINE_LABEL = { radiation: 'Radon / radiation', gas: 'Bad air (gas)', structural: 'Unstable roof',
    thermal: 'Fire', fire: 'Fire', victim: 'Trapped miner', obstruction: 'Blocked gallery' };
  const typeOf = (t) => {
    const base = TYPES[t] || { label: t ? cap(t) : 'Unknown', color: '#94a3b8', icon: '●', hazard: false };
    return state.site && state.site.underground && MINE_LABEL[t] ? { ...base, label: MINE_LABEL[t] } : base;
  };
  const gradeClass = (g) => (g > 27 ? 'rich' : (g >= 22 ? 'medium' : (g >= 15 ? 'poor' : 'waste')));
  // what a record's severity means underground (mine_site.py / mission_log.py: one scale per kind)
  function mineMeasure(b) {
    const s = Number(b.severity) || 0;
    switch (kindKey(b)) {
      case 'gas': return `${(s / 20).toFixed(1)}× the alarm level`;
      case 'radiation': return `radon ≈ ${Math.round(s * 50)} Bq/m³`;
      case 'thermal': case 'fire': return `hot spot ≈ ${Math.round(s * 6)} °C`;
      case 'structural': return `roof sag ≈ ${Math.round(s * 0.6)} mm, cracks`;
      case 'phosphate': return `${(s / 4).toFixed(1)} % P₂O₅ · ${gradeClass(s / 4)}`;
      case 'searched': return `${s} % of the area seen by the thermal camera`;
      case 'gold': case 'gemstone': return 'signal · sample it';
      case 'victim': return 'body heat on the thermal camera';
      default: return null;
    }
  }
  const kindKey = (b) => String(b.kind || b.event_type || 'unknown').toLowerCase();

  // ------------------------------------------------------------ helpers
  const $ = (id) => document.getElementById(id);
  const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  function cap(s) { return s.charAt(0).toUpperCase() + s.slice(1); }
  const pad = (n) => String(n).padStart(2, '0');
  const hms = (d) => `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
  const tsDate = (ts) => new Date((ts > 1e12 ? ts : ts * 1000));
  const fmtVal = (v) => (typeof v === 'number' ? (Number.isInteger(v) ? String(v) : v.toFixed(2)) : esc(v ?? '—'));
  function ago(ms) {
    const s = Math.max(0, Math.round(ms / 1000));
    if (s < 60) return `${s}s`;
    if (s < 3600) return `${Math.floor(s / 60)}m${pad(s % 60)}`;
    return `${Math.floor(s / 3600)}h${pad(Math.floor((s % 3600) / 60))}`;
  }
  function fmtDur(s) {
    s = Math.max(0, Number(s) || 0);
    if (s < 90) return `${Math.round(s)} s`;
    if (s < 5400) return `${Math.round(s / 60)} min`;
    if (s < 172800) return `${+(s / 3600).toFixed(s < 36000 ? 1 : 0)} h`;
    return `${+(s / 86400).toFixed(1)} d`;
  }
  const pct = (f) => `${Math.round(f * 100)}%`;

  // ------------------------------------------------------------ state
  const state = {
    beacons: new Map(),   // id -> { id, data, marker, heardAt, lostTag }
    selected: [],         // ordered beacon ids (strings)
    planMode: false,
    follow: false,
    executor: null, execMarker: null, execTrail: null, execHeardAt: 0,
    gateways: new Map(),  // id -> { lastSeen, status, info }
    feedCount: 0,
    startedAt: Date.now(),
    centered: false,
    // outside network area
    robots: new Map(),    // robot_id -> { r, heardAt, marker, trail, ellipse, ghost, ghostLine, rings, tag }
    ona: null, onaHeardAt: 0,
    gwMarkers: new Map(), // gateway id -> L.marker
    gwOrder: [],          // gateway ids in first-heard order (colours)
    spheres: true,
    seenOnaEvents: new Set(),
    // site (the Gafsa mine)
    site: null,
    zones: new Map(),     // zone id -> { id, name, layer, rings: [[latlng...]], status }
    showRes: true, showSearch: true,
  };

  // ------------------------------------------------------------ map
  const map = L.map('map', { zoomControl: false, preferCanvas: false, maxZoom: 22 })
    .setView(CFG.center, CFG.zoom);
  L.control.zoom({ position: 'bottomright' }).addTo(map);
  L.control.scale({ position: 'bottomright', imperial: false, maxWidth: 120 }).addTo(map);

  const dark = L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
    maxNativeZoom: 19, maxZoom: 22, subdomains: 'abcd',
    attribution: '&copy; OpenStreetMap &copy; CARTO',
  });
  const sat = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxNativeZoom: 19, maxZoom: 22, attribution: 'Imagery &copy; Esri',
  });
  const osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxNativeZoom: 19, maxZoom: 22, attribution: '&copy; OpenStreetMap',
  });
  // Offline fallback: a tactical metric grid drawn locally (no internet needed)
  const TacticalGrid = L.GridLayer.extend({
    createTile() {
      const tile = document.createElement('canvas');
      const size = this.getTileSize();
      tile.width = size.x; tile.height = size.y;
      const ctx = tile.getContext('2d');
      ctx.fillStyle = '#0b1118'; ctx.fillRect(0, 0, size.x, size.y);
      ctx.strokeStyle = 'rgba(56,189,248,0.07)'; ctx.lineWidth = 1;
      for (let i = 0; i <= 256; i += 32) {
        ctx.beginPath(); ctx.moveTo(i + .5, 0); ctx.lineTo(i + .5, 256); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, i + .5); ctx.lineTo(256, i + .5); ctx.stroke();
      }
      ctx.strokeStyle = 'rgba(56,189,248,0.16)';
      ctx.strokeRect(.5, .5, 255, 255);
      return tile;
    },
  });
  const grid = new TacticalGrid({ maxZoom: 22 });

  // underground: the mine plan is the map; a dark rock tone instead of streets
  const RockGrid = L.GridLayer.extend({
    createTile() {
      const tile = document.createElement('canvas');
      const size = this.getTileSize();
      tile.width = size.x; tile.height = size.y;
      const ctx = tile.getContext('2d');
      ctx.fillStyle = '#0d0b09'; ctx.fillRect(0, 0, size.x, size.y);
      ctx.strokeStyle = 'rgba(212,163,115,0.06)'; ctx.lineWidth = 1;
      for (let i = 0; i <= 256; i += 32) {
        ctx.beginPath(); ctx.moveTo(i + .5, 0); ctx.lineTo(i + .5, 256); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, i + .5); ctx.lineTo(256, i + .5); ctx.stroke();
      }
      return tile;
    },
  });
  const rock = new RockGrid({ maxZoom: 22 });

  dark.addTo(map);
  const layersCtl = L.control.layers({ 'Dark': dark, 'Satellite': sat, 'Street': osm, 'Offline grid': grid }, null,
    { position: 'topleft', collapsed: true }).addTo(map);

  let tileErrors = 0, switchedOffline = false;
  [dark, sat, osm].forEach((layer) => layer.on('tileerror', () => {
    tileErrors += 1;
    if (tileErrors > 4 && !switchedOffline && map.hasLayer(layer)) {
      switchedOffline = true;
      map.removeLayer(layer); grid.addTo(map);
      toast('Map tiles unreachable — switched to offline grid', 'err');
    }
  }));

  const planLayer = L.layerGroup().addTo(map);     // the mine plan: rock, pillars, seams, search areas
  const linkLayer = L.layerGroup().addTo(map);     // next-hop links (way out)
  const focusLayer = L.layerGroup().addTo(map);    // route-out highlight + error circle
  const sphereLayer = L.layerGroup().addTo(map);    // gateway distance circles
  const sphereFx = L.layerGroup().addTo(map);       // the expanding "ping" rings
  const robotLayer = L.layerGroup().addTo(map);     // error ellipses, robot's own estimate
  const gwLayer = L.layerGroup().addTo(map);        // gateway antennas
  const beaconLayer = L.layerGroup().addTo(map);
  const resLayer = L.layerGroup().addTo(map);       // mine: resources (phosphate seams, finds)
  const searchLayer = L.layerGroup().addTo(map);    // mine: "area searched" markers
  const layerOf = (k) => (TYPES[k] && TYPES[k].resource ? resLayer : (TYPES[k] && TYPES[k].search ? searchLayer : beaconLayer));
  const routeLine = L.polyline([], { color: '#38bdf8', weight: 3, dashArray: '8 8', opacity: .9, interactive: false }).addTo(map);

  // ------------------------------------------------------------ aging
  function effectiveTtl(rec) {                        // v1 contract
    const aged = Math.floor((Date.now() - rec.heardAt) / 1000 / CFG.ttlStepS);
    const ttl = Number(rec.data.ttl ?? CFG.ttlMax) - aged;
    return Math.max(0, Math.min(CFG.ttlMax, ttl));
  }
  // 0..1. LMB2: confidence at reception x 2^(-elapsed / half-life), 0 past the hard TTL
  function freshness(rec) {
    const b = rec.data;
    if (b.v !== 2) return effectiveTtl(rec) / CFG.ttlMax;
    if (b.ev === 'expired') return 0;
    const dt = (Date.now() - rec.heardAt) / 1000 * (Number(b.time_scale) || 1);
    const hl = Number(b.half_life_s) || 3600;
    let f = Number(b.eff_conf ?? 1) * Math.pow(2, -dt / hl);
    if (b.ttl_s != null && b.age_s != null && Number(b.age_s) + dt > Number(b.ttl_s)) f = 0;
    return Math.max(0, Math.min(1, f));
  }
  const fadeOpacity = (f) => 0.15 + 0.85 * f;
  const isLive = (rec) => freshness(rec) > CFG.liveMin;

  function styleMarker(rec) {
    const b = rec.data; const k = kindKey(b); const t = typeOf(k);
    const f = freshness(rec); const op = fadeOpacity(f);
    const sel = state.selected.includes(rec.id);
    const color = b.retracted ? '#64748b' : t.color;
    const unconf = b.ona && b.ona.state !== 'confirmed';     // only one gateway reported it so far
    rec.marker.setStyle({
      radius: sel ? 12 : (k === 'waypoint' ? 6 : (k === 'victim' || k === 'exit' ? 10 : 9)),
      color: sel ? '#e0f2fe' : (b.silent ? '#f87171' : (unconf ? '#cbd5e1' : color)),
      weight: sel ? 3 : (b.silent ? 2.5 : 2),
      dashArray: b.silent ? '3 3' : (unconf ? '1 4' : (b.stale ? '2 4' : null)),
      opacity: Math.max(op, sel ? 1 : 0.35),
      fillColor: color,
      fillOpacity: b.stale ? 0.07 : op * (b.retracted ? 0.35 : 1) * (unconf ? 0.45 : 1),
    });
    // "LOST?" tag for beacons whose neighbours stopped hearing them
    if (b.silent && !rec.lostTag) {
      rec.lostTag = L.marker(rec.marker.getLatLng(), {
        interactive: false, keyboard: false,
        icon: L.divIcon({ className: 'lost-tag', html: 'LOST?', iconSize: [42, 16], iconAnchor: [21, 26] }),
      }).addTo(beaconLayer);
    } else if (!b.silent && rec.lostTag) {
      beaconLayer.removeLayer(rec.lostTag); rec.lostTag = null;
    } else if (rec.lostTag) {
      rec.lostTag.setLatLng(rec.marker.getLatLng());
    }
  }

  function pulseAt(latlng, color) {
    const m = L.marker(latlng, {
      interactive: false, keyboard: false,
      icon: L.divIcon({ className: 'pulse-marker', html: `<div class="pulse-ring" style="--c:${color}"></div>`, iconSize: [60, 60], iconAnchor: [30, 30] }),
    }).addTo(map);
    setTimeout(() => map.removeLayer(m), 1700);
  }

  // ------------------------------------------------------------ next-hop route (way out)
  const nextId = (b) => (b && b.next && b.next.id != null ? String(b.next.id) : null);
  function bearingDeg(a, b) {                      // screen-free geographic bearing a -> b
    const dy = b.lat - a.lat;
    const dx = (b.lng - a.lng) * Math.cos(a.lat * Math.PI / 180);
    return (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
  }
  let linksQueued = false;
  function queueLinks() {
    if (linksQueued) return;
    linksQueued = true;
    requestAnimationFrame(() => { linksQueued = false; drawLinks(); });
  }
  function drawLinks() {
    linkLayer.clearLayers();
    for (const rec of state.beacons.values()) {
      const to = state.beacons.get(nextId(rec.data));
      if (!to) continue;
      const a = rec.marker.getLatLng(); const b = to.marker.getLatLng();
      L.polyline([a, b], { color: '#94a3b8', weight: 1.5, opacity: .5, dashArray: '4 6', interactive: false }).addTo(linkLayer);
      const mid = L.latLng((a.lat + b.lat) / 2, (a.lng + b.lng) / 2);
      L.marker(mid, {
        interactive: false, keyboard: false,
        icon: L.divIcon({ className: 'hop-arrow', html: `<i style="transform:rotate(${bearingDeg(a, b).toFixed(0)}deg)">▲</i>`, iconSize: [12, 12], iconAnchor: [6, 6] }),
      }).addTo(linkLayer);
    }
  }
  // follow next pointers: [start, next, next-of-next, ...] (stops at a gap or a loop)
  function routeOut(id) {
    const out = []; const seen = new Set(); let cur = String(id);
    while (cur && !seen.has(cur) && state.beacons.has(cur) && out.length < 64) {
      seen.add(cur); out.push(cur);
      cur = nextId(state.beacons.get(cur).data);
    }
    const missing = cur && !seen.has(cur) && !state.beacons.has(cur) ? cur : null;
    let dist = 0;
    for (let i = 0; i + 1 < out.length; i++) dist += Number(state.beacons.get(out[i]).data.next?.dist_m) || 0;
    return { ids: out, dist, missing };
  }
  function showFocus(id) {
    focusLayer.clearLayers();
    const rec = state.beacons.get(id); if (!rec) return;
    const b = rec.data; const t = typeOf(kindKey(b));
    if (b.v === 2 && b.err_m != null && Number(b.err_m) > 0) {
      L.circle(rec.marker.getLatLng(), { radius: Number(b.err_m), color: t.color, weight: 1, opacity: .7, fillOpacity: .08, dashArray: '4 4', interactive: false }).addTo(focusLayer);
    }
    const r = routeOut(id);
    if (r.ids.length > 1) {
      L.polyline(r.ids.map((x) => state.beacons.get(x).marker.getLatLng()),
        { color: '#22c55e', weight: 4, opacity: .8, interactive: false }).addTo(focusLayer);
    }
  }

  // ------------------------------------------------------------ beacons
  function upsertBeacon(b, { live = false } = {}) {
    if (!Number.isFinite(Number(b.lat)) || !Number.isFinite(Number(b.lon))) return;
    const id = String(b.beacon_id);
    const latlng = [Number(b.lat), Number(b.lon)];
    let heardAt = Date.now();
    if (!live && b.received_at) heardAt = Math.min(Date.now(), b.received_at * 1000);

    let rec = state.beacons.get(id);
    const prev = rec ? rec.data : null;
    if (rec) {
      rec.data = b; rec.heardAt = heardAt;
      rec.marker.setLatLng(latlng);
    } else {
      const marker = L.circleMarker(latlng, { bubblingMouseEvents: false }).addTo(layerOf(kindKey(b)));
      rec = { id, data: b, marker, heardAt, lostTag: null };
      marker.on('click', () => onPinClick(id));
      state.beacons.set(id, rec);
      if (state.beacons.size > CFG.beaconMax) removeBeacon(state.beacons.keys().next().value);
      if (!state.centered && live) { state.centered = true; map.setView(latlng, Math.max(map.getZoom(), 18)); }
    }
    styleMarker(rec);
    queueLinks();
    if (live && b.ev === 'pending') {      // an unconfirmed newer version, kept aside by the server
      if (!prev || !prev.pending || prev.pending.seq !== b.pending.seq) {
        addFeedRow({ ...b, ev: 'pending' }, true);
      }
    } else if (live) {
      const ev = b.ev || 'new';
      const confirmedNow = prev && prev.ona && prev.ona.state !== 'confirmed' && b.ona && b.ona.state === 'confirmed' && prev.seq === b.seq;
      const quiet = ev === 'dump' || ev === 'own' || (prev && prev.v === 2 && ev === 'new' && prev.seq === b.seq && !confirmedNow);
      if (!quiet) {
        pulseAt(latlng, b.silent ? '#f87171' : typeOf(kindKey(b)).color);
        addFeedRow(b, true);
        if (ev === 'silent') toast(`Beacon #${id} went silent — possibly destroyed. Its record is still replicated by its neighbours.`, 'err');
        if (ev === 'revived') toast(`Beacon #${id} heard again`, 'ok');
        if (kindKey(b) === 'victim' && ev === 'new' && !confirmedNow) {
          toast(b.ona && b.ona.state !== 'confirmed'
            ? `Victim reported at beacon #${id} by ${b.ona.votes} gateway — waiting for a second one`
            : `Victim reported at beacon #${id}`, 'err');
        }
      }
    }
    if (state.selected.includes(id)) refreshRoute();
    if (openPopupId === id) showFocus(id);
    if (state.site) queueSite();
  }

  function removeBeacon(id) {
    const rec = state.beacons.get(id);
    if (!rec) return;
    layerOf(kindKey(rec.data)).removeLayer(rec.marker);
    if (rec.lostTag) beaconLayer.removeLayer(rec.lostTag);
    state.beacons.delete(id);
    const i = state.selected.indexOf(id);
    if (i >= 0) { state.selected.splice(i, 1); renderPlanner(); }
    queueLinks();
  }

  function popupHtml(id) {
    const rec = state.beacons.get(id);
    if (!rec) return '';
    const b = rec.data; const k = kindKey(b); const t = typeOf(k);
    const f = freshness(rec);
    const sel = state.selected.includes(id);
    const rows = [];
    const status = [];
    if (b.silent) status.push('<b class="bad">LOST?</b> neighbours stopped hearing it');
    if (b.stale) status.push('event aged out — kept as a route marker');
    if (b.retracted) status.push('cleared (retracted)');
    if (b.ev === 'expired') status.push('aged out');
    if (b.ona && b.ona.state !== 'confirmed') status.push(`<b class="warnc">UNCONFIRMED</b> only ${esc(b.ona.votes)} of ${esc(b.ona.of)} gateways reported this version`);
    if (b.v === 2) {
      const dt = (Date.now() - rec.heardAt) / 1000 * (Number(b.time_scale) || 1);
      const age = (Number(b.age_s) || 0) + dt;
      const meas = state.site && state.site.underground ? mineMeasure(b) : null;
      if (meas) rows.push(['Measured', esc(meas)]);
      else if (t.hazard || k === 'victim') rows.push(['Severity', `${b.severity ?? 0}`]);
      if (k === 'searched') {
        const z = zoneAt(rec.marker.getLatLng());
        if (z) rows.push(['Area', `${esc(z.name)} · <b>${z.victims.length}</b> victim${z.victims.length === 1 ? '' : 's'}`]);
      }
      if (t.resource) rows.push(['Note', '<span class="muted">a resource: marked, never avoided</span>']);
      rows.push(['Confidence', `${pct(f)} <span class="muted">(was ${pct(Number(b.confidence ?? 1))})</span>`]);
      rows.push(['Half-life', fmtDur(b.half_life_s)]);
      rows.push(['Age / TTL', `${fmtDur(age)} / ${fmtDur(b.ttl_s)}`]);
      rows.push(['Position', `${Number(b.lat).toFixed(6)}, ${Number(b.lon).toFixed(6)}`]);
      rows.push(['Accuracy', b.err_m != null ? `± ${Number(b.err_m).toFixed(1)} m (${esc(b.gps_src || 'slam')})` : 'unknown']);
      if (b.next) {
        rows.push(['Next hop', `#${esc(b.next.id)} · ${Number(b.next.dist_m).toFixed(1)} m${b.next.bearing_deg != null ? ` · ${Number(b.next.bearing_deg).toFixed(0)}°` : ''}`]);
      }
      const r = routeOut(id);
      if (r.ids.length > 1) {
        const last = state.beacons.get(r.ids[r.ids.length - 1]);
        const end = last && kindKey(last.data) === 'exit' ? ' (EXIT)' : '';
        const chain = r.ids.length > 6 ? `${r.ids.slice(0, 3).map((x) => '#' + x).join(' → ')} → … → #${r.ids[r.ids.length - 1]}` : r.ids.map((x) => '#' + x).join(' → ');
        rows.push(['Way out', `${chain}${end} · ${r.dist.toFixed(0)} m${r.missing ? ` <span class="muted">(#${esc(r.missing)} unknown)</span>` : ''}`]);
      }
      rows.push(['Heard via', `${esc(b.gateway_id || '?')} · ${b.hops ?? '?'} hop${b.hops === 1 ? '' : 's'}${b.relay != null && b.relay !== b.beacon_id ? ` (last #${esc(b.relay)})` : ''}`]);
      if (b.ona) rows.push(['Gateways', voteHtml(b.ona, true)]);
      if (b.pending) {
        rows.push(['Unconfirmed', `<span class="warnc">seq ${esc(b.pending.seq)}${b.pending.severity != null ? ` (severity ${esc(b.pending.severity)})` : ''} heard by ${esc(b.pending.ona.votes)} gateway${b.pending.ona.votes === 1 ? '' : 's'} only</span> <span class="muted">— shown when a 2nd agrees</span>`]);
      }
      if (b.rssi != null) rows.push(['Link', `${esc(b.rssi)} dBm${b.snr != null ? ` · SNR ${esc(b.snr)} dB` : ''}`]);
      rows.push(['Version', `seq ${esc(b.seq)} · ${tsDate(b.timestamp).toLocaleTimeString()}`]);
    } else {
      const ttl = effectiveTtl(rec);
      rows.push(['Value', fmtVal(b.value)], ['Time', tsDate(b.timestamp).toLocaleString()], ['TTL left', `${ttl} / ${CFG.ttlMax}`]);
      if (b.gateway_id) rows.push(['Gateway', esc(b.gateway_id)]);
      if (b.rssi != null) rows.push(['RSSI', `${esc(b.rssi)} dBm`]);
      rows.push(['Position', `${Number(b.lat).toFixed(5)}, ${Number(b.lon).toFixed(5)}`]);
    }
    const route = b.v === 2 ? routeOut(id) : { ids: [] };
    return `
      <div class="pop-head">
        <div class="fi-icon" style="--c:${t.color}">${t.icon}</div>
        <div class="pop-title">${esc(t.label)}</div>
        <div class="pop-id">#${esc(b.beacon_id)}</div>
      </div>
      ${status.length ? `<div class="pop-status">${status.join('<br>')}</div>` : ''}
      <div class="pop-rows">${rows.map(([a, v]) => `<span>${a}</span><span>${v}</span>`).join('')}</div>
      <div class="ttlbar" title="confidence now"><div style="width:${Math.round(f * 100)}%;background:${t.color};opacity:${fadeOpacity(f)}"></div></div>
      <div class="pop-actions">
        <button class="btn ${sel ? 'ghost' : 'primary'} pop-btn" data-action="toggle-wp" data-id="${esc(id)}">
          ${sel ? `Remove from mission (WP${state.selected.indexOf(id) + 1})` : 'Add to mission'}
        </button>
        ${route.ids.length > 1 ? `<button class="btn ghost pop-btn" data-action="route-in" data-id="${esc(id)}" title="Waypoints from the entrance to this beacon, following the beacon chain">Executor path to here</button>` : ''}
      </div>`;
  }

  let openPopupId = null;
  function openPopup(id) {
    const rec = state.beacons.get(id);
    if (!rec) return;
    openPopupId = id;
    L.popup({ offset: [0, -6], autoPanPadding: [40, 40], maxWidth: 340 })
      .setLatLng(rec.marker.getLatLng()).setContent(popupHtml(id)).openOn(map);
    showFocus(id);
  }
  map.on('popupclose', () => { openPopupId = null; focusLayer.clearLayers(); });

  function onPinClick(id) {
    if (state.planMode) toggleWaypoint(id);
    else openPopup(id);
  }

  // ------------------------------------------------------------ feed
  function feedText(b) {
    const k = kindKey(b); const t = typeOf(k); const id = esc(b.beacon_id);
    if (b.v !== 2) {
      return k === 'waypoint' ? `${esc(t.label)} beacon dropped` : `${esc(t.label)} detected — value <b>${fmtVal(b.value)}</b>`;
    }
    switch (b.ev) {
      case 'pending': return `<span class="warnc">${esc(t.label)} #${id}: newer version heard by 1 gateway</span> — waiting for a 2nd`;
      case 'silent': return `<span class="bad">Beacon #${id} (${esc(t.label)}) went silent</span>`;
      case 'revived': return `Beacon #${id} (${esc(t.label)}) heard again`;
      case 'expired': return `${esc(t.label)} #${id} aged out`;
      case 'update':
        if (b.stale) return `${esc(t.label)} #${id} event aged out → route marker`;
        if (b.retracted) return `${esc(t.label)} #${id} cleared`;
        return `${esc(t.label)} #${id} updated — severity <b>${esc(b.severity)}</b>`;
      default:
        if (k === 'waypoint') return 'Trail beacon dropped';
        if (k === 'exit') return 'Exit / staging beacon';
        if (b.ona && b.ona.state !== 'confirmed') return `<span class="warnc">Unconfirmed ${esc(t.label.toLowerCase())}</span> — severity <b>${esc(b.severity)}</b>, 1 gateway only`;
        if (state.site && state.site.underground && mineMeasure(b)) {
          if (k === 'searched') {
            const z = zoneAt(L.latLng(Number(b.lat), Number(b.lon)));
            return `Area searched${z ? `: ${esc(z.name)}` : ''} — <b>${esc(b.severity)} %</b> seen`;
          }
          return `${esc(t.label)} — <b>${esc(mineMeasure(b))}</b>`;
        }
        return `${esc(t.label)} detected — severity <b>${esc(b.severity)}</b>`;
    }
  }
  function addFeedRow(b, animate) {
    const empty = $('feedEmpty'); if (empty) empty.remove();
    const t = typeOf(kindKey(b));
    const li = document.createElement('li');
    li.className = 'feed-item' + (animate ? ' new' : '') + (b.silent ? ' warn' : '') + (b.ev === 'pending' ? ' alert' : '');
    li.dataset.id = String(b.beacon_id);
    const sub = b.v === 2
      ? `#${esc(b.beacon_id)} · ${esc(b.gateway_id || 'direct')}${(b.ev === 'pending' && b.pending) ? voteHtml(b.pending.ona) : (b.ona ? voteHtml(b.ona) : '')} · ${b.hops ?? '?'} hop${b.hops === 1 ? '' : 's'}${b.rssi != null ? ` · ${esc(b.rssi)} dBm` : ''} · conf ${pct(Number(b.eff_conf ?? 1))}${b.err_m != null ? ` · ±${Number(b.err_m).toFixed(1)} m` : ''}`
      : `#${esc(b.beacon_id)} · ${esc(b.gateway_id || 'direct')}${b.rssi != null ? ` · ${esc(b.rssi)} dBm` : ''} · ttl ${esc(b.ttl)}`;
    li.innerHTML = `
      <div class="fi-icon" style="--c:${b.silent ? '#f87171' : t.color}">${t.icon}</div>
      <div class="fi-main">
        <div class="fi-title">${feedText(b)}</div>
        <div class="fi-sub">${sub}</div>
      </div>
      <div class="fi-time">${hms(b.received_at ? new Date(Math.min(Date.now(), b.received_at * 1000)) : new Date())}</div>`;
    li.addEventListener('click', () => {
      const rec = state.beacons.get(li.dataset.id);
      if (!rec) return;
      map.flyTo(rec.marker.getLatLng(), Math.max(map.getZoom(), 19), { duration: .6 });
      setTimeout(() => openPopup(li.dataset.id), 650);
    });
    const feed = $('feed');
    feed.prepend(li);
    while (feed.children.length > CFG.feedMax) feed.lastElementChild.remove();
    feed.scrollTop = 0;
    state.feedCount += 1;
    $('feedCount').textContent = `${state.feedCount} event${state.feedCount === 1 ? '' : 's'}`;
  }

  // ------------------------------------------------------------ stats
  function updateStats() {
    let victims = 0, hazards = 0, lost = 0;
    for (const rec of state.beacons.values()) {
      const b = rec.data; const k = kindKey(b);
      const active = isLive(rec) && !b.stale && !b.retracted && !(b.ona && b.ona.state !== 'confirmed');
      if (k === 'victim' && active) victims += 1;
      else if (typeOf(k).hazard && active) hazards += 1;
      if (b.silent) lost += 1;
    }
    $('stTotal').textContent = state.beacons.size;
    $('stVictim').textContent = victims;
    $('stHazard').textContent = hazards;
    $('stLost').textContent = lost;
    $('stLostTile').classList.toggle('alert', lost > 0);
  }

  // ------------------------------------------------------------ executor
  const execIcon = L.divIcon({
    className: 'exec-marker', iconSize: [34, 34], iconAnchor: [17, 17],
    html: `<div class="exec-rot"><svg viewBox="0 0 34 34" width="34" height="34">
      <circle cx="17" cy="17" r="15" fill="rgba(34,211,238,.14)" stroke="#22d3ee" stroke-width="1.5"/>
      <path d="M17 5 L25 26 L17 21 L9 26 Z" fill="#22d3ee" stroke="#04121c" stroke-width="1"/></svg></div>`,
  });

  function updateExecutor(e) {
    const latlng = [Number(e.lat), Number(e.lon)];
    state.executor = e; state.execHeardAt = Date.now();
    if (!state.execMarker) {
      state.execTrail = L.polyline([], { color: '#22d3ee', weight: 2, opacity: .55, interactive: false }).addTo(map);
      state.execMarker = L.marker(latlng, { icon: execIcon, zIndexOffset: 1000, keyboard: false })
        .addTo(map).bindTooltip('Executor', { direction: 'top', offset: [0, -16] });
    } else {
      state.execMarker.setLatLng(latlng);
    }
    // robot heading: radians, 0 = east, CCW+  ->  CSS rotation: deg clockwise from north
    const deg = 90 - (Number(e.heading) || 0) * 180 / Math.PI;
    const el = state.execMarker.getElement();
    if (el) el.querySelector('.exec-rot').style.transform = `rotate(${deg}deg)`;
    const pts = state.execTrail.getLatLngs(); pts.push(latlng);
    if (pts.length > CFG.trailMax) pts.shift();
    state.execTrail.setLatLngs(pts);

    $('execLat').textContent = latlng[0].toFixed(5);
    $('execLon').textContent = latlng[1].toFixed(5);
    const hdg = ((((Number(e.heading) || 0) * 180 / Math.PI) % 360) + 360) % 360;
    $('execHdg').textContent = `${hdg.toFixed(0)}°`;
    if (state.follow) map.panTo(latlng, { animate: true });
    refreshRoute();
  }

  function tickExecutor() {
    const chip = $('execStatus');
    if (!state.executor) return;
    const age = (Date.now() - state.execHeardAt) / 1000;
    $('execAge').textContent = `${ago(age * 1000)} ago`;
    const stale = age > CFG.execStaleS;
    chip.textContent = stale ? 'stale' : (state.executor.status || 'online');
    chip.className = 'chip ' + (stale ? 'stale' : 'ok');
  }

  // ------------------------------------------------------------ gateways
  function noteGateway(h) {
    const g = state.gateways.get(h.gateway_id) || {};
    const prevRej = g.info ? Number(g.info.rejected) || 0 : 0;
    const wasQuar = g.info && g.info.quarantined;
    g.lastSeen = Date.now(); g.status = h.status || 'online'; g.info = h;
    state.gateways.set(h.gateway_id, g);
    if (h.lat != null && h.lon != null) upsertGatewayMarker(h.gateway_id, h);
    if (h.quarantined && !wasQuar && g.lastSeen) { /* the ONA event carries the toast */ }
    const rej = Number(h.rejected) || 0;
    if (rej > prevRej) {
      const why = Object.keys(h.reject_reasons || {}).join(', ') || 'bad frame';
      toast(`⚠ ${h.gateway_id} rejected ${rej - prevRej} unauthenticated frame${rej - prevRej > 1 ? 's' : ''} (${why}) — spoofing attempt?`, 'err');
    }
    renderGateways();
  }
  function renderGateways() {
    const list = $('healthList');
    const ids = [...state.gateways.keys()].sort();
    list.innerHTML = ids.map((id) => {
      const g = state.gateways.get(id);
      const seen = g.lastSeen ? (Date.now() - g.lastSeen) : Infinity;
      const i = g.info || {};
      // on satellite the ONA stops sending gateway health (only essentials go out): not a dead gateway
      const viaSat = i.via === 'ona' && String(state.onaLink || '').toUpperCase().startsWith('SAT');
      const online = g.lastSeen && (seen <= CFG.gatewayTimeoutS * 1000 || viaSat) && g.status === 'online';
      const cls = g.lastSeen ? (online ? 'online' : 'offline') : '';
      const age = g.lastSeen ? (viaSat && seen > CFG.gatewayTimeoutS * 1000 ? 'sat' : ago(seen)) : 'waiting';
      const tip = [online ? 'Heard recently' : 'No heartbeat for 10s+',
        i.known != null ? `${i.known} records` : '', i.rx_ok != null ? `${i.rx_ok} frames ok` : '',
        i.rejected ? `${i.rejected} rejected` : '', i.duty != null ? `duty ${(i.duty * 100).toFixed(2)}%` : '']
        .filter(Boolean).join(' · ');
      const extra = i.known != null ? `<span class="gw-known" title="records held">${esc(i.known)}</span>` : '';
      const rej = i.rejected ? `<span class="gw-rej" title="frames rejected (HMAC)">⚠${esc(i.rejected)}</span>` : '';
      const quar = i.quarantined ? '<span class="gw-quar" title="the ONA ignores this gateway: it disagreed with the majority">IGNORED</span>' : '';
      const agree = i.agreement != null && !i.quarantined && i.agreement < 1 ? `<span class="gw-rej" title="share of its reports the other gateways confirmed">${Math.round(i.agreement * 100)}%</span>` : '';
      return `<div class="gw ${cls}${i.quarantined ? ' quar' : ''}" title="${esc(tip)}${i.agreement != null ? ` · agrees ${Math.round(i.agreement * 100)}%` : ''}">
        <i></i><span class="gw-name">${esc(id)}</span>${extra}${rej}${agree}${quar}<span class="gw-age">${age}</span></div>`;
    }).join('') || '<span class="muted small">none yet</span>';
  }


  // ================================================================ outside network area
  // ------------------------------------------------------------ helpers
  function voteHtml(o, long = false) {
    if (!o) return '';
    const ok = o.state === 'confirmed';
    const who = long && o.gateways && o.gateways.length ? ` <span class="muted">(${o.gateways.map(esc).join(', ')})</span>` : '';
    const label = long ? (ok ? `confirmed by ${o.votes} of ${o.of}` : `${o.votes} of ${o.of} — needs ${o.quorum ?? 2}`) : `${ok ? '✓' : '…'}${o.votes}/${o.of}`;
    return ` <span class="vote ${ok ? 'ok' : 'wait'}" title="${ok ? 'confirmed: identical signed copies from ' + o.votes + ' gateways' : 'only ' + o.votes + ' gateway(s) reported this version'}">${label}</span>${who}`;
  }
  function gwColor(id) {
    id = String(id);
    if (!state.gwOrder.includes(id)) state.gwOrder.push(id);
    const m = /(\d+)$/.exec(id);
    const i = m ? Number(m[1]) - 1 : state.gwOrder.indexOf(id);
    return GW_PALETTE[((i % GW_PALETTE.length) + GW_PALETTE.length) % GW_PALETTE.length];
  }
  // metres east/north of a point -> lat/lon (good to a few cm over a building)
  function offsetLatLng(lat, lon, e, n) {
    return L.latLng(lat + n / 111320, lon + e / (111320 * Math.cos(lat * Math.PI / 180)));
  }
  function ellipsePts(lat, lon, el, k) {
    const a = Math.max(0.05, Number(el.a_m) * k); const b = Math.max(0.05, Number(el.b_m) * k);
    const th = Number(el.angle_rad) || 0;           // major axis, from east, counter-clockwise
    const pts = [];
    for (let i = 0; i < 40; i++) {
      const t = (i / 40) * 2 * Math.PI;
      const x = a * Math.cos(t); const y = b * Math.sin(t);
      pts.push(offsetLatLng(lat, lon, x * Math.cos(th) - y * Math.sin(th), x * Math.sin(th) + y * Math.cos(th)));
    }
    return pts;
  }

  // ------------------------------------------------------------ gateway antennas
  function gwIcon(id, cls) {
    const c = gwColor(id);
    return L.divIcon({
      className: 'gw-marker', iconSize: [30, 42], iconAnchor: [15, 30],
      html: `<div class="gw-ico ${cls}" style="--c:${c}"><svg viewBox="0 0 30 30" width="30" height="30">
        <circle class="wave" cx="15" cy="9" r="8" fill="none" stroke="${c}" stroke-width="1.4"/>
        <circle class="wave w2" cx="15" cy="9" r="8" fill="none" stroke="${c}" stroke-width="1.4"/>
        <path d="M15 9 L22 29 H8 Z" fill="rgba(4,18,28,.85)" stroke="${c}" stroke-width="1.6" stroke-linejoin="round"/>
        <path d="M11.2 20 H18.8 M10 24.5 H20" stroke="${c}" stroke-width="1.2"/>
        <circle cx="15" cy="9" r="3" fill="${c}"/></svg><span class="gw-lbl">${esc(id)}</span></div>`,
    });
  }
  function upsertGatewayMarker(id, g) {
    const lat = Number(g.lat); const lon = Number(g.lon);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    const alive = g.alive != null ? g.alive : (g.status ? g.status === 'online' : true);
    const cls = g.quarantined ? 'quar' : (alive ? '' : 'off');
    let m = state.gwMarkers.get(id);
    if (!m) {
      m = L.marker([lat, lon], { icon: gwIcon(id, cls), zIndexOffset: 500, keyboard: false }).addTo(gwLayer);
      m._cls = cls;
      m.bindTooltip('', { direction: 'top', offset: [0, -30], className: 'sphere-tip' });
      state.gwMarkers.set(id, m);
      document.body.classList.add('has-ona');
    } else {
      m.setLatLng([lat, lon]);
      if (m._cls !== cls) { m.setIcon(gwIcon(id, cls)); m._cls = cls; }
    }
    const agree = g.agreement != null ? `agrees with the others ${Math.round(g.agreement * 100)} %` : '';
    m.setTooltipContent(`<b>${esc(id)}</b> · ONA gateway<br>${g.quarantined ? '<span class="warnc">IGNORED — ' + esc(g.note || 'disagreed with the majority') + '</span><br>' : ''}` +
      `${alive ? 'online' : '<span class="badc">silent</span>'}${agree ? ' · ' + agree : ''}` +
      `${g.reports != null ? `<br>${g.reports} reports · ${g.agreed} agreed · ${g.disagreed} disagreed` : ''}` +
      `${g.alt != null ? `<br>antenna ${Number(g.alt).toFixed(1)} m a.s.l.` : ''}`);
  }

  // ------------------------------------------------------------ robots (measured by the gateways)
  const writerIcon = L.divIcon({
    className: 'exec-marker writer-marker', iconSize: [30, 30], iconAnchor: [15, 15],
    html: `<div class="exec-rot"><svg viewBox="0 0 34 34" width="30" height="30">
      <circle cx="17" cy="17" r="15" fill="rgba(226,232,240,.12)" stroke="#e2e8f0" stroke-width="1.5" stroke-dasharray="3 2"/>
      <path d="M17 5 L25 26 L17 21 L9 26 Z" fill="#e2e8f0" stroke="#04121c" stroke-width="1"/></svg></div>`,
  });
  function sweepRing(center, radius, color) {
    if (document.hidden || !state.spheres || !(radius > 0)) return;
    const c = L.circle(center, { radius: 0.1, color, weight: 2.2, opacity: .95, fill: false, interactive: false }).addTo(sphereFx);
    const t0 = performance.now(); const dur = 1100;
    const step = (t) => {
      const k = Math.min(1, (t - t0) / dur); const e = 1 - Math.pow(1 - k, 3);
      c.setRadius(Math.max(0.1, radius * e)); c.setStyle({ opacity: 0.95 * (1 - k) + 0.05 });
      if (k < 1) requestAnimationFrame(step); else sphereFx.removeLayer(c);
    };
    requestAnimationFrame(step);
  }
  function robotRec(id) {
    let o = state.robots.get(id);
    if (!o) { o = { id, r: null, heardAt: 0, rings: new Map() }; state.robots.set(id, o); }
    return o;
  }
  function updateRobot(r) {
    if (!Number.isFinite(Number(r.lat)) || !Number.isFinite(Number(r.lon))) return;
    document.body.classList.add('has-ona');
    const id = String(r.robot_id);
    const o = robotRec(id); const prev = o.r;
    o.r = r; o.heardAt = Date.now();
    const role = roleOf(r.role); const isExec = String(r.role).toUpperCase() === 'EXECUTOR';
    const here = L.latLng(Number(r.lat), Number(r.lon));

    // marker: the Executor's comes from /api/executor-status; draw our own for the others
    if (isExec) {
      if (!state.execMarker || (Date.now() - state.execHeardAt) / 1000 > CFG.execStaleS) {
        updateExecutor({ lat: r.lat, lon: r.lon, heading: r.heading, status: String(r.phase || 'online').toLowerCase() });
      }
    } else {
      if (!o.marker) {
        o.trail = L.polyline([], { color: role.color, weight: 2, opacity: .45, dashArray: '2 5', interactive: false }).addTo(robotLayer);
        o.marker = L.marker(here, { icon: writerIcon, zIndexOffset: 900, keyboard: false }).addTo(map)
          .bindTooltip(`${role.label} robot`, { direction: 'top', offset: [0, -14] });
      } else o.marker.setLatLng(here);
      const el = o.marker.getElement();
      if (el && r.heading != null) el.querySelector('.exec-rot').style.transform = `rotate(${90 - Number(r.heading) * 180 / Math.PI}deg)`;
      const pts = o.trail.getLatLngs(); pts.push(here);
      if (pts.length > CFG.trailMax) pts.shift();
      o.trail.setLatLngs(pts);
    }

    // 95 % error ellipse of the gateways' measurement
    const m = r.measured;
    if (m && m.ellipse) {
      const pts = ellipsePts(Number(m.lat), Number(m.lon), m.ellipse, CFG.ellipseK);
      const st = { color: role.color, weight: 1, opacity: .8, fillColor: role.color, fillOpacity: .13, interactive: false };
      if (!o.ellipse) o.ellipse = L.polygon(pts, st).addTo(robotLayer); else o.ellipse.setLatLngs(pts);
    } else if (o.ellipse) { robotLayer.removeLayer(o.ellipse); o.ellipse = null; }

    // the robot's own estimate (SLAM -> GPS) and the gap to the measurement
    const rep = r.reported; const cons = r.consistency || {};
    if (rep && rep.lat != null && m) {
      const rl = L.latLng(Number(rep.lat), Number(rep.lon));
      const drift = cons.state === 'drift';
      const col = drift ? '#f59e0b' : role.color;
      const tip = `${esc(role.label)}'s own position (SLAM): ${cons.dist_m != null ? Number(cons.dist_m).toFixed(2) + ' m' : '?'} from the gateways' measurement${drift ? ' — <b class="warnc">DRIFT</b>, trust the gateways' : ''}`;
      if (!o.ghost) {
        o.ghost = L.circleMarker(rl, { radius: 5, color: col, weight: 1.5, dashArray: '2 2', fill: false }).addTo(robotLayer)
          .bindTooltip(tip, { className: 'sphere-tip', direction: 'top' });
        o.ghostLine = L.polyline([here, rl], { color: col, weight: 1.2, opacity: .7, dashArray: '3 4', interactive: false }).addTo(robotLayer);
      } else {
        o.ghost.setLatLng(rl).setStyle({ color: col }).setTooltipContent(tip);
        o.ghostLine.setLatLngs([here, rl]).setStyle({ color: col });
      }
    }

    // the spheres: one circle per gateway, radius = measured distance cut at robot height
    const seen = new Set();
    for (const g of r.ranges || []) {
      const gwid = String(g.gw); seen.add(gwid);
      if (g.lat == null || g.horiz_m == null) continue;
      const c = gwColor(gwid); const center = L.latLng(Number(g.lat), Number(g.lon));
      const rej = g.accepted === false;
      const tip = `<b>${esc(gwid)}</b> → ${esc(role.label)}: <b>${Number(g.range_m).toFixed(2)} m</b> ±${Number(g.sigma_m).toFixed(2)}` +
        `<br>${g.src === 'rssi' ? 'from signal strength (no time of flight)' : 'time of flight'}` +
        `${Number(g.walls) > 0.2 ? ` · ~${Number(g.walls).toFixed(1)} wall(s), corrected from ${Number(g.raw_m).toFixed(2)} m` : ''}` +
        `${g.residual_m != null ? `<br>fix residual ${Number(g.residual_m).toFixed(2)} m` : ''}${rej ? '<br><span class="warnc">rejected by the filter (outlier)</span>' : ''}`;
      const st = { color: c, weight: rej ? 1 : 1.6, opacity: rej ? .45 : .75, dashArray: rej ? '3 6' : null, fill: false };
      let ring = o.rings.get(gwid);
      if (!ring) {
        ring = L.circle(center, { radius: Number(g.horiz_m), ...st }).bindTooltip(tip, { className: 'sphere-tip', sticky: true });
        ring.addTo(sphereLayer); o.rings.set(gwid, ring);
      } else {
        ring.setLatLng(center); ring.setRadius(Number(g.horiz_m)); ring.setStyle(st); ring.setTooltipContent(tip);
      }
      if (!prev || prev.seq !== r.seq) sweepRing(center, Number(g.horiz_m), c);
    }
    for (const [gwid, ring] of o.rings) if (!seen.has(gwid)) { sphereLayer.removeLayer(ring); o.rings.delete(gwid); }

    // tag over the robot when something is wrong
    const tagTxt = r.emergency ? 'SOS' : (cons.state === 'drift' ? 'SLAM DRIFT' : null);
    if (tagTxt) {
      const icon = L.divIcon({ className: `robot-tag ${r.emergency ? 'sos' : 'drift'}`, html: tagTxt, iconSize: [70, 14], iconAnchor: [35, 30] });
      if (!o.tag) o.tag = L.marker(here, { icon, interactive: false, keyboard: false }).addTo(robotLayer);
      else { o.tag.setLatLng(here); o.tag.setIcon(icon); }
      o.tag.getElement() && o.tag.getElement().style.setProperty('--c', role.color);
    } else if (o.tag) { robotLayer.removeLayer(o.tag); o.tag = null; }

    renderRobotPanel(o);
    if (state.follow && isExec) map.panTo(here, { animate: true });
  }

  function robotDetailsHtml(r) {
    const m = r.measured; const cons = r.consistency || {}; const fix = r.fix;
    const rows = [];
    const used = (r.ranges || []).filter((g) => g.accepted !== false).length;
    if (r.source === 'gateways' && m) {
      rows.push(['Position', `<span class="okc">measured by ${r.via === 'sat' ? 'the gateways' : `${used} gateway${used === 1 ? '' : 's'}`}</span> ±<b>${Number(m.sigma_m).toFixed(2)}</b> m` +
        `${fix && fix.hdop != null ? ` <span class="muted">· HDOP ${Number(fix.hdop).toFixed(1)}${fix.raim && fix.raim !== 'n/a' ? ' · RAIM ' + esc(fix.raim) : ''}</span>` : ''}`]);
    } else {
      rows.push(['Position', `<span class="warnc">robot's own report only</span> <span class="muted">(${state.site && state.site.underground ? 'no gateway in line of sight underground' : "gateways can't fix it yet"})</span>`]);
    }
    if (cons.state === 'agree') rows.push(['Own SLAM', `<span class="okc">agrees</span> <span class="muted">(${Number(cons.dist_m).toFixed(2)} m apart)</span>`]);
    else if (cons.state === 'drift') rows.push(['Own SLAM', `<span class="warnc"><b>DRIFT</b> ${Number(cons.dist_m).toFixed(1)} m</span> <span class="muted">— map follows the gateways</span>`]);
    const parts = [];
    if (r.phase) parts.push(`<b>${esc(r.phase)}</b>`);
    if (r.battery_pct != null) parts.push(`battery ${esc(r.battery_pct)} %`);
    if (r.odo_m != null) parts.push(`${Number(r.odo_m).toFixed(0)} m driven`);
    if (r.stuck) parts.push('<span class="warnc">stuck</span>');
    if (r.emergency) parts.push('<span class="badc"><b>EMERGENCY</b></span>');
    if (parts.length) rows.push(['Status', parts.join(' · ')]);
    let prog = '';
    if (r.total) {
      const f = Math.max(0, Math.min(1, Number(r.done || 0) / Number(r.total)));
      rows.push([String(r.role).toUpperCase() === 'EXECUTOR' ? 'Mission' : 'Progress',
        `${r.mission_id ? `#${esc(r.mission_id)} ${r.mission_ack ? '<span class="okc">✓ briefed</span>' : ''} · ` : ''}<b>${esc(r.done || 0)}/${esc(r.total)}</b> ${String(r.role).toUpperCase() === 'EXECUTOR' ? 'targets treated' : 'done'}`]);
      prog = `<div class="progress"><div style="width:${Math.round(f * 100)}%"></div></div>`;
    }
    if (r.last_beacon) rows.push(['Last beacon', `#${esc(r.last_beacon)}`]);
    if (r.via === 'sat') rows.push(['Link', '<span class="warnc">satellite summary</span> <span class="muted">— full picture when LTE is back</span>']);
    return rows.map(([k, v]) => `<div class="ro-row"><span class="k">${k}</span><span class="v">${v}</span></div>`).join('') + prog;
  }
  function renderRobotPanel(o) {
    const r = o.r; const role = String(r.role).toUpperCase();
    if (role === 'EXECUTOR') {
      $('execOna').hidden = false; $('execOna').innerHTML = robotDetailsHtml(r);
    } else if (role === 'WRITER') {
      $('writerPanel').hidden = false; $('writerOna').innerHTML = robotDetailsHtml(r);
    }
  }
  function tickRobots() {
    for (const o of state.robots.values()) {
      if (!o.r) continue;
      const stale = (Date.now() - o.heardAt) / 1000 > CFG.robotStaleS;
      if (String(o.r.role).toUpperCase() === 'WRITER') {
        const chip = $('writerStatus');
        chip.textContent = stale ? `stale ${ago(Date.now() - o.heardAt)}` : String(o.r.phase || 'online').toLowerCase();
        chip.className = 'chip ' + (stale ? 'stale' : 'ok');
      }
      if (o.ellipse) o.ellipse.setStyle({ opacity: stale ? .3 : .8, fillOpacity: stale ? .04 : .13 });
      if (stale && o.rings.size) { for (const ring of o.rings.values()) sphereLayer.removeLayer(ring); o.rings.clear(); }
    }
  }

  // ------------------------------------------------------------ ONA panel
  function renderOna() {
    const s = state.ona;
    const chip = $('onaLink');
    const age = s ? (Date.now() - state.onaHeardAt) / 1000 : Infinity;
    if (!s) return;
    if (s.site) setSite(s.site);
    $('onaEmpty').hidden = true; $('onaContent').hidden = false;
    // on satellite the ONA only sends the essentials (no periodic status): the LINK_CHANGE event says so
    const link = String(state.onaLink || (s.link && s.link.state) || 'LTE').toUpperCase();
    const sat = link.startsWith('SAT');
    const silent = age > CFG.onaStaleS && !sat;
    const label = silent ? `silent ${ago(age * 1000)}` : (sat ? 'SATELLITE' : (link === 'NONE' ? 'NO UPLINK' : link));
    chip.className = 'link-chip ' + (silent || link === 'NONE' ? 'none' : (sat ? 'sat' : 'lte'));
    chip.lastElementChild.textContent = label;
    chip.title = sat ? 'LTE is down: hazards, alerts and the Executor position come over the satellite; the rest is queued' : '';

    $('onaGws').innerHTML = (s.gateways || []).map((g) => {
      const c = gwColor(g.gw);
      const ag = g.agreement != null ? g.agreement : 1;
      const cls = g.quarantined ? 'quar' : (g.alive ? '' : 'off');
      const st = g.quarantined ? 'IGNORED' : (g.alive ? `${g.reports ?? 0} rpt` : `silent ${g.last_heard_s != null ? fmtDur(g.last_heard_s) : ''}`);
      const tip = g.quarantined ? (g.note || 'disagreed with the majority') : `${g.agreed ?? 0} agreed · ${g.disagreed ?? 0} disagreed · ${g.invalid ?? 0} bad signatures · ${g.lines ?? 0} lines`;
      return `<div class="ona-gw ${cls}" style="--c:${c}" data-gw="${esc(g.gw)}" title="${esc(tip)}">
        <span class="sw"></span><span class="nm">${esc(g.gw)}</span>
        <div class="bar"><div style="width:${Math.round(ag * 100)}%"></div></div>
        <span class="pc">${Math.round(ag * 100)}%</span><span class="st">${esc(st)}</span></div>`;
    }).join('');
    for (const g of s.gateways || []) upsertGatewayMarker(g.gw, g);

    const rc = s.records || {};
    $('onaRec').innerHTML = `<b class="okc">${rc.confirmed ?? 0}</b> confirmed (${s.quorum ?? 2} of ${(s.gateways || []).length})` +
      `${rc.unconfirmed ? ` · <b class="warnc">${rc.unconfirmed}</b> waiting` : ''}${rc.conflicts ? ` · <b class="badc">${rc.conflicts}</b> conflict${rc.conflicts > 1 ? 's' : ''}` : ''}`;
    const L_ = s.link || {};
    const sent = Object.entries(L_.sent_by_link || {}).map(([k, v]) => `${esc(k)} ${v}`).join(' · ');
    $('onaQueue').innerHTML = `${L_.pending ? `<b class="warnc">${L_.pending}</b> queued${L_.oldest_pending_s ? ` (oldest ${fmtDur(L_.oldest_pending_s)})` : ''}` : '<span class="okc">queue empty</span>'}${sent ? ` <span class="muted">· sent ${sent}</span>` : ''}`;
    const cal = s.calibration || {}; const chk = cal.in_mission_check;
    $('onaCal').innerHTML = chk
      ? (chk.verdict === 'holds'
        ? `<span class="okc">holds</span> <span class="muted">· ${Number(chk.mean_shift_m).toFixed(2)} m, ${Number(chk.dyaw_deg).toFixed(1)}° over ${chk.pairs} fixes</span>`
        : `<span class="warnc"><b>DRIFTED</b> ${Number(chk.mean_shift_m).toFixed(1)} m / ${Number(chk.dyaw_deg).toFixed(1)}°</span> <span class="muted">· re-survey the entrance markers</span>`)
      : `<span class="muted">${Number(cal.rms_m) > 0 ? `entrance markers fit ±${Number(cal.rms_m).toFixed(2)} m` : 'set from the anchor + heading'} · in-mission check needs 3-gateway fixes</span>`;
    const ms = s.mission;
    if (ms) {
      const pr = ms.progress || {};
      const acked = ms.acked_by && ms.acked_by.length;
      $('onaMission').innerHTML = `#${esc(ms.mission_id)} ${(ms.targets || []).map((t) => '#' + esc(t)).join(' → ')}${ms.return_to_exit ? ' → exit' : ''}` +
        `<br>${acked ? '<span class="okc">✓ acknowledged</span>' : `<span class="warnc">sent ${esc(ms.tx_count)}×, no ack yet</span>`}` +
        `${pr.total ? ` · ${esc(pr.done ?? 0)}/${esc(pr.total)} · ${esc(pr.phase || '')}` : ''}`;
    } else $('onaMission').innerHTML = '<span class="muted">none — dispatch one from the planner</span>';
  }
  function tickOna() {
    if (!state.ona) return;
    renderOna();
  }

  // ------------------------------------------------------------ ONA events -> feed + toasts
  const ONA_ICONS = {
    GATEWAY_DISAGREES: ['⚖', '#f59e0b'], GATEWAY_QUARANTINED: ['⛔', '#ef4444'], RECORD_CONFLICT: ['⚠', '#ef4444'],
    BEACON_LOST: ['✖', '#f87171'], SPOOF_ON_AIR: ['🛡', '#ef4444'], SLAM_DRIFT: ['⌖', '#f59e0b'], SLAM_OK: ['⌖', '#22c55e'],
    MISSION_DISPATCH: ['➤', '#38bdf8'], MISSION_ACK: ['✔', '#22c55e'], ROBOT_EMERGENCY: ['🆘', '#ef4444'],
    UNKNOWN_BRIEFING: ['?', '#f59e0b'], LINK_CHANGE: ['📡', '#38bdf8'],
    RESOURCE_MARKED: ['⛏', '#d4a373'], AREA_SEARCHED: ['✓', '#86efac'],
  };
  const ONA_TITLES = {
    GATEWAY_DISAGREES: 'Gateway disagrees', GATEWAY_QUARANTINED: 'Gateway ignored', RECORD_CONFLICT: 'Record conflict',
    BEACON_LOST: 'Beacon lost', SPOOF_ON_AIR: 'Forged frames on air', SLAM_DRIFT: 'SLAM drift', SLAM_OK: 'SLAM back on track',
    MISSION_DISPATCH: 'Briefing sent', MISSION_ACK: 'Briefing acknowledged', ROBOT_EMERGENCY: 'Robot emergency',
    UNKNOWN_BRIEFING: 'Unknown briefing on air', LINK_CHANGE: 'Uplink changed',
    RESOURCE_MARKED: 'Resource marked', AREA_SEARCHED: 'Area searched',
  };
  function onaEventKey(e) { return `${e.t}|${e.event}|${e.text}`; }
  function addOnaEvent(e, live) {
    const key = onaEventKey(e);
    if (state.seenOnaEvents.has(key)) return;
    state.seenOnaEvents.add(key);
    const empty = $('feedEmpty'); if (empty) empty.remove();
    const [icon, color] = ONA_ICONS[e.event] || ['◈', '#94a3b8'];
    const li = document.createElement('li');
    li.className = `feed-item ona-ev${live ? ' new' : ''}${e.level === 'alert' ? ' warn' : (e.level === 'warn' ? ' alert' : '')}`;
    const title = esc(ONA_TITLES[e.event] || String(e.event).replace(/_/g, ' ').toLowerCase().replace(/^./, (c) => c.toUpperCase()));
    li.innerHTML = `
      <div class="fi-icon" style="--c:${color}">${icon}</div>
      <div class="fi-main">
        <div class="fi-title">${title}</div>
        <div class="fi-sub" title="${esc(e.text)}">ONA · ${esc(e.text)}</div>
      </div>
      <div class="fi-time">${hms(new Date(Math.min(Date.now(), (e.received_at || e.t || Date.now() / 1000) * 1000)))}</div>`;
    li.addEventListener('click', () => {
      if (e.beacon_id != null && state.beacons.has(String(e.beacon_id))) {
        const rec = state.beacons.get(String(e.beacon_id));
        map.flyTo(rec.marker.getLatLng(), Math.max(map.getZoom(), 19), { duration: .6 });
        setTimeout(() => openPopup(String(e.beacon_id)), 650);
      } else if (e.gw && state.gwMarkers.has(e.gw)) {
        const m = state.gwMarkers.get(e.gw); map.flyTo(m.getLatLng(), Math.max(map.getZoom(), 19), { duration: .6 });
        setTimeout(() => m.openTooltip(), 650);
      } else if (e.robot_id != null && state.robots.has(String(e.robot_id))) {
        const o = state.robots.get(String(e.robot_id)); map.flyTo([Number(o.r.lat), Number(o.r.lon)], Math.max(map.getZoom(), 19), { duration: .6 });
      }
    });
    const feed = $('feed');
    feed.prepend(li);
    while (feed.children.length > CFG.feedMax) feed.lastElementChild.remove();
    state.feedCount += 1;
    $('feedCount').textContent = `${state.feedCount} event${state.feedCount === 1 ? '' : 's'}`;
    if (live && (e.level === 'alert' || e.level === 'warn' || e.event === 'MISSION_ACK')) {
      toast(`${icon} ${e.text}`, e.level === 'alert' ? 'err' : (e.level === 'warn' ? 'warn' : 'ok'));
    }
  }
  function setSpheres(on) {
    state.spheres = on;
    if (on) { sphereLayer.addTo(map); sphereFx.addTo(map); } else { map.removeLayer(sphereLayer); map.removeLayer(sphereFx); }
    $('btnSpheres').classList.toggle('active', on);
  }


  // ================================================================ site: the Gafsa mine
  // point in polygon (rings of L.LatLng), even-odd rule
  function inRing(ll, ring) {
    let inside = false;
    for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
      const a = ring[i]; const b = ring[j];
      if (((a.lat > ll.lat) !== (b.lat > ll.lat)) &&
          (ll.lng < (b.lng - a.lng) * (ll.lat - a.lat) / ((b.lat - a.lat) || 1e-12) + a.lng)) inside = !inside;
    }
    return inside;
  }
  function zoneAt(ll) {
    for (const z of state.zones.values()) {
      if (z.rings.some((r) => inRing(ll, r))) {
        const victims = [...state.beacons.values()].filter((rec) => kindKey(rec.data) === 'victim' && isLive(rec)
          && !rec.data.retracted && z.rings.some((r) => inRing(rec.marker.getLatLng(), r)));
        return { ...z, victims };
      }
    }
    return null;
  }
  function setSite(site) {
    if (!site || (state.site && state.site.id === site.id)) return;
    state.site = site;
    document.body.classList.toggle('underground', !!site.underground);
    $('brandSub').textContent = `TSYP14 · RAS × AESS · ${site.name || site.id}`;
    $('sitePanel').hidden = false;
    if (site.underground) {        // the legend in mine words
      document.querySelectorAll('[data-legend]').forEach((el) => {
        const k = el.dataset.legend; if (MINE_LABEL[k]) el.lastChild.textContent = MINE_LABEL[k];
      });
    }
    $('siteName').textContent = site.underground ? 'Underground site' : 'Site';
    if (site.underground) {
      [dark, sat, osm, grid].forEach((l) => { if (map.hasLayer(l)) map.removeLayer(l); });
      switchedOffline = true;                         // no tiles needed underground
      layersCtl.addBaseLayer(rock, 'Mine plan (underground)');
      rock.addTo(map);
    }
    if (site.plan) {
      fetch(site.plan).then((r) => r.json()).then(drawPlan).catch(() => toast(`Mine plan ${site.plan} not found`, 'err'));
    }
    for (const rec of state.beacons.values()) styleMarker(rec);
    renderSite();
  }
  function drawPlan(g) {
    planLayer.clearLayers(); state.zones.clear();
    const toLL = (c) => L.latLng(c[1], c[0]);
    const bounds = [];
    for (const f of g.features || []) {
      const p = f.properties || {}; const geo = f.geometry || {};
      if (p.kind === 'rock' || p.kind === 'pillar') {
        const ring = geo.coordinates[0].map(toLL); bounds.push(...ring);
        L.polygon(ring, { color: p.kind === 'pillar' ? '#8a6f55' : '#5b4a3a', weight: 1, fillColor: p.kind === 'pillar' ? '#3d3127' : '#2a221b',
          fillOpacity: .92, interactive: p.kind === 'pillar' })
          .bindTooltip(p.kind === 'pillar' ? `Pillar ${esc(String(p.name).replace('pillar_', 'P'))}` : '', { className: 'sphere-tip', sticky: true })
          .addTo(planLayer);
      } else if (p.kind === 'seam') {
        L.polyline(geo.coordinates.map(toLL), { color: '#d4a373', weight: 4, opacity: .55, dashArray: '2 5' })
          .bindTooltip(`${esc(p.name)} on the old plan (layer ${esc(p.layer)}) — the robot measures the grade`, { className: 'sphere-tip', sticky: true })
          .addTo(planLayer);
      } else if (p.kind === 'zone') {
        const rings = geo.coordinates.map((poly) => poly[0].map(toLL));
        const layer = L.polygon(rings, { color: '#64748b', weight: 1, dashArray: '3 5', fillColor: '#64748b', fillOpacity: .03 })
          .bindTooltip('', { className: 'sphere-tip', sticky: true }).addTo(planLayer);
        layer.on('click', () => map.fitBounds(layer.getBounds().pad(0.3), { maxZoom: 21 }));
        state.zones.set(p.id, { id: p.id, name: p.name, layer, rings, status: 'unknown' });
      } else if (p.kind === 'portal') {
        L.marker(toLL(geo.coordinates), { interactive: false, keyboard: false,
          icon: L.divIcon({ className: 'portal-tag', html: '⛏ PORTAL', iconSize: [64, 16], iconAnchor: [32, 30] }) }).addTo(planLayer);
      }
    }
    if (bounds.length) map.fitBounds(L.latLngBounds(bounds).pad(0.04), { maxZoom: 22 });   // the plan is the map
    state.centered = true;
    renderSite();
  }
  let siteQueued = false;
  function queueSite() {
    if (siteQueued) return;
    siteQueued = true;
    setTimeout(() => { siteQueued = false; renderSite(); }, 250);
  }
  function renderSite() {
    if (!state.site) return;
    // victim search, area by area
    const live = [...state.beacons.values()].filter((rec) => isLive(rec) && !rec.data.retracted);
    let searched = 0;
    const zrows = [];
    for (const z of state.zones.values()) {
      const inZ = (rec) => z.rings.some((r) => inRing(rec.marker.getLatLng(), r));
      const s = live.filter((rec) => kindKey(rec.data) === 'searched' && inZ(rec));
      const v = live.filter((rec) => kindKey(rec.data) === 'victim' && inZ(rec));
      const status = v.length ? 'victim' : (s.length ? 'searched' : 'unknown');
      if (s.length) searched += 1;
      const col = { victim: '#ec4899', searched: '#22c55e', unknown: '#64748b' }[status];
      z.layer.setStyle({ color: col, fillColor: col, fillOpacity: status === 'unknown' ? (state.showSearch ? .03 : 0) : (state.showSearch ? .13 : 0),
        dashArray: status === 'unknown' ? '3 5' : null, opacity: state.showSearch ? .8 : 0, weight: status === 'unknown' ? 1 : 1.5 });
      const cov = s.length ? Math.max(...s.map((rec) => Number(rec.data.severity) || 0)) : null;
      z.layer.setTooltipContent(`<b>${esc(z.name)}</b><br>` + (status === 'unknown' ? 'not searched yet'
        : `${status === 'victim' ? `<span class="badc">${v.length} victim${v.length > 1 ? 's' : ''} found</span>` : '<span class="okc">searched — nobody found</span>'}` +
          `${cov != null ? ` · ${cov} % seen` : ' · search not finished'}`));
      z.status = status;
      zrows.push(`<div class="zone ${status}" data-zone="${esc(z.id)}" title="${esc(z.name)}">
        <i>${status === 'victim' ? '✚' : (status === 'searched' ? '✓' : '·')}</i><span>${esc(z.name)}</span>
        <b>${status === 'victim' ? `${v.length} victim${v.length > 1 ? 's' : ''}` : (status === 'searched' ? `${cov} %` : '—')}</b></div>`);
    }
    const nz = state.zones.size;
    $('searchCount').textContent = nz ? `${searched}/${nz} areas` : '';
    $('searchBar').style.width = nz ? `${Math.round(100 * searched / nz)}%` : '0%';
    $('zoneList').innerHTML = zrows.join('') || '<span class="muted small">no search areas on this plan</span>';
    // hazards with their measured values, resources with their grades
    const row = (rec) => {
      const b = rec.data; const t = typeOf(kindKey(b));
      return `<div class="site-row" data-id="${esc(rec.id)}"><i style="--c:${t.color}">${t.icon}</i>
        <span class="nm">${esc(t.label)} <span class="muted">#${esc(b.beacon_id)}</span></span>
        <span class="val">${esc(mineMeasure(b) || '')}</span></div>`;
    };
    const conf = (rec) => !(rec.data.ona && rec.data.ona.state !== 'confirmed');
    const haz = live.filter((rec) => conf(rec) && (typeOf(kindKey(rec.data)).hazard || kindKey(rec.data) === 'victim') && !rec.data.stale);
    const order = { victim: 0, gas: 1, radiation: 2, thermal: 3, fire: 3, structural: 4, obstruction: 5 };
    haz.sort((a, b) => (order[kindKey(a.data)] ?? 9) - (order[kindKey(b.data)] ?? 9));
    $('hazList').innerHTML = haz.map(row).join('') || '<span class="muted small">none reported yet</span>';
    const res = live.filter((rec) => conf(rec) && typeOf(kindKey(rec.data)).resource);
    res.sort((a, b) => (kindKey(a.data) === 'phosphate' ? -Number(a.data.severity) : 1) - (kindKey(b.data) === 'phosphate' ? -Number(b.data.severity) : 1));
    $('resList').innerHTML = res.map(row).join('') || '<span class="muted small">none marked yet</span>';
  }
  function setLayerShown(layer, on, btn) {
    if (on) layer.addTo(map); else map.removeLayer(layer);
    $(btn).classList.toggle('active', on);
  }

  // ------------------------------------------------------------ mission planner
  function toggleWaypoint(id) {
    const i = state.selected.indexOf(id);
    if (i >= 0) state.selected.splice(i, 1);
    else {
      if (state.selected.length >= 10) { toast('Max 10 waypoints per mission', 'err'); return; }
      state.selected.push(id);
    }
    renderPlanner();
    if (openPopupId === id) openPopup(id);
  }

  // Executor path = the target's way-out chain, reversed (entrance -> target),
  // thinned to at most 10 waypoints (always keeps the first and the target).
  function routeIn(id) {
    const ids = routeOut(id).ids.slice().reverse();
    let pick = ids;
    if (ids.length > 10) {
      pick = [];
      for (let i = 0; i < 10; i++) pick.push(ids[Math.round(i * (ids.length - 1) / 9)]);
    }
    state.selected = [...new Set(pick)];
    renderPlanner(); setPlannerCollapsed(false);
    toast(`Executor path: ${state.selected.map((x) => '#' + x).join(' → ')} (review, then dispatch)`, 'ok');
    if (openPopupId === id) openPopup(id);
  }

  function refreshRoute() {
    const pts = state.selected
      .map((id) => state.beacons.get(id)).filter(Boolean)
      .map((rec) => rec.marker.getLatLng());
    if (pts.length && state.executor) pts.unshift(L.latLng(Number(state.executor.lat), Number(state.executor.lon)));
    routeLine.setLatLngs(pts.length > 1 ? pts : []);
  }

  function renderPlanner() {
    const list = $('wpList');
    for (const rec of state.beacons.values()) {
      styleMarker(rec);
      rec.marker.unbindTooltip();
    }
    state.selected.forEach((id, i) => {
      const rec = state.beacons.get(id);
      if (rec) rec.marker.bindTooltip(`WP${i + 1}`, { permanent: true, direction: 'top', className: 'wp-tip', offset: [0, -10] }).openTooltip();
    });

    list.innerHTML = state.selected.map((id, i) => {
      const rec = state.beacons.get(id); if (!rec) return '';
      const b = rec.data; const t = typeOf(kindKey(b));
      return `<li class="wp" draggable="true" data-idx="${i}">
        <span class="wp-grip">⋮⋮</span>
        <span class="wp-num">${i + 1}</span>
        <div class="wp-info">
          <div class="wp-title"><span style="color:${t.color}">${t.icon}</span> ${esc(t.label)} · #${esc(b.beacon_id)}${b.silent ? ' <span class="bad">lost?</span>' : ''}</div>
          <div class="wp-sub">${Number(b.lat).toFixed(5)}, ${Number(b.lon).toFixed(5)}${b.err_m != null ? ` ±${Number(b.err_m).toFixed(1)} m` : ''}</div>
        </div>
        <div class="wp-btns">
          <button class="icon-btn" data-action="wp-up" data-idx="${i}" ${i === 0 ? 'disabled' : ''} title="Move up">▲</button>
          <button class="icon-btn" data-action="wp-down" data-idx="${i}" ${i === state.selected.length - 1 ? 'disabled' : ''} title="Move down">▼</button>
          <button class="icon-btn" data-action="wp-del" data-idx="${i}" title="Remove">✕</button>
        </div></li>`;
    }).join('');

    const n = state.selected.length;
    $('wpEmpty').style.display = n ? 'none' : '';
    $('wpCount').textContent = `${n} waypoint${n === 1 ? '' : 's'}`;
    $('btnDispatch').disabled = n === 0;
    $('btnClear').disabled = n === 0;
    refreshRoute();
  }

  function moveWp(from, to) {
    if (to < 0 || to >= state.selected.length || from === to) return;
    const [x] = state.selected.splice(from, 1);
    state.selected.splice(to, 0, x);
    renderPlanner();
  }

  // drag & drop reorder
  let dragIdx = null;
  $('wpList').addEventListener('dragstart', (e) => {
    const li = e.target.closest('.wp'); if (!li) return;
    dragIdx = Number(li.dataset.idx); li.classList.add('dragging');
    e.dataTransfer.effectAllowed = 'move';
    e.dataTransfer.setData('text/plain', String(dragIdx));
  });
  $('wpList').addEventListener('dragover', (e) => {
    const li = e.target.closest('.wp'); if (!li) return;
    e.preventDefault();
    document.querySelectorAll('.wp.drop-target').forEach((x) => x.classList.remove('drop-target'));
    li.classList.add('drop-target');
  });
  $('wpList').addEventListener('drop', (e) => {
    const li = e.target.closest('.wp'); if (!li || dragIdx === null) return;
    e.preventDefault(); moveWp(dragIdx, Number(li.dataset.idx)); dragIdx = null;
  });
  $('wpList').addEventListener('dragend', () => {
    dragIdx = null;
    document.querySelectorAll('.wp').forEach((x) => x.classList.remove('dragging', 'drop-target'));
  });

  function setPlanMode(on) {
    state.planMode = on;
    $('btnPlan').classList.toggle('active', on);
    $('btnPlan').textContent = on ? '✓ Done selecting' : '＋ Select on map';
    $('planBanner').classList.toggle('show', on);
    document.querySelector('.map-wrap').classList.toggle('planning', on);
    if (on) { map.closePopup(); setPlannerCollapsed(false); }
  }

  function setPlannerCollapsed(c) {
    $('planner').classList.toggle('collapsed', c);
    $('plannerToggle').setAttribute('aria-expanded', String(!c));
  }

  async function dispatchMission() {
    const waypoints = state.selected.map((id) => state.beacons.get(id)).filter(Boolean)
      .map((rec) => {
        const w = { beacon_id: rec.data.beacon_id, lat: Number(rec.data.lat), lon: Number(rec.data.lon) };
        if (rec.data.v === 2) { w.kind = rec.data.kind; if (rec.data.err_m != null) w.err_m = Number(rec.data.err_m); }
        return w;
      });
    if (!waypoints.length) return;
    const btn = $('btnDispatch');
    btn.disabled = true; btn.firstElementChild.textContent = 'Dispatching…';
    lastSelfDispatch = Date.now();   // set BEFORE the POST: the socket echo can beat the response
    try {
      const r = await fetch('/api/mission-dispatch', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ waypoints, dispatched_at: Date.now() / 1000, return_to_exit: $('chkReturn').checked }),
      });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      toast(`Mission dispatched ✅ — ${waypoints.length} waypoint${waypoints.length > 1 ? 's' : ''} sent to Executor`, 'ok');
      $('lastMission').textContent = `Last dispatch ${hms(new Date())}: ${waypoints.map((w) => '#' + w.beacon_id).join(' → ')}`;
      setPlanMode(false);
    } catch (err) {
      toast(`Dispatch failed: ${err.message}`, 'err');
    } finally {
      btn.firstElementChild.textContent = 'Dispatch mission';
      btn.disabled = state.selected.length === 0;
    }
  }
  let lastSelfDispatch = 0;

  // an ABORT briefing (the ONA signs it, the gateways transmit it, the Executor goes back to the exit)
  async function abortMission() {
    if (!confirm('Call the Executor back to the exit now? It stops the current task.')) return;
    lastSelfDispatch = Date.now();
    try {
      const r = await fetch('/api/mission-dispatch', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ waypoints: [], abort: true, dispatched_at: Date.now() / 1000 }),
      });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      toast('Call-back sent — the ONA signs it and the gateways transmit it', 'warn');
      $('lastMission').textContent = `Call-back ${hms(new Date())}`;
    } catch (err) {
      toast(`Call-back failed: ${err.message}`, 'err');
    }
  }

  // ------------------------------------------------------------ UI wiring
  document.addEventListener('click', (e) => {
    const el = e.target.closest('[data-action]'); if (!el) return;
    const act = el.dataset.action; const idx = Number(el.dataset.idx);
    if (act === 'toggle-wp') toggleWaypoint(el.dataset.id);
    else if (act === 'route-in') routeIn(el.dataset.id);
    else if (act === 'wp-up') moveWp(idx, idx - 1);
    else if (act === 'wp-down') moveWp(idx, idx + 1);
    else if (act === 'wp-del') { state.selected.splice(idx, 1); renderPlanner(); }
  });
  $('btnPlan').addEventListener('click', () => setPlanMode(!state.planMode));
  $('btnClear').addEventListener('click', () => { state.selected = []; renderPlanner(); });
  $('btnDispatch').addEventListener('click', dispatchMission);
  $('btnAbort').addEventListener('click', abortMission);
  $('plannerToggle').addEventListener('click', () => setPlannerCollapsed(!$('planner').classList.contains('collapsed')));
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && state.planMode) setPlanMode(false); });

  $('btnFit').addEventListener('click', () => {
    const pts = [...state.beacons.values()].map((r) => r.marker.getLatLng());
    if (state.executor) pts.push(L.latLng(Number(state.executor.lat), Number(state.executor.lon)));
    for (const m of state.gwMarkers.values()) pts.push(m.getLatLng());
    for (const o of state.robots.values()) if (o.r) pts.push(L.latLng(Number(o.r.lat), Number(o.r.lon)));
    for (const z of state.zones.values()) pts.push(z.layer.getBounds().getNorthWest(), z.layer.getBounds().getSouthEast());
    if (pts.length) map.fitBounds(L.latLngBounds(pts).pad(state.zones.size ? 0.05 : 0.25), { maxZoom: 21 });
    else map.setView(CFG.center, CFG.zoom);
  });
  $('btnFollow').addEventListener('click', () => {
    state.follow = !state.follow;
    $('btnFollow').classList.toggle('active', state.follow);
    if (state.follow && state.executor) map.panTo([Number(state.executor.lat), Number(state.executor.lon)]);
  });
  $('btnSpheres').addEventListener('click', () => setSpheres(!state.spheres));
  $('onaToggle').addEventListener('click', () => {
    const c = !$('onaPanel').classList.contains('collapsed');
    $('onaPanel').classList.toggle('collapsed', c); $('onaToggle').setAttribute('aria-expanded', String(!c));
  });
  $('onaGws').addEventListener('click', (e) => {
    const row = e.target.closest('[data-gw]'); if (!row) return;
    const m = state.gwMarkers.get(row.dataset.gw); if (!m) return;
    map.flyTo(m.getLatLng(), Math.max(map.getZoom(), 19), { duration: .6 }); setTimeout(() => m.openTooltip(), 650);
  });
  $('btnRes').addEventListener('click', () => { state.showRes = !state.showRes; setLayerShown(resLayer, state.showRes, 'btnRes'); });
  $('btnSearch').addEventListener('click', () => {
    state.showSearch = !state.showSearch; setLayerShown(searchLayer, state.showSearch, 'btnSearch'); renderSite();
  });
  document.addEventListener('click', (e) => {
    const z = e.target.closest('[data-zone]');
    if (z && state.zones.has(z.dataset.zone)) map.fitBounds(state.zones.get(z.dataset.zone).layer.getBounds().pad(0.3), { maxZoom: 21 });
    const r = e.target.closest('.site-row[data-id]');
    if (r && state.beacons.has(r.dataset.id)) {
      map.flyTo(state.beacons.get(r.dataset.id).marker.getLatLng(), Math.max(map.getZoom(), 20), { duration: .6 });
      setTimeout(() => openPopup(r.dataset.id), 650);
    }
  });
  $('btnLinks').addEventListener('click', () => {
    const on = !map.hasLayer(linkLayer);
    if (on) linkLayer.addTo(map); else map.removeLayer(linkLayer);
    $('btnLinks').classList.toggle('active', on);
  });

  // ------------------------------------------------------------ toasts
  function toast(msg, kind = '') {
    const t = document.createElement('div');
    t.className = `toast ${kind}`; t.textContent = msg;
    $('toasts').appendChild(t);
    while ($('toasts').children.length > 4) $('toasts').firstElementChild.remove();
    setTimeout(() => { t.classList.add('out'); setTimeout(() => t.remove(), 320); }, 4200);
  }

  // ------------------------------------------------------------ clock / timers
  function tickClock() {
    $('clock').textContent = hms(new Date());
    const s = Math.floor((Date.now() - state.startedAt) / 1000);
    $('uptime').textContent = `session ${pad(Math.floor(s / 60))}:${pad(s % 60)}`;
  }
  setInterval(() => {
    tickClock();
    for (const rec of state.beacons.values()) styleMarker(rec);   // confidence / TTL fade
    updateStats();
    renderGateways();
    tickExecutor();
    tickRobots();
    tickOna();
    if (state.site) renderSite();
    if (openPopupId) {   // keep an open popup live without stealing focus
      const p = map._popup; if (p && p.isOpen()) p.setContent(popupHtml(openPopupId));
    }
  }, 1000);
  tickClock();

  // ------------------------------------------------------------ data: load + live
  async function hydrate() {
    try {
      const r = await fetch('/api/state');
      const s = await r.json();
      (s.expectedGateways || []).forEach((id) => { if (!state.gateways.has(id)) state.gateways.set(id, {}); });
      Object.values(s.health || {}).forEach((h) => {
        const g = state.gateways.get(h.gateway_id) || {};
        g.status = h.status || 'online'; g.info = h;
        g.lastSeen = Date.now() - Math.max(0, (s.serverTime - h.received_at) * 1000);
        state.gateways.set(h.gateway_id, g);
      });
      const clockSkew = Date.now() - s.serverTime * 1000;
      const fix = (b) => ({ ...b, received_at: b.received_at + clockSkew / 1000 });
      (s.beacons || []).forEach((b) => upsertBeacon(fix(b)));
      // backfill the feed (oldest first so newest ends on top)
      const $feed = $('feed');
      if (!$feed.querySelector('.feed-item')) {
        (s.events || s.beacons || []).slice(-CFG.feedMax)
          .filter((b) => b.ev !== 'dump' && b.ev !== 'own').forEach((b) => addFeedRow(fix(b), false));
      }
      if (s.site) setSite(s.site);
      if (s.executor) updateExecutor(s.executor);
      // outside network area
      if (s.ona) { state.ona = s.ona; state.onaHeardAt = Date.now() - Math.max(0, (s.serverTime - s.ona.received_at) * 1000); renderOna(); }
      Object.values(s.robots || {}).forEach((r) => {
        updateRobot(r);
        const o = state.robots.get(String(r.robot_id));
        if (o) o.heardAt = Date.now() - Math.max(0, (s.serverTime - r.received_at) * 1000);
      });
      (s.onaEvents || []).forEach((e) => { addOnaEvent(e, false); if (e.event === 'LINK_CHANGE' && e.link) state.onaLink = e.link; });
      if (s.ona && s.ona.link && s.onaEvents && !(s.onaEvents.some((e) => e.event === 'LINK_CHANGE' && e.received_at > s.ona.received_at))) state.onaLink = s.ona.link.state;
      if (s.latestMission) {
        $('lastMission').textContent = s.latestMission.abort ? `Call-back ${hms(tsDate(s.latestMission.dispatched_at))}` :
          `Last dispatch ${hms(tsDate(s.latestMission.dispatched_at))}: ` +
          s.latestMission.waypoints.map((w) => '#' + w.beacon_id).join(' → ');
      }
      if (!state.centered && state.beacons.size) {
        state.centered = true;
        const pts = [...state.beacons.values()].map((x) => x.marker.getLatLng());
        for (const m of state.gwMarkers.values()) pts.push(m.getLatLng());
        map.fitBounds(L.latLngBounds(pts).pad(0.2), { maxZoom: 20 });
      }
    } catch (err) {
      try {   // fall back to the minimal contract endpoint
        const r = await fetch('/api/beacons');
        (await r.json()).forEach((b) => upsertBeacon(b));
      } catch (_) { /* server not up yet - socket will retry */ }
    }
    renderGateways(); updateStats(); renderPlanner();
  }

  const socket = io({ reconnectionDelayMax: 3000 });
  const badge = $('linkBadge');
  let everConnected = false;
  socket.on('connect', () => {
    badge.className = 'link-badge live'; badge.lastElementChild.textContent = 'LIVE';
    if (everConnected) { toast('Reconnected to command post server', 'ok'); hydrate(); }
    everConnected = true;
  });
  socket.on('disconnect', () => {
    badge.className = 'link-badge down'; badge.lastElementChild.textContent = 'OFFLINE';
  });
  socket.on('beacon:new', (b) => { upsertBeacon(b, { live: true }); updateStats(); });
  socket.on('executor:update', (e) => updateExecutor(e));
  socket.on('network:health', (h) => noteGateway(h));
  socket.on('robot:update', (r) => updateRobot(r));
  socket.on('ona:status', (o) => {
    state.ona = o; state.onaHeardAt = Date.now();
    if (o.link && o.link.state) state.onaLink = o.link.state;
    renderOna();
  });
  socket.on('ona:event', (e) => {
    if (e.event === 'LINK_CHANGE' && e.link) {
      state.onaLink = e.link;
      if (!state.ona) state.ona = { link: { state: e.link }, gateways: [] };
      state.onaHeardAt = Date.now(); renderOna();
    }
    addOnaEvent(e, true);
  });
  socket.on('mission:dispatched', (m) => {
    if (Date.now() - lastSelfDispatch < 2000) return;   // it was us
    toast(m.abort ? 'Executor called back from another console' : `Mission dispatched from another console (${m.waypoints.length} waypoints)`);
  });

  hydrate();
})();
