# Living Map — Command Post Dashboard

Live operator dashboard for the TSYP14 Living Map (Writer + Executor robots).
Node.js + Express + Socket.io backend, Leaflet map frontend, built exactly to the
data contract in the brief.

## What's on screen

| Area | What it does |
|---|---|
| **Live map** | One pin per beacon, colored by kind: victim, gas, radiation, thermal, collapse risk, obstruction, exit, trail. Pins **fade with each record's confidence half-life** (LMB2 records) or TTL (original contract). Dashed arrows = each beacon's **next hop** (the way out). Click a pin for severity, confidence now, half-life, age/TTL, position ± error, next hop, the **full route to the exit** (highlighted in green), hops/RSSI. Lost beacons get a red dashed ring and a **LOST?** tag; aged-out events stay as hollow route markers. Cyan arrow = Executor's live position and heading, with its track. |
| **Live event feed** | Newest event on top, last 50 kept. Click a row to fly to that pin. |
| **Network health** (top bar) | One badge per gateway. Green if heard in the last 10 s, red otherwise. Shows records held and a ⚠ counter of frames rejected by HMAC (spoofing attempts raise a toast). |
| **Mission planner** | Click **Select on map**, click 2–3 pins, reorder by drag-and-drop or the ▲▼ buttons, then **Dispatch mission**. Or open a pin and press **Executor path to here**: the beacon chain from the entrance to that beacon becomes the waypoints. Draws the planned route on the map. **Back to the exit afterwards** (untick: the Executor waits at its last target), and **Call the Executor back** (a signed ABORT). |
| **Outside network area** | The ONA's three gateways as antennas on the map (amber, teal, indigo), each with its agreement bar; a gateway that disagrees with the majority is marked IGNORED. The uplink (LTE / SATELLITE / NO UPLINK), the queue, the records confirmed by the 2-of-3 vote, the in-mission check of the map-to-GPS translation, and the briefing (sent, acknowledged, progress). |
| **Robots measured by the gateways** | Each gateway's measured distance to a robot is drawn as a circle around it (the GPS-like "spheres", cut at robot height); a ring sweeps out at every new ping. The robot sits where the circles cross, inside its 95 % error ellipse; a small dashed circle is where the robot itself *thinks* it is (its SLAM), with a line to it. **Spheres** toggles them. The Executor and the Writer each get a panel: measured by N gateways ± error, HDOP, own SLAM agrees / DRIFT, phase, battery, mission progress. |
| **Votes** | Every LMB2 record from the ONA carries its vote: `✓3/3`, `✓2/3`, or `…1/3` for a version only one gateway reported. Unconfirmed pins are dotted and faded and are not counted in the tiles. A newer version heard by one gateway never replaces a confirmed one: it is kept next to it ("Unconfirmed" in the popup) until a second gateway agrees. |

It also has stats tiles (beacons, victims, active hazards, lost beacons), a LIVE/OFFLINE link badge, Fit and Follow-Executor buttons, and base-map
switching (Dark / Satellite / Street / Offline grid). If there's no internet at the venue, it
**switches to an offline grid automatically**, so pins still work.

## Run it

Needs Node.js 16+ (`sudo apt install nodejs npm` on Ubuntu).

```bash
cd command_post
npm install          # one time
npm run demo         # server + scripted LMB2 mission (fake data) -> open http://localhost:3000
```

For real data: `npm start` (no fake data). The terminal prints the LAN address, e.g.
`http://192.168.1.20:3000`. The robot side posts to that address.

Run the mock feeder separately (e.g. against another machine):
`TARGET=http://192.168.1.20:3000 node mock_feeder.js`

Change the port or the expected gateways (default gw1,gw2,gw3):
`PORT=4000 GATEWAYS=gw1,gw2 npm start`

`npm run demo` includes the Outside Network Area story: three gateways vote on every
record, the Writer and the Executor are positioned by the gateways' distance circles, gw3
starts lying at minute 18 and gets ignored, a forged victim heard by one gateway stays
unconfirmed, the Executor's SLAM slips at 20 min (drift alert, back at 24), LTE fails at
30 min (satellite) and comes back at 36. Dispatch a mission and the ONA briefs the Executor.
`ONA=0 npm run demo` = one gateway, no ONA. The real thing: `../ona` (`python -m ona`).

**The Gafsa mine.** `npm run demo:mine` replays the mine mission: the map becomes the mine plan (galleries,
pillars, the portal, search areas, gateways on the cable backbone), pins use mine words with the measured value
decoded from the severity (CO ppm, radon Bq/m³, fire °C, roof sag mm, % P₂O₅), and the **Gafsa mine** panel shows
the victim search area by area (green when searched, red with a victim), the dangers and the resources.
**Resources** and **Search** toggle those layers. The site comes from the ONA status (`site: gafsa_mine`); without
the ONA, `SITE=gafsa_mine npm start`. The plan is `public/sites/gafsa_mine.geojson`, written by
`writer_robot/tools/make_mine.py`.

## API (the contract)

| Method | Path | Effect |
|---|---|---|
| POST | `/api/beacon` | store (latest record per beacon, 200 beacons) + push `beacon:new` |
| POST | `/api/executor-status` | push `executor:update` |
| POST | `/api/network-health` | push `network:health` |
| GET | `/api/beacons` | last 200 beacons |
| POST | `/api/mission-dispatch` | store as latest mission, 200 OK |
| GET | `/api/mission-dispatch/latest` | the last dispatched mission (for the robot to fetch) |
| GET | `/api/state` | everything at once (the dashboard uses it on load/reconnect) |
| POST | `/api/robot-status` | from the ONA: a robot's position measured by the gateways (`measured` with its error ellipse, `ranges` per gateway, `reported` = its own pose, `consistency`, phase, progress) → `robot:update` |
| POST | `/api/ona-status` | uplink, queue, gateways with their agreement, votes, calibration check, briefing → `ona:status` |
| POST | `/api/ona-event` | alerts and decisions (GATEWAY_DISAGREES, GATEWAY_QUARANTINED, RECORD_CONFLICT, BEACON_LOST, SPOOF_ON_AIR, SLAM_DRIFT, SLAM_OK, MISSION_DISPATCH, MISSION_ACK, ROBOT_EMERGENCY, UNKNOWN_BRIEFING, LINK_CHANGE) → `ona:event`, feed + toast |

`/api/mission-dispatch` also takes `return_to_exit: false`, and `{"waypoints": [], "abort": true}`.
A record with `via: "sat"` (compacted for the satellite link) gets its missing contract fields
rebuilt by the server.

A beacon re-sent with the same `beacon_id` updates the existing pin (e.g. a lower TTL) instead of
adding a duplicate.

## Live data from the beacon network (LMB2)

The LoRa gateway (or the network simulator) feeds the dashboard through
`beacon_net/python/beaconnet/gateway_bridge.py`. It posts the contract fields plus the full record
(`v: 2`, `kind`, `seq`, `severity`, `eff_conf`, `half_life_s`, `ttl_s`, `err_m`, `next`, `hops`,
`silent`, `stale`, …). See `beacon_net/ARCHITECTURE.md` §10.

```bash
# simulated network, 20x real time
cd beacon_net/host && ./bpsim --speed 20 | python3 ../python/beaconnet/gateway_bridge.py --stdin --server http://localhost:3000
# real gateway on USB
python3 beacon_net/python/beaconnet/gateway_bridge.py --serial /dev/ttyUSB0 --key <mission key> --server http://localhost:3000
```

From the Writer in simulation, `beacon_net/ros/beacon_record_node.py` builds the same records from
`/writer/beacon_dropped` and can post them directly (`-p dashboard:=http://<ip>:3000`).

## Connecting the real Writer robot (original contract)

`ros_bridge.py` subscribes to `/writer/beacon_dropped` and POSTs each beacon to the dashboard in
contract format. It converts map x/y to lat/lon around the anchor and renames `fire` to `thermal`.
It also sends a `gw1` heartbeat.

```bash
# in a sourced ROS 2 terminal, with run_explorer.sh running
python3 ros_bridge.py --ros-args -p server:=http://<dashboard-ip>:3000
```

Parameters: `anchor_lat`, `anchor_lon` (default Tunis), `map_yaw_deg` (the compass direction the
robot faced at start; 0 = east), `gateway_id`, and `send_trail` (false = hazards only). When the
vision node provides the hazard's own location, the bridge includes it as `event_lat`/`event_lon`.
