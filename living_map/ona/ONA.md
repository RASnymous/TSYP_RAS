# The Outside Network Area (ONA)

TSYP14 "The Living Map" · IEEE RAS × AESS. This is the part of the system between the zone and the Command Post.

The brief asks for an ONA that connects the disconnected zone to the outside world. It must:

- translate robot coordinates into GPS;
- forward the data to a distant Command Post over wireless or satellite links;
- brief the Executor before entry.

All communication between inside and outside must pass through the ONA.

This ONA does all of that and adds three things:

- **three gateways that vote**: a message only counts when two of them heard exactly the same signed bytes;
- **positioning by three spheres**: the gateways measure the robot's distance and locate it, the way GPS satellites do;
- **a physical cross-check**: what a robot *says* about its position is compared with where the gateways *measure* it.

```
                 THE ZONE (no GPS, no network)                         OUTSIDE
 ┌───────────────────────────────────────────────┐
 │  beacons (LMB2 gossip)       robots            │     gw1 ─┐
 │  B0 ─ B1 ─ B2 ─ ... ─ B14    Writer, Executor  │ LoRa     │ USB / UDP     ┌───────────────────────────┐   LTE    ┌──────────────┐
 │  records, 44 B signed        ROBOT pings 44 B  │ ───────► gw2 ─┼──────────────► │ ONA                       │ ───────► │ COMMAND POST │
 │                              ◄── MISSION 44 B  │ ◄─────── │                │  1 re-check signatures    │ satellite│  dashboard   │
 └───────────────────────────────────────────────┘     gw3 ─┘                │  2 vote 2-of-3            │ (backup) │  live map    │
                                                                             │  3 spheres + EKF + frame  │ ◄─────── │  mission     │
                                                         ranging (ToF) ◄───► │  4 SQLite queue           │ dispatch │  planner     │
                                                                             │  5 briefing downlink      │          └──────────────┘
                                                                             └───────────────────────────┘
```

Rules the design keeps:

- **The ONA is the only place with both a LoRa side and a network side.** The robots only have LoRa, and the Command Post has no LoRa radio.
- **The ONA never trusts one source.** It re-checks every signature itself, votes across its gateways, and compares a robot's claim with a physical measurement.

---

## 1. What came from the GALERIA code, and what changed

The GALERIA team wrote the first ONA (`ona.py`, `ona3.py`, `ona4.py`), the frame translation (`trans.py`) and the Kalman filter test (`ekf.py`). The originals are in `galeria_original/`, unchanged. Their ideas are all in the ONA:

| original | where now | kept | changed, and why |
|---|---|---|---|
| `trans.py`: WGS84 ⇄ ECEF ⇄ ENU, Umeyama, `LocalToGpsCalibrator` | `ona/geodesy.py` | every function, the same maths, the same results. The original example still gives err_m 1.22 m, and a test checks it. | **Planar mode.** A 2D SLAM map is level, so only its yaw against north is unknown. A full 3D fit on GPS anchors lets GPS altitude noise (metres) tilt the whole frame (0.8 m of tilt at 47 m in the test); planar mode cannot tilt. **Other additions:** inverse conversions (GPS → map), refusal of collinear anchors, per-anchor weights, and 7 decimals (1 cm, the LMB2 resolution) instead of 6 (11 cm). |
| `ekf.py`: `PositionEKF` (predict with odometry, correct with a position) | `ona/kalman.py` | `PositionEKF`, unchanged | **An independent measurement.** The original test corrected the prediction with the Umeyama position of the same point. That is the same information as the prediction, so the smaller error bar (1.22 → 0.70 m) was not earned. Inside the building there is no GPS, but the gateway ranges are an independent measurement. `RangeEKF` makes one correction per range (a true *extended* Kalman filter, since distance is nonlinear). |
| `ona.py`: frame + CRC + HMAC, dedup, SQLite queue, LTE → satellite, MQTT, audit, mission downlink | `ona/store.py`, `ona/uplink.py`, `ona/core.py` | the SQLite queue (same table; your old file opens and is upgraded in place), LTE first with satellite as backup, the audit log, the downlink to the Executor, and the rule that only the ONA has both radios | **The radio format is the team's LMB2 frame.** It is 44 bytes, already signed with HMAC, and carries GPS, the way out (`next`), a half-life and 8 kinds. A CRC16 is not needed: LoRa checks every packet with a CRC, and the HMAC catches any change. **Dedup** is by (beacon, version) and the vote, instead of beacon id + timestamp within 1 s. **Queue additions:** priorities and coalescing. **Briefing** is a signed 44-byte MISSION frame instead of unbounded JSON. |

The local-to-GPS translation *at the ONA*, which was your friend's design, is kept exactly where it matters most: robot position reports. Robots send private map coordinates and the ONA translates them. Beacon records carry GPS, as in the brief's example message.

---

## 2. The 2-of-3 vote (`quorum.py`)

Each gateway passes on every frame it hears as one JSON line, including the frame bytes. The ONA:

1. **re-checks the signature** with its own copy of the mission key. A gateway that passes on frames failing this check is counted, and after 3 of them it is quarantined;
2. **votes on the sealed part** of the frame. This is the type, the network, the 30 signed bytes and the MAC. The relay id and hop count are left out, because they legitimately differ when two gateways hear the same record through different beacons;
3. applies the rule: **a version counts when `quorum` gateways (2 of 3) reported identical bytes.**

| state | when | on the dashboard |
|---|---|---|
| CONFIRMED | 2 or 3 gateways agree | normal pin, "2/3" or "3/3" |
| unconfirmed | only one gateway after 4 s | dashed pin, "1/3" (hidden with `--strict`) |
| CONFLICT | two different signed versions, no majority for 10 s | alert |

**What it protects against, which a signature alone cannot:**

- **A faulty or hacked gateway.** A gateway holds the key, so it can make a correctly signed record. With the vote, no single gateway can put anything on the map.
- **An attacker with a stolen key, near one gateway.** Only that gateway hears the attacker, so the record never reaches a quorum.
- **Two versions of the same record.** When two different signed versions exist, the majority wins. The gateway that disagrees is named, and after 3 disagreements it is quarantined. The operator can release it.

**What it does not do.** Three gateways that all hear the same forged frame will all vote for it. The signature is what stops forged frames; the vote stops single points of failure.

---

## 3. Where is the robot? Three spheres (`multilateration.py`, `kalman.py`, `tracker.py`)

### 3.1 The idea, and why three are enough

**Each gateway gives one sphere.** It measures its distance r to the robot by two-way time of flight: a ping and its echo, like a radar. The robot lies on a sphere of radius r around that gateway.

**Three spheres give the position.** They meet in two mirror points. The robot is on the floor with its antenna height known, so on the floor the three spheres become three circles that meet in **one point**.

**GPS needs four satellites; we need three.** A GPS receiver measures one-way travel times with a cheap clock, so its clock error is a 4th unknown. Two-way ranging cancels the clock error.

### 3.2 From three ranges to one fix

1. **Start** from the closed-form solution: subtracting one circle's equation from the others gives a linear system.
2. **Refine** with weighted least squares (Gauss-Newton) on the slant ranges, with Huber weights so that one bad range does not pull the fix.
3. **HDOP** measures how the gateways' geometry turns range errors into position errors. With the three gateways around the building it is 1.2 to 1.6 everywhere inside. Gateways in a line are detected.
4. **RAIM** (the integrity check of aviation GPS): with n ranges and 2 unknowns there are n − 2 spare measurements. The normalised residuals must follow a chi-square law, so a range that is off is detected. With 4 gateways the bad one is also named and excluded; the tests check both.

### 3.3 Walls (NLOS)

A wall in the way makes the signal path longer, so time-of-flight ranges come out **too long, never too short**. The same wall also weakens the signal. The ONA uses this:

- **Count the walls.** The ONA compares the RSSI with what free space would give at that range. The excess loss divided by the loss per wall (≈ 6 dB) estimates how many walls are in the way.
- **Correct the range.** It is shortened by the expected extra length (≈ 0.6 m per wall), and its error bar is widened.

In simulation, every wall has its own loss (4–9 dB) and its own extra length (0.2–1.0 m). The ONA only knows the averages, so its correction is deliberately imperfect. Over 6 missions:

| | position error (RMS) | error bars honest (error < 3σ) |
|---|---|---|
| without the wall correction | 1.07 m (0.50–2.09) | 71–100 % of pings |
| **with it** | **0.42 m (0.15–0.88)** | **100 %** |

The two wall constants (dB and metres per wall) must be measured on site: drive the robot to 2 or 3 known points.

### 3.4 The filter (tight coupling)

The ONA does not just draw the raw fix. `RangeEKF` (your friend's filter on the ground plane) combines two inputs:

- **Prediction:** the robot's own displacement since its last ping (the difference of two SLAM poses, rotated into east/north).
- **Correction:** one update per gateway range. A single range still corrects the robot along its line of sight, so tracking continues when only 1 or 2 gateways hear it. In the test, with one gateway down for 80 s, the worst error was 0.56 m.

Refinements, each found by a failing test:

| refinement | why |
|---|---|
| Robust updates: Huber above 2σ, rejection above 5σ | a single bad range must not move the estimate |
| Range noise × 2.5 in the filter | the wall error is the same at every ping (same walls), so averaging pings does not remove it. Without this the filter became over-confident and raised false "drift" alarms. |
| Standing still, each ping counts less | a robot parked at the exit repeats exactly the same wall errors ping after ping. Without this the error bar kept shrinking while the error stayed (found in a test). |
| A robot cannot teleport | if the robot's own pose moves faster than 1 m/s between two pings (SLAM relocalised or slipped), the filter does not follow it and stays confident (the robot moved at most half its top speed). The gateways decide where the robot went, and the jump shows up as a drift within a few pings. |
| Adaptive process noise | when the ranges keep disagreeing with the prediction, the robot's odometry is trusted less |
| Re-acquisition | when a drift is confirmed and the filter had followed the slipping odometry (it disagrees with the three spheres), it restarts from them; if it had rejected the jump, it is right and is kept |
| Fewer than 3 gateways | each range counts less (× 1.6 noise): with no spare range a wall bias cannot be seen. Error bars stay honest with a gateway ignored (100 % in the test). |

### 3.5 Reported vs measured: the physical cross-check

Every ping carries the robot's **own** pose, which the ONA translates to GPS with the entrance calibration: the *reported* position. The gateways give the *measured* position.

- **They agree** within their error bars: the position on the map is confirmed by physics, not only by a signature.
- **They drift apart** for 3 pings in a row (3σ), and a 3-sphere fix of the same ping disagrees with the robot too: **SLAM_DRIFT** alert. With only two gateways (one down or ignored) the ONA keeps tracking but does not accuse the robot's SLAM: a live run showed a false alert there. The robot's SLAM has slipped, as it did in our Gazebo runs when the map smeared, and the dashboard follows the measured position. In the test, a 2.9 m slip is detected and the measured track stays 0.32 m RMS from the truth afterwards.
- **"Agrees again" (SLAM_OK)** needs 3 agreeing pings **and** the robot's own pose back within max(1 m, half the drift). A wider error ellipse on the gateways' side (a parked robot, a gateway lost) is not a recovery: without this rule the alert flapped on and off in a live run with one gateway quarantined.

### 3.6 The frame translation, checked during the mission

Good fixes are pairs of (map position, ENU position): exactly what Umeyama needs. The ONA refits the map → ENU alignment from them during the mission and compares it with the entrance calibration.

Pairs are not collected while the robot's SLAM is flagged as drifting (and the last few before the alert are dropped): a SLAM slip moves the robot's map, not the entrance calibration. Through walls this detects **gross** errors only: a frame off by more than 1.5 m or 6°, for example a wrong north at the entrance. In the tests the verdict is "holds" on every nominal mission.

---

## 4. Frames (`lmb2.py`)

All frames are 44 bytes with the same header and the same MAC rule as the beacon records:

- **the MAC**: `HMAC-SHA256(key, "LMB2" ‖ b[0..1] ‖ b[6..35])`, truncated to 8 bytes, at bytes 36..43;
- **the type byte is signed**, so a MISSION cannot be replayed as a ROBOT frame (tested).

| type | name | from → to | content |
|---|---|---|---|
| 0x21 | RECORD | beacon → everyone | the beacon's record (unchanged, see LMB2 ARCHITECTURE.md) |
| 0x22 | DIGEST | gateway → beacons | pull request (unchanged) |
| **0x23** | **ROBOT** | robot → gateways | robot id, seq, role and phase, x/y in mm (private map frame), yaw, own error estimate, distance driven, battery, targets done/total, time, mission id + ack flag, last beacon passed |
| **0x24** | **MISSION** | ONA → gateways → Executor | mission id, version, robot id, flags (return to exit, abort, in order), up to 9 target beacon ids per frame, multi-frame for more |

The ROBOT frame's layout, byte by byte, is in the header of `ona/lmb2.py`.

---

## 5. Store and forward (`store.py`, `uplink.py`)

- **The queue.** Every message for the Command Post goes into the SQLite queue first (your friend's queue). Priorities:
  - 3: victims, radiation, gas, fire, and alerts;
  - 2: other hazards, robot positions, lost beacons;
  - 1: trail beacons, vote counts, ONA status.

  A robot's position is only worth its latest value: a new one replaces the unsent older one ("coalescing").
- **LTE.** Everything, most important first.
- **Satellite (Iridium SBD model).** Used only when LTE is down, for priority ≥ 2 only, one message at a time, compacted to fit one 340-byte SBD message (a beacon record fits).
- **No link.** Nothing is lost. The queue survives a restart, and the backlog is delivered when a link returns.
- **Demo options.** `--lte-outage 60-150` simulates LTE going down between 60 s and 150 s after start. `--no-sat` turns the satellite off.
- **Audit.** Every decision (rejected frame, vote, alert, link change, briefing) goes into the `audit` table of the same database.
- **The dashboard knows.** A switch of link is an event (**LINK_CHANGE**, priority 3, so it goes over the satellite too). The dashboard shows SATELLITE in the ONA panel, keeps the gateways green ("sat") instead of turning them red for missing heartbeats, and shows the robots' satellite summaries (position ± error, phase, progress) without stale distance circles.

---

## 6. Briefing the Executor

1. The operator selects targets on the dashboard and clicks **Dispatch**.
2. The ONA polls `GET /api/mission-dispatch/latest` over the uplink, turns the mission into signed MISSION frames, and has every gateway transmit them.
3. The Executor, waiting at the exit, checks the signature and assembles the frames. From then on its ROBOT pings carry the mission id with the ACK flag.
4. The ONA sees the acknowledgement through the gateways (**MISSION_ACK** on the dashboard) and stops repeating the briefing. Until then it repeats it every 8 s, at most 12 times.
5. The Executor's progress (targets done / total) is in every ping.
6. **During the mission** a new briefing re-targets the Executor on the spot (a hazard being treated is finished first). **Call the Executor back** on the dashboard sends a signed ABORT: it stops, even mid-treatment, and follows the beacons out. Untick "back to the exit afterwards" and it waits at its last target for the next briefing. A briefed beacon whose record has not arrived yet is taken as soon as it does.

If the gateways ever hear a signed briefing the ONA did not send, that is an **UNKNOWN_BRIEFING** alert.

---

## 7. Running it

The ONA runs on the Windows laptop next to the Command Post, or in the VM. Only numpy is needed, plus pyserial for real boards.

| where | command | what |
|---|---|---|
| Windows | `windows\run_ona.bat` | ONA, UDP port 47100, posts to `http://localhost:3000` |
| Windows | `windows\simulate_zone.bat` | the built-in building, 16 beacons, the Executor waiting for a briefing |
| Windows | `windows\simulate_faults.bat` | the same with a lying gateway, a forged victim, a crushed beacon and a SLAM slip |
| VM | `./run_ona.sh --command-post http://10.0.2.2:3000` | the ONA in the VM, dashboard on Windows |
| real gateways | `python -m ona --serial gw1=COM3 --serial gw2=COM4 --serial gw3=COM5` | three boards on USB. The ONA sends them `KEY`, `NET`, `NAME` and `TIME`, and briefings as `TX <hex>`. |
| Windows | `windows\bpsim_to_ona.bat` | the beacon network simulator (C) with 3 gateways → the vote → the dashboard |
| VM, Gazebo | `~/writer_robot_ws/run_executor.sh demo headless ona` | the real robots: their `ona_link` node plays the 3 gateways (walls from the world file), this ONA runs in the VM and posts to the Windows dashboard |
| Windows + VM | `windows\run_ona_for_gazebo.bat`, then in the VM `ONA_HOST=10.0.2.2 ~/writer_robot_ws/run_executor.sh demo headless ona` | the ONA on Windows for the Gazebo robots |

Useful options:

- `--strict`: never show 1/3 records;
- `--lte-outage A-B,C-D`: simulated LTE outages;
- `--no-sat`: satellite off;
- `--db FILE` and `--fresh`: the queue file, and starting with it empty;
- `--quiet`: no console output.

Zone simulator options:

- `--speed 2` and `--minutes 8`;
- `--no-wait`: don't wait for a briefing;
- `--robot writer`;
- faults: `--liar gw3`, `--forge gw2@60`, `--corrupt gw1@30`, `--gw-down gw2@100-160`, `--kill 6@90`, `--slam-slip 150:2.5,-1.5`;
- `--mission FILE.json --world FILE.world`: a mission recorded by the Writer, with the Gazebo walls.

`ona_config.json` holds:

- the mission key and network id;
- the quorum;
- the map anchor (lat, lon, alt, and the compass direction of map +x), or 3 or more `entrance_anchors` for a full Umeyama fit;
- the three gateways, in map coordinates (`local`) or GPS (`lat`, `lon`, `alt`);
- the tracker constants (robot antenna height, wall correction).

---

## 8. Tests

| test | checks | result |
|---|---|---|
| `tests/test_ona.py` | frames: the brief's record, byte for byte, and 300 random records; the geodesy of `trans.py`, including the original example; `PositionEKF` and `RangeEKF`; the spheres, HDOP and RAIM; the vote; the queue, including your friend's original database file; LTE/satellite/no link; the ONA end to end with a lying gateway, a SLAM slip and the briefing with its acknowledgement; a gateway sending a raw frame and its JSON line counts once; **bpsim --gateways 3** through the vote (14/14 confirmed, the destroyed beacon reported lost, no gateway blamed) | **70 / 70** |
| `tests/test_zone.py` | whole missions (zone simulator → ONA): 16 beacons all confirmed; briefed targets treated in order; robot positioned through walls with honest error bars; lying gateway quarantined; forged victim never confirmed; crushed beacon reported lost; one gateway down; SLAM slip, alert raised once (no flapping) and not blamed on the entrance calibration; no false drift with a gateway ignored; corrupted frames; strict mode | **20 / 20** |
| `tests/test_vs_beaconnet.py` | 600 random records: byte-identical to the beacons' own codec (which is byte-identical to the C firmware); the vote key is the same for a raw frame and its JSON line; ROBOT/MISSION frames are never taken for records | **3 / 3** |
| writer_robot `tools/test_ona_link.py` | the Gazebo robots' side: its copy of `lmb2.py` is this one; the gateways' radio model in both worlds; the **whole chain in-process**: executor_node + ona_link_node → this ONA → briefing → the Executor treats exactly the briefed targets in order; 43/43 records confirmed by 3/3; the moving Executor measured through the walls (0.66 m RMS); a SLAM slip caught | **18 / 18** |

Measured over 8 simulated missions (the robot drives about 60 m through the 22 × 14 m building):

- **Records:** 16/16 confirmed in every run.
- **Robot position:** 0.15–0.88 m RMS, 95 % of pings within 0.97 m.
- **Error bars:** honest in 100 % of pings.
- **False drift alarms:** none.

---

## 9. Hardware

**Gateway.** The LMB2 gateway board (ESP32 + SX1276 at 868 MHz), plus a ranging radio:

- **Semtech SX1280** (2.4 GHz LoRa with a built-in time-of-flight ranging engine, a few metres through walls, 1 m-class in line of sight after averaging); or
- **a UWB module** (Qorvo DW3000: 10 cm in line of sight, shorter range through walls).

**Power and position.** Each gateway runs on a battery and has a GPS fix, since it is outside: its position is surveyed.

**Robot.** It carries the ranging responder.

**Without ranging hardware.** The ONA falls back to RSSI ranging, which is honest but crude: about ±45 % of the distance.

**Firmware.** Done (`beacon_net/firmware/gateway_node`): one raw JSON line per frame heard (`{"ev":"rx","gw":…,"frame":…,"rssi":…,"snr":…}`), and a `TX <hex>` command that transmits a briefing within the 1 % duty budget. Tested on the PC build of the firmware. The beacons ignore ROBOT and MISSION frames (counted as `rx_foreign`). The `range_m` field waits for the ranging radio.

---

## 10. Limits, stated plainly

- **The accuracy numbers are simulation numbers.** The wall model (4–9 dB, 0.2–1.0 m per wall, 4 dB shadowing) is standard but still a model. Measure the two wall constants on site.
- **Three gateways give one spare measurement.** RAIM detects a bad range but cannot say which one; a 4th gateway would.
- **The vote needs two gateways in range of the same beacons.** Place them where each hears the entrance area. `--strict` hides records that only one gateway hears.
- **A shared mission key** is the weak point of LMB2 (a captured beacon reveals it). The vote limits what one captured *gateway* can do, not what a stolen key can do everywhere.
