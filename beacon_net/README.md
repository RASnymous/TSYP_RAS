# Living Map Beacon Network (LMB2)

The radio part of the TSYP14 Living Map: the beacons the Writer drops, the LoRa gossip network between them, and the gateway that feeds the Command Post.

* **44-byte record** per beacon: id, kind, GPS position ± error, severity, confidence, next-hop vector (the way out), timestamp, TTL, confidence half-life (gas fades in minutes, an obstruction lasts days)
* **LoRa 868 MHz** (SF9, 288 ms per frame), **HMAC-SHA256** (64-bit tag, per-mission key)
* **Gossip replication**: own announce, reactive relay with route-aware suppression and passive acks, anti-entropy, pull digests. A crushed beacon's record survives in its neighbours, and the Command Post is told the beacon is lost.
* Same portable C core in the **ESP32 firmware**, the **gateway**, the **network simulator** and (mirrored byte-for-byte) **Python**

Read **[ARCHITECTURE.md](ARCHITECTURE.md)** for the full design, security analysis, radio budget and test results.

## Quick start

```bash
cd host
make                 # bptool + bpsim
./bptool example     # the brief's victim record -> 44-byte frame
./bpsim              # 60-min mission, 14 beacons, beacon loss, attacker -> report + 13 checks
make test            # C <-> Python <-> firmware cross-checks
make matrix          # 42 stress scenarios
```

Live on the dashboard (from `../command_post`, run `npm start` first):

```bash
./bpsim --speed 20 | python3 ../python/beaconnet/gateway_bridge.py --stdin --server http://localhost:3000
```

Three gateways through the Outside Network Area (2-of-3 vote, robot positioning, LTE/satellite uplink, briefings): see `../ona/README.md`.

```bash
cd ../../ona && ../beacon_net/host/bpsim --gateways 3 --speed 20 | python3 -m ona --stdin --config ona_config_bpsim.json
```

Windows: everything except ROS runs natively; open `windows/README.md` (double-click launchers, prebuilt .exe).

Hardware: `firmware/beacon_node` and `firmware/gateway_node` (ESP32 + SX1276, Arduino IDE or PlatformIO). The library is `lib/BeaconProto`. Provisioning and the serial commands are in ARCHITECTURE.md §9.

Writer (ROS 2): `python3 ros/beacon_record_node.py --ros-args -p dashboard:=http://10.0.2.2:3000`
