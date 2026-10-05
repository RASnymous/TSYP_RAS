# The Executor robot: following the beacons to complete the mission

The Executor enters the zone after the Writer. It has **no map and no data from the Writer** except the beacons' 44-byte LoRa frames (LMB2). With those frames alone it:

1. drives **beacon by beacon** to every hazard the Writer marked (radiation, fire, gas, victims);
2. **treats** each hazard from a safe 1.6 m standoff: shielding, extinguishing, sealing the leak, first aid;
3. **returns to the exit** by following the beacons' `next` hops, and stops: `MISSION COMPLETE`.

The pictures `docs/sim_mission_big.png` and `docs/sim_mission_retreat_test.png` show a whole mission in the 2D simulator: the Writer's run on top, the Executor's run below.

## 1. How the beacons guide it

Each beacon's LMB2 record has a `next` field ("the way out"). It gives the id, distance and compass bearing of an older beacon. The Writer sets `next` so that the beacons form a **tree rooted at the EXIT beacon**:

- **EXIT beacon.** It is dropped at the Writer's start. That is the entrance, and the way out.
- **Every new beacon links to the last beacon the Writer dropped *or drove past*.** It never links to one left far behind, so each link is a short (≤ 3.7 m) drivable leg.
- **No duplicates.** When the Writer drives back along its own trail, it does not drop new beacons. It passes the old ones instead, so the tree takes the short way round. A beacon behind a wall never counts: this is checked on the SLAM map.

For a trail beacon, the record position is the beacon itself. For a hazard beacon, it is the hazard; the Executor finds the beacon from its `next` vector (drop = parent − next).

**Route to a hazard** = the tree path from where the Executor is to the hazard's beacon. It goes up the `next` hops to the common beacon, then down.

**Route out** = follow `next` from beacon to beacon until the EXIT.

## 2. What the Executor does

The brain is `writer_robot/executor_core.py`, with no ROS inside. It reuses the Writer's navigation from `explorer_core.py`: path planning, pure pursuit, keep-out zones, recovery and LiDAR safety.

| phase (log / RViz) | what happens |
|---|---|
| `WAIT` | The robot receives the beacon frames and checks each HMAC with the mission key. It waits until no new beacon has arrived for 2 s and the EXIT beacon is known, then builds the tree and picks the targets. If a beacon between a hazard and the exit has not been heard yet, it waits up to 20 s more. A record that arrives later still becomes a target, even after `MISSION COMPLETE`: the robot sets off again. |
| `GOTO` | It drives to the next target beacon by beacon. The order is the nearest target along the tree, or the Command Post's order when it is briefed (section 7). Each leg is planned on its own LiDAR map. Space it has not seen yet counts as passable, because the Writer drove there. It drives through the beacons on the way without braking. Every hazard is a 1.2 m keep-out zone. |
| `APPROACH` | From the hazard's beacon, where the Writer stood when it saw the hazard, the robot drives to a standoff point. It considers points all around the hazard, 0.3 m apart, on rings at 1.6, 2.0, 2.4 and 2.8 m. Each point needs a clear view of the hazard on the robot's own map. It picks the cheapest reachable point. The Writer's line of sight and the 1.6 m ring are strongly preferred; the outer rings are used only when nothing nearer can be reached. On arrival it checks that nothing stands between it and the hazard (the map or its camera). If an approach fails (no path, stuck, no view, more than 60 s, or only a detour of more than 12 m through unmapped space), it tries up to 4 standoff points. If the hazard is in plain sight within 3 m, it treats it from where it is. |
| `TREAT` | It turns to face the hazard and treats it for 4 s (`treat_time`). Its own camera confirms the hazard and refines its position. A green disc appears on the floor in Gazebo, and the zone turns green in RViz. |
| `RETURN` | Before going home, it makes a **second attempt** at every hazard it had to skip, because it now knows much more of the map. Then it follows the `next` hops back to the exit. |
| `DONE` | Stopped at the exit: `MISSION COMPLETE: 4/4 targets treated`. |

Targets are every hazard or victim record that has not been retracted and whose confidence has not faded below 0.15. Confidence fades with each kind's half-life: gas in 10 minutes, radiation in 6 hours. By default the Executor counts ages as if it entered right after the Writer finished (`age_mode: mission`). A camera sighting of a hazard that is **not** in any record also becomes a keep-out zone, and the robot turns back if it is in its way.

Its own camera confirms a hazard from any distance. Only sightings closer than 2.6 m move the hazard's position, because the range of a far block measured from where it touches the floor can be 20 % off.

The log says why a hazard was skipped: `approach failed (...) - trying another standoff point (2/4)`, `in plain sight 2.9 m away - treating from here`, `target #13 unreachable (...) - skipped`, and later `second attempt at RADIATION #13`.

## 3. Run it (Ubuntu VM)

1. **Writer first.** It explores and drops beacons, and `mission_recorder` saves every beacon to `~/writer_robot_ws/missions/latest.json`:
   ```bash
   ~/writer_robot_ws/run_explorer.sh
   ```
   Wait for `DONE` (`tail -f /tmp/explorer.log`), then press Ctrl-C.
2. **Executor.** Same world, with the Writer's beacons lying where they fell:
   ```bash
   ~/writer_robot_ws/run_executor.sh
   ```
   Follow it with `tail -f /tmp/executor.log`.

**To skip the Writer**, use a ready-made mission recorded by the 2D simulator on the same map:
```bash
~/writer_robot_ws/run_executor.sh demo
```
Options:
- `demo retreat`: the small map.
- `FILE.json`: any mission file.
- `headless`: no Gazebo window.

What runs:

| process | role |
|---|---|
| `mission_world` | Builds `/tmp/executor_world.world`: the Writer's world plus a static beacon model at every drop point. |
| `gazebo_sim.launch.py robot:=executor` | The Executor robot: the same base, LiDAR mast and camera as the Writer, but blue, with a treatment canister instead of the beacon magazine. |
| `slam_toolbox` | The Executor's own map. |
| `beacon_radio_sim` | The LoRa receiver. It reads the frames from the mission file, verifies the HMAC, decodes them and publishes `/executor/lora_rx`. |
| `vision_event_detector` (remapped) | The Executor's camera, on `/executor/hazard`. |
| `executor_node` | Drives the robot on `/cmd_vel`. It publishes `/executor/route`, `/executor/plan`, `/executor/markers`, `/executor/status` and `/executor/mission_events`. |

## 4. Parameters

Set these with `ros2 run writer_robot executor_node --ros-args -p use_sim_time:=true -p NAME:=VALUE`, or in `run_executor.sh`.

| parameter | default | meaning |
|---|---|---|
| `target_kinds` | `RADIATION,THERMAL,GAS,VICTIM` | what to treat |
| `target_ids` | (all) | e.g. `"12,30"`: only these beacons |
| `standoff` | 1.6 | m from a hazard while treating it (victims: `victim_standoff` 0.8) |
| `treat_time` | 4.0 | s per target |
| `keepout_radius` | 1.2 | m around every hazard |
| `min_confidence` | 0.15 | skip events that have faded below this |
| `age_mode` | `mission` | `wallclock`: age the records by the real clock |
| `v_max` | 0.70 | m/s (`run_executor.sh slow`: 0.5 m/s) |
| `treat_max` | 3.0 | m: a hazard in plain sight this close can be treated from where the robot is |
| `approach_tries` | 4 | standoff points tried before a hazard is skipped |
| `approach_timeout` / `approach_max_path` | 60 / 12 | s per standoff point / m of path to it |
| `retry_skipped` | true | one more attempt at every skipped hazard before going home |
| `refine_range` | 2.6 | m: only closer camera sightings move a hazard |
| `link_wait` | 20 | s to wait for a missing beacon between a hazard and the exit |

`beacon_radio_sim`: `mission_file`, `key` (`demo` or the 32-hex mission key), `rate_hz`. `mission_recorder`: `mission_dir`, `anchor_lat`, `anchor_lon`, `key`, `net_id`.

## 5. Beacon-graph navigation (Dijkstra / A*): ready, not wired in yet

`writer_robot/beacon_graph_nav.py` turns the beacons into a graph:
- **Nodes** are the beacons at their drop points.
- **Tree edges** follow `next`; the Writer drove them.
- **Shortcut edges** join beacons closer than 4 m with a wall-free straight line.
- **Hazards:** edges that cross a keep-out zone are removed.

It finds routes with Dijkstra or A* (`dijkstra`, `astar`, `tree_path`, `shortest_from`, `order_targets`). The Executor still follows the tree today. The end of the file lists the five steps to plug the graph in.

Try it without ROS:
```bash
python3 tools/beacon_graph_demo.py --png /tmp/graph.png
python3 tools/test_beacon_graph.py
```

On the big demo mission it adds 46 wall-free shortcuts. The routes from the EXIT to the four hazards get 1.6–4.9 m shorter. A* expands about half as many beacons as Dijkstra for the same routes.

## 6. Tested without ROS (also on Windows)

| command | result |
|---|---|
| `python3 tools/sim_mission.py --png mission.png` | The whole mission: the Writer explores and drops beacons (56 on the big map), and the Executor follows them. Big map: all 4 hazards treated, about 75 m driven, about 3 simulated minutes, back at the exit. `--mission missions/demo_big.json --briefing 5:25,11,20` replays the ready-made mission with a Command Post briefing (section 7). |
| `python3 tools/run_mission_tests.py` | 21 missions: 3 maps, several seeds, a camera that sees later, a radio that joins the record cycle halfway, and an Executor map misaligned with the Writer's (0.5 m, 3°). Five more replay the ready-made demo mission with problems Gazebo can cause: an Executor camera whose range is 25 % off, beacons heard late, a door narrowed to 0.6 m, and a door blocked completely (the hazard must be skipped cleanly). Four more brief the Executor from the Command Post: targets in order, re-targeted and staying, called back (abort), and a briefed record heard late. Five run in the Gafsa mine: two seeds (victims reached first), a briefing that samples the rich phosphate layer then reaches the miner, and the mine sensors reading 15 % too far and too short. **26/26 pass.** Every run: all frames verified, every hazard reported by the Writer treated, no collision, never closer than 1.0 m to a hazard, at least 90 % of the driving between beacons within 1 m of a beacon link, back at the exit. |
| `python3 tools/test_nodes.py` | The real ROS nodes on a small stand-in for ROS: the beacon node, recorder, radio and Executor node drive the simulator through the ROS topics. **70/70 checks pass** (the mine sensors node included). |

Windows: `windows\sim_mission_big_map.bat`, `windows\sim_mission_retreat_test.bat`, `windows\run_all_tests.bat`.

**Real hardware.** On the real robot, a LoRa gateway (`beacon_net/firmware/gateway_node`) replaces `beacon_radio_sim`. A small bridge would then publish the gateway's decoded records on `/executor/lora_rx`, with map `x`, `y` computed from the same anchor. That bridge is not written yet.


## 7. Briefed by the Command Post

The operator can choose the Executor's targets on the dashboard. The Outside Network Area signs the briefing as
LMB2 MISSION frames (type 0x24), its three gateways transmit them, and `ona_link_node` checks the signature,
assembles the parts and publishes `/executor/briefing` (latched). `ExecutorCore.set_briefing()`:

| briefing | what the Executor does |
|---|---|
| targets `[25, 11, 20]` | exactly these, **in this order**; hazards are approached and treated, a trail or exit beacon is visited; a briefed hazard is taken even if its confidence has faded |
| arrives before the start | used for the whole mission (`wait_briefing` seconds of waiting, 60 with the `ona` option; 0 = don't wait) |
| arrives during the mission | re-targets on the spot (a hazard being treated is finished first) |
| `return_to_exit: false` | after the last target it stops there and waits for the next briefing |
| `abort: true` | stops at once (even mid-treatment) and follows the `next` hops to the exit |
| a target with no record yet | taken as soon as its record (or the missing link to the exit) arrives |

The robot acknowledges in every ROBOT frame (`/executor/telemetry` → `ona_link_node`): mission id, ack flag,
targets done / total, phase, last beacon. Run it:

```bash
~/writer_robot_ws/run_executor.sh demo headless ona
```
then **Dispatch mission** on the dashboard. Without Gazebo:
```bash
python3 tools/sim_mission.py --mission missions/demo_big.json --wait-briefing 30 --briefing 5:25,11,20
python3 tools/sim_mission.py --mission missions/demo_big.json --briefing 40:20,30:stay --briefing 150:13
python3 tools/sim_mission.py --mission missions/demo_big.json --briefing 45:abort
python3 tools/test_ona_link.py          # the real nodes + the real ONA, in-process
```
