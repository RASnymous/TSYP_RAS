# writer_robot: the Writer and Executor robots (ROS 2 package)

TSYP14 "The Living Map" · ROS 2 Lyrical · Gazebo Jetty · slam_toolbox (no Nav2).

- The **Writer** explores the contaminated building on its own, turns back at hazard blocks, checks every room
  with its camera, and drops a tree of LoRa beacons (exit, trail, one per hazard). Each beacon becomes a signed
  44-byte LMB2 record. See [EXPLORER.md](EXPLORER.md).
- The **Executor** enters knowing only those radio frames. It follows the beacons to the hazards, treats each one
  from a safe distance and follows them back out. It can be **briefed from the Command Post** (these targets, in
  this order; stay; come back now). See [EXECUTOR.md](EXECUTOR.md).
- **The Gafsa mine** (`run_explorer.sh mine`, `run_executor.sh mine`): the same robots in a simulated
  room-and-pillar phosphate panel, with a mine sensor head (thermal camera, roof scanner, multi-gas, radon and
  gamma, XRF wall probe, headlights) instead of the colour-block vision. Dangers, resources (phosphate grade, gold
  and gemstone finds) and *searched, nobody here* areas become beacons. See `../docs/3_Gafsa_Mine_Case.pdf`.
- The **`ona_link` node** is a robot's radio as the three gateways of the Outside Network Area hear it: signed
  position pings, the distance each gateway measures through the walls, the beacon records, the briefings.

Install, run and every command: `../START_HERE.md` and `../docs/1_Living_Map_Guide.pdf`.
How it works: `../docs/2_Living_Map_Architecture.pdf`.

```bash
bash ~/living_map/writer_robot/install.sh                 # copies into ~/writer_robot_ws and builds
~/writer_robot_ws/run_explorer.sh headless ona             # the Writer (add "retreat" for the small map)
~/writer_robot_ws/run_executor.sh headless ona             # the Executor, on the mission the Writer recorded
~/writer_robot_ws/run_explorer.sh mine headless ona        # the Writer in the Gafsa mine
~/writer_robot_ws/run_executor.sh mine headless ona        # the Executor on the ready-made mine mission
bash ~/writer_robot_ws/stop_demo.sh                        # stop everything
```

## Inside

| path | what |
|---|---|
| `writer_robot/explorer_core.py`, `frontier_explorer.py` | the Writer's brain (no ROS inside) and its ROS node |
| `writer_robot/executor_core.py`, `executor_node.py` | the Executor's brain and its ROS node |
| `writer_robot/vision_event_detector.py` | RGB-D camera: purple radiation, red fire, yellow gas blocks |
| `writer_robot/mine_site.py`, `mine_sensors_node.py` | the mine instruments (gas, radon, gamma, thermal, roof, XRF, victim search) and their ROS node |
| `tools/make_mine.py` | writes the Gafsa mine: the world, its site file, the dashboard plan and the ONA configs |
| `writer_robot/beacon_drop_node.py`, `beacon_logic.py` | when and where to drop a beacon, the trapdoor servo, the beacon tree |
| `writer_robot/mission_recorder.py`, `mission_log.py`, `lmb2.py` | beacons → signed 44-byte LMB2 records → the mission file |
| `writer_robot/beacon_radio_sim.py` | the Executor's LoRa receiver in simulation (replays the mission file) |
| `writer_robot/ona_link_node.py`, `ona_radio.py`, `ona_lmb2.py` | the link to the Outside Network Area (three virtual gateways) |
| `writer_robot/mission_world.py` | the Executor's Gazebo world, with the beacons where the Writer dropped them |
| `writer_robot/beacon_graph_nav.py` | beacon-graph navigation (Dijkstra / A*), tested, not wired in |
| `writer_robot/frame_translator_node.py`, `event_detection_node.py` | map → GPS on a ROS topic; a simulated detector |
| `launch/`, `urdf/`, `worlds/`, `models/`, `config/` | Gazebo launch, the robot model, the worlds, the beacon and "treated" models, SLAM / RViz / gateway settings |
| `missions/` | three ready-made missions (big map, small map, Gafsa mine) for the Executor |
| `tools/` | the 2D simulator and every test (they run without ROS, also on Windows) |
| `windows/` | double-click launchers for the 2D simulator and the tests |
| `test_reports/` | the output of the last test runs |
| `run_explorer.sh`, `run_executor.sh`, `stop_demo.sh`, `check_sim.sh`, `install.sh` | the scripts (install.sh copies the first four into the workspace) |

License: MIT.
