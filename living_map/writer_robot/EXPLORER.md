# The Writer robot: the frontier explorer

The robot now explores the whole map by itself. Hazard blocks never stop it:

- **If a hazard is in its way, it turns back the way it came.** This applies to the purple radiation, red fire and new yellow gas blocks.
- **It keeps exploring by other routes** until nothing reachable is left.
- **It checks everything with its camera**, then drives back to where it started and stops.

It is started with `~/writer_robot_ws/run_explorer.sh` (options: `retreat` small map, `headless`, `ona`, `slow`, `gpu`, `ekf`).

## 1. Design choices that keep the robot moving

Each row is a problem a simple explorer runs into in Gazebo, and what this package does about it:

| problem | effect in Gazebo | what the package does |
|---|---|---|
| **A dropped beacon model with a collision box, spawned at the robot's own position.** | Every drop could hit the chassis and jam or shove the robot. | `models/lora_beacon`: the beacon is static and visual-only (no collision). It is dropped under the magazine, on the ground. |
| A LiDAR scan plane at 0.20 m cuts through the robot's own beacon magazine and antenna. | Phantom obstacles appeared right behind the robot, in the scan and in the SLAM map. | The LiDAR sits on a 14 cm mast, so its scan plane is at 0.34 m, above the magazine, the antenna and the dropped beacons (< 0.17 m). The explorer also ignores any point inside the robot's footprint. |
| A purely reactive explorer backs off from a sighting and then drives forward again toward it, and guesses a range when there is no depth. | It oscillates in front of the block, which looks like stopping. | A real planner (below). The vision node measures the range from the depth image, or from where the block touches the floor. |
| No notion of "the map is finished". | It wanders forever. | Frontier exploration plus a camera check, then return home. |

## 2. What the robot does

The brain is `writer_robot/explorer_core.py`. It contains no ROS code, so exactly the same code runs in Gazebo and in the 2D test simulator. `writer_robot/frontier_explorer.py` connects it to ROS.

| state (log / RViz text) | what happens |
|---|---|
| first turn | Before the first plan, the robot makes one smooth full turn on the spot at 1.0 rad/s. slam_toolbox adds a scan to its map only after 0.3 m or 0.3 rad of motion, at most one every 0.5 s, and marks a cell only when several rays crossed it, so a robot standing still has a map of barely 1 m around itself. The turn adds about 12 scans. If the map is still tiny, the robot drives about 0.6 m toward the most open direction the LiDAR sees (up to 3 times), because moving always makes SLAM add scans, and only then turns again. It never declares the map complete before it has driven 1 m, and it writes a `map check:` line to the log. If the TF heading hardly changes during a full turn, the log shows a WARNING (the EKF does not see the turn). |
| `PLAN` | The robot stops for a moment. It picks the cheapest target among three kinds, then plans a path to it with Dijkstra on the SLAM map. The map is inflated by the robot size and a soft margin keeps the path away from walls. |
| `FOLLOW` | Pure-pursuit path following, up to 0.7 m/s (0.5 m/s with the `slow` option). Speed follows the heading error smoothly, slows before sharp corners and before obstacles, and ramps up gently. The robot turns on the spot at sharp corners and never cuts them. When the goal becomes useless on the way (already explored or already seen), it picks the next goal without stopping. |
| `RETREAT` | **Turning back the way it came.** A hazard showed up on its path, or it is within 1.8 m of one. The robot reverses 0.3 m, turns round and drives back along its own recorded trail, at least 1.5 m and until it is 2.5 m from the hazard. It then plans again, and the keep-out zone makes it choose another route. |
| `LOOK` | Camera check. The robot turns on the spot, at up to 1.6 rad/s, until the camera faces an area it has not looked at, and goes straight on to the next one (at most 3 per stop): the turn itself sweeps the camera over the area. It holds still only for 0.25 s before turning back, and for 0.35 s in front of a suspicious object. About 2 s per stop. |
| `RECOVER` | No progress along the path for 8 s, told to drive for 2 s without moving (`blocked`: something the LiDAR does not see), or an obstacle keeps blocking it. The robot backs up 0.3 m, turns toward the open side and plans again. A goal that fails twice is abandoned. |
| `HOME` | Nothing is left: `EXPLORATION COMPLETE`. The robot drives back to its start point. |
| `DONE` | Stopped at the start. 12 s later it checks the newest map once more, in case something new appeared. |

Every simulated minute the log gets a line that shows where the time goes, and a `total time` line at the end:

```
[explorer] time so far 4.0 min: driving 177 s, camera looks 46 s, planning 17 s; driven 76 m
```

If recoveries happened, the line also counts them by reason (`recoveries: 1x blocked, 2x no progress`). Show them with `grep -E "time so far|recover" /tmp/explorer.log`.

`PLAN` chooses among these kinds of target:

- LiDAR frontiers: edges between known and unknown space.
- Small isolated obstacles that the camera has not looked at yet. The LiDAR cannot tell rubble from a hazard block, so the camera must look at each one.
- Floor areas the camera has not seen yet.

**Beacons.**

- An EXIT beacon marks the start.
- Trail beacons are dropped at turns and every 2.5 m.
- Hazard beacons are dropped where the robot stands when the camera reports a new hazard.
- Each beacon links to the last beacon dropped or driven past (its LMB2 `next`). All the beacons therefore form a tree rooted at the exit, which the Executor follows (see `EXECUTOR.md`).
- `mission_recorder` saves every beacon as its signed 44-byte record to `~/writer_robot_ws/missions/latest.json`.

**Hazards → keep-out zones.** Every block the camera recognises becomes a 1.2 m disc that no path or goal may enter. A 1.2 m disc closes a corridor, so the robot takes the long way round. If the robot is already inside a zone, it may only move away from the hazard. Beacons for the hazard are dropped from where the robot stands. The beacon message carries the hazard's own position (`event_x`, `event_y`).

**Safety net (always on):**

- The robot stops if the LiDAR sees something closer than 0.30 m in front.
- It checks the 0.30 m circle it sweeps before any turn on the spot, and creeps away first if needed.
- It stops when `/scan` is more than 1.5 s old.

## 3. Maps

| file | what |
|---|---|
| `worlds/contaminated_zone.world` (default) | 20 × 14 m building: hall, 6 rooms, a loop of corridors around an island room, zigzag east wing, 6 rubble piles. Hazards: 2 purple radiation blocks (one in the 1.8 m north corridor: the LiDAR sees a gap beside it, the camera says no), 1 red fire block, 1 **yellow gas block** (new event type `gas`). |
| `worlds/retreat_test.world` | Small and quick. The purple block stands 3 m after a corner, so the camera only sees it after turning the corner, and the robot must turn back. |
| `worlds/contaminated_zone_small.world` | The original small arena. |

The layouts are generated by `tools/make_world.py`. Edit the lists there and re-run it.

## 4. Run it (Ubuntu VM)

```bash
~/writer_robot_ws/run_explorer.sh
```
Options: `retreat` (small turn-back map), `small`, `headless` (no Gazebo window, much faster in a VM), e.g. `~/writer_robot_ws/run_explorer.sh retreat headless`.

Follow what it is doing: `tail -f /tmp/explorer.log`. Health check of the running simulation: `bash ~/writer_robot_ws/check_sim.sh`. Option `ekf`: odometry from wheels + IMU instead of Gazebo's true motion (experiment only). RViz shows:

- the plan (blue line)
- the beacons: posts with a ball on top and their id, above the robot so they are never hidden, with lines to their way out
- the keep-out zones (coloured discs with labels)
- the goal (blue sphere; red while retreating, green going home)
- the state above the robot

## 5. Topics

| topic | type | |
|---|---|---|
| `/writer/hazard` | String JSON `{type, x, y, range, bearing, area, stamp}` | vision → explorer, every sighting (≤ 5 Hz per type) |
| `/writer/events` | String JSON | vision → beacon node, once per hazard (≤ 4 m) |
| `/cmd_vel` | Twist | explorer → robot, 10 Hz |
| `/writer/plan` | nav_msgs/Path | current path |
| `/writer/explorer_markers` | MarkerArray | keep-out zones, goal, state |
| `/writer/explorer_status` | String | e.g. `RETREAT: retreating: 6 waypoints left` |

## 6. Parameters you may want to change

Change them with `ros2 run writer_robot frontier_explorer --ros-args -p use_sim_time:=true -p NAME:=VALUE`, or in `run_explorer.sh`. `run_explorer.sh slow` starts the robot with slower speeds (`v_max` 0.5, `w_max` 1.2, `accel` 0.6, `look_w_max` 1.0, `spin_rate` 0.8).

| parameter | default | meaning |
|---|---|---|
| `keepout_radius` | 1.2 | m, no-go disc around a hazard |
| `v_max` | 0.70 | m/s top speed (`slow`: 0.50) |
| `w_max` / `look_w_max` | 1.6 / 1.6 | rad/s top turn rate while driving / during camera looks |
| `accel` | 1.0 | m/s² speed-up (braking is immediate) |
| `spin_rate` | 1.0 | rad/s of the first full turn |
| `look_max` | 3 | camera headings per look stop |
| `stuck_time` / `blocked_time` | 8 / 2 | s without progress / s told to move without moving, before a recovery |
| `time_report` | 60 | s between `time so far` lines (0 = none) |
| `retreat_min` / `retreat_clear` | 1.5 / 2.5 | m back along the trail / distance from the hazard to reach |
| `coverage` | true | false = LiDAR exploration only (faster, but may miss hazards) |
| `camera_range` | 3.2 | m. Must be below what the vision node really detects (`min_area_px` 800 ≈ 3.7 m). |
| `robot_radius` | 0.36 | m, clearance of the robot centre to obstacles |
| `home_on_finish` | true | return to the start at the end |
| `verbose` | false | log every planning decision |

Vision: `min_area_px` (800), `drop_range_m` (4.0). Beacons: `turn_threshold_deg` (35), `max_trail_gap_m` (2.5), `trail_dedup_m` (1.2). The beacon node does not drop a second beacon where one already lies, for example when the robot drives back along its trail.

## 7. Tested without ROS (also on Windows)

| command | what it checks | result |
|---|---|---|
| `python3 tools/sim2d.py --png run.png` | Runs the explorer in a 2D simulator of the same world: LiDAR, SLAM-like growing map, camera with field of view, blob size and line of sight, robot footprint and acceleration limits. Prints checks and draws the run. | big map: explored in about 7 simulated min, about 150 m, 4/4 hazards found, back home |
| `python3 tools/run_tests.py` | 21 runs: all 3 building maps and the Gafsa mine, several seeds, cameras that see hazards later (up to about 1.9 m only), LiDAR self-hits | 21/21 pass. Every run: no collision, never closer than 0.9 m to a hazard once seen, never standing still more than 20 s, exploration complete, back at start; in the mine also every resource graded and every search area searched |
| `python3 tools/test_nodes.py` | The real ROS nodes run on a tiny ROS stand-in (`tools/ros_stub`). Vision on rendered RGB-D frames: 3 colours, 3 distances, rubble, walls and beacons ignored, 16-bit depth. The explorer node drives the simulator through `/map` `/scan` TF `/cmd_vel`. The beacon node is checked for trail spacing, no duplicates, the gas beacon and the spawn command. | 70/70 pass (with the Executor and mine sensor checks) |

Windows: double-click the `.bat` files in `windows/`.

What these tests cannot prove is how Gazebo renders the colours and how slam_toolbox behaves in your VM. If the robot misses hazards in Gazebo, run `grep health /tmp/vision.log`: every 10 s it says how many colour and depth frames arrived and which colours were seen. Without depth images the node ranges the block from where it touches the floor. Then lower `min_area_px` and `camera_range` if needed.
