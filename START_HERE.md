# Living Map: start here

**TSYP14 · IEEE RAS × AESS · The Living Map**

A **Writer** robot explores a contaminated building, maps it and drops small LoRa beacons. Each beacon carries one signed 44-byte record (exit, trail, radiation, fire, gas, victim, and the next beacon on the way out), and the beacons copy each other's records. Outside, **three gateways** listen. The **Outside Network Area (ONA)** accepts a record only when two gateways heard the identical signed bytes, locates the robots from the gateways' distances (like GPS), and forwards everything to the **Command Post** dashboard over LTE, or satellite when LTE fails. The **Executor** robot, briefed from the dashboard, then follows the beacons to treat the hazards and comes back out.

Everything runs in simulation (Gazebo for the robots, simulators for the radio network and the gateways), and every part also runs on real ESP32 boards.

The same chain also runs **underground**, in a simulated room-and-pillar panel of the **Gafsa phosphate mine** (Métlaoui): bad air, radon, fire, unstable roof, victims and *searched, nobody here* areas, and phosphate grades (gold and gemstones as demonstration finds). See `docs/3_Gafsa_Mine_Case.pdf` and part N of the Guide.

## What is in this folder, in reading order

| # | file or folder | what it is |
|---|---|---|
| 1 | `START_HERE.md` | this page |
| 2 | `docs/1_Living_Map_Guide.pdf` | **install and run everything**: every command, in order, from an empty computer to the full demo |
| 3 | `COMMANDS.txt` | the same commands, one per line, under the same section numbers: copy commands from here, not from the PDF |
| 4 | `docs/2_Living_Map_Architecture.pdf` | how everything works: robots, beacons, protocol, gateways, ONA, dashboard, tests |
| 4b | `docs/3_Gafsa_Mine_Case.pdf` | the Gafsa mine case: the dangers underground, the sensors and robot specification, the radio, what changes on the dashboard |
| 5 | `writer_robot/` | ROS 2 package of both robots (Writer and Executor), Gazebo worlds, `install.sh`, run scripts, 2D simulators and tests, Windows launchers |
| 6 | `beacon_net/` | the LMB2 beacon protocol (C), ESP32 firmware (beacon and gateway), network simulator `bpsim`, `bptool`, Python mirror, one-gateway bridge |
| 7 | `ona/` | the Outside Network Area: three gateways, the 2-of-3 vote, robots located, GPS translation, LTE / satellite queue, briefings; zone simulator; tests; the GALERIA team's original code |
| 8 | `command_post/` | the Command Post dashboard (Node.js) and its scripted demo |

Each folder also has its own `README.md`.

## From scratch: the short path

The Guide has every detail (part A and B). In short:

**Ubuntu VM** (ROS 2 Lyrical and Gazebo Jetty already installed). Copy `living_map.zip` to `~/Downloads`, then one line at a time:

```
sudo apt update
sudo apt install -y ros-lyrical-ros-gz ros-lyrical-robot-localization ros-lyrical-slam-toolbox ros-lyrical-xacro ros-lyrical-robot-state-publisher ros-lyrical-joint-state-publisher ros-lyrical-cv-bridge python3-opencv ros-lyrical-teleop-twist-keyboard ros-lyrical-tf2-tools
sudo apt install -y build-essential unzip curl nodejs npm python3-numpy python3-serial python3-matplotlib
grep -q 'opt/ros/lyrical' ~/.bashrc || echo 'source /opt/ros/lyrical/setup.bash' >> ~/.bashrc
cd ~ && rm -rf ~/living_map && unzip -o ~/Downloads/living_map.zip
bash ~/living_map/writer_robot/install.sh
grep -q 'writer_robot_ws/install' ~/.bashrc || echo 'source ~/writer_robot_ws/install/setup.bash' >> ~/.bashrc
make -C ~/living_map/beacon_net/host
```

`install.sh` must end with **Summary: 1 package finished**. Then open a new terminal.

**Windows.** Install Node.js LTS and Python 3 (tick *Add python.exe to PATH*). Unzip `living_map.zip` into `C:\tsyp\` so you get `C:\tsyp\living_map\`, then double-click once:

```
C:\tsyp\living_map\writer_robot\windows\install_python_deps.bat
C:\tsyp\living_map\ona\windows\install_deps.bat
```

## The first run (about 10 minutes)

1. Windows: `C:\tsyp\living_map\beacon_net\windows\run_dashboard.bat`, then open http://localhost:3000
2. VM: `~/writer_robot_ws/run_explorer.sh retreat headless ona`: the Writer explores the small map; its beacons appear on the dashboard, confirmed by the three gateways.
3. When its log says DONE (`tail -f /tmp/explorer.log`), press Ctrl-C in its terminal.
4. VM: `~/writer_robot_ws/run_executor.sh headless ona`: the Executor waits at the exit.
5. Dashboard: **Select on map**, pick the hazards, **Dispatch mission**. The Executor acknowledges, treats them in your order and returns to the exit.
6. VM: `bash ~/writer_robot_ws/stop_demo.sh`

**The Gafsa mine.** VM: `~/writer_robot_ws/run_explorer.sh mine headless ona` (the dashboard switches to the mine plan by itself); after DONE and Ctrl-C, `~/writer_robot_ws/run_executor.sh headless ona` (the Executor reads the Writer's mission, so it stays in the mine; `run_executor.sh mine headless ona` uses the ready-made mine mission instead). No VM: `beacon_net\windows\run_dashboard_mine_demo.bat` (the whole mine story on the dashboard) and `writer_robot\windows\sim_mission_mine.bat` (both robots in the mine, in 2D).

No VM at hand: `run_dashboard_demo.bat` (the dashboard tells the whole story on its own), `writer_robot\windows\sim_mission_big_map.bat` (both robots in 2D), and `ona\windows\run_ona.bat` + `ona\windows\simulate_faults.bat` (the three gateways with a lying gateway, a forged victim, a lost beacon and a SLAM drift).

## Check that everything works

| where | command | a good run ends with |
|---|---|---|
| Windows | `writer_robot\windows\run_all_tests.bat` | 21/21 Writer runs, 26/26 missions, 70/70 node checks, 21/21 graph checks, 18/18 robot + ONA checks |
| Windows | `ona\windows\run_tests.bat` | 70/70, 20/20, 3/3 |
| VM | `make -C ~/living_map/beacon_net/host test` and `… matrix` | all cross-checks OK, 42/42 PASS |

## The Guide, part by part

| part | what |
|---|---|
| A, B | set up the VM and Windows once; install the project |
| C | tests and 2D simulators (no Gazebo) |
| D | the Command Post dashboard |
| E | the Writer robot |
| F | the Executor robot, briefed from the dashboard |
| G | the Outside Network Area: three gateways, vote, robots located, LTE / satellite |
| H | the beacon network simulator and frame tool |
| I | one gateway without the ONA |
| J | the full demo, in the order of the presentation |
| K | real hardware: ESP32 beacons and gateways |
| L, M | command reference, troubleshooting, logs |
| N | the Gafsa mine: without ROS, in Gazebo, changing the mine, troubleshooting |
