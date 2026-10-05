# Living Map: the Outside Network Area (ONA)

The ONA is the only bridge between the zone and the outside world. It has three LoRa gateways around the building, and does five jobs:

1. **2-of-3 vote.** Every message the gateways pass on is re-checked with the mission key. It only counts when at least two gateways heard exactly the same signed bytes. A gateway that lies or passes on bad frames is named and then ignored.
2. **Robot position from three spheres.** Each gateway measures its distance to the robot. The ONA intersects the three spheres (as GPS does with satellites), checks the result (RAIM, HDOP) and fuses it with the robot's own movement in a Kalman filter. It then compares that with where the robot *says* it is, which catches a SLAM that has slipped.
3. **Frame translation.** Private map coordinates become WGS84 GPS through Umeyama calibration, and the calibration is re-checked during the mission from the gateways' fixes.
4. **Store and forward.** Messages go to the Command Post over LTE, or over satellite when LTE is down (urgent messages first). Nothing is lost while both links are down: the queue is kept on disk in SQLite.
5. **Mission briefing.** A mission dispatched on the dashboard becomes signed LoRa frames, and the gateways transmit it to the Executor. The Executor's acknowledgement comes back through the gateways.

The GALERIA team's code is the base of this folder: `trans.py` (frame translation, Umeyama), `ekf.py` (the Kalman filter) and `ona.py` (the persistent queue, the LTE/satellite link and the audit log). The originals are kept unchanged in `galeria_original/`. **[ONA.md](ONA.md)** explains the design, every change to the original code and why, and the test results.

## Quick start (Windows or the VM)

```bat
windows\install_deps.bat          (once: numpy, pyserial)
windows\run_tests.bat             (70 + 20 + 3 checks, each block must end with "checks passed")
```

Live demo, with the Command Post dashboard already running on `http://localhost:3000`:

```bat
windows\run_ona.bat               (terminal 1: the ONA)
windows\simulate_zone.bat         (terminal 2: 3 gateways hear a mission)
```

On the dashboard, select 2 or 3 hazard pins and click **Dispatch mission**. The simulated Executor waits at the exit for this briefing (120 s of mission time, 60 s at speed 2), acknowledges it and treats those targets in that order. **Call the Executor back** sends a signed ABORT; untick "back to the exit afterwards" to make it wait at its last target for the next briefing. Then run `windows\simulate_faults.bat` to see the ONA catch these faults:

- a lying gateway;
- a forged victim;
- a crushed beacon;
- a robot whose SLAM slips.

Linux / the VM: `./run_ona.sh` and `./simulate_zone.sh`, with the same options.

**The Gafsa mine:** `windows\run_ona_mine.bat` (terminal 1) and `windows\simulate_mine.bat` (terminal 2). The gateways sit at the portal and on the cable backbone underground (`ona_config_gazebo_mine.json`); the dashboard draws the mine plan, and the ONA adds RESOURCE_MARKED and AREA_SEARCHED events. Resources are forwarded last and never over the satellite. In the VM: `python3 -m ona --config ona_config_gazebo_mine.json --udp 0.0.0.0:47100 --command-post http://10.0.2.2:3000` and `./simulate_zone.sh --config ona_config_gazebo_mine.json --mission ../writer_robot/missions/demo_mine.json`.

More launchers:

- `windows\bpsim_to_ona.bat`: the beacon network simulator (the beacons' own C code) with **three** gateways, through the vote, to the dashboard.
- **With the Gazebo robots** (the VM): add `ona` to the robot scripts, e.g. `~/writer_robot_ws/run_executor.sh demo headless ona`. `writer_robot/install.sh` copies this folder into the robot workspace, and it runs in the VM, posting to the dashboard on Windows (`http://10.0.2.2:3000`). To run the ONA on Windows instead: `windows\run_ona_for_gazebo.bat`, and in the VM `ONA_HOST=10.0.2.2 ~/writer_robot_ws/run_executor.sh demo headless ona`.

## Files

| file | what |
|---|---|
| `ona/lmb2.py` | LMB2 frames: RECORD (beacons), and the new ROBOT (position report) and MISSION (briefing) types; signature checks |
| `ona/quorum.py` | the 2-of-3 vote, gateway scores, quarantine |
| `ona/geodesy.py` | **from trans.py**: WGS84 / ECEF / ENU, Umeyama, `LocalToGpsCalibrator` |
| `ona/kalman.py` | **from ekf.py**: `PositionEKF`, plus `RangeEKF` (one correction per gateway range) |
| `ona/multilateration.py` | the three spheres: Gauss-Newton, HDOP, RAIM |
| `ona/tracker.py` | per robot: grouping one ping's ranges, NLOS correction, EKF, reported-vs-measured check, calibration check |
| `ona/store.py` | **from ona.py**: SQLite queue + audit log, plus priorities and coalescing |
| `ona/uplink.py` | **from ona.py**: LTE first, satellite backup; HTTP to the Command Post |
| `ona/core.py` | the ONA logic (no I/O) |
| `ona/server.py` | `python -m ona`: gateway inputs (USB serial, UDP, stdin), uplink thread, mission polling |
| `ona/zonesim.py` | `python -m ona.zonesim`: the zone as three gateways hear it (walls, ranging, faults) |
| `ona_config.json` | gateways' positions, map anchor, mission key, vote quorum (built-in building of the zone simulator) |
| `ona_config_gazebo_big.json`, `ona_config_gazebo_retreat.json` | the same for the two Gazebo worlds (the robots' `ona_link` node reads the same files) |
| `ona_config_bpsim.json` | the same for `bpsim --gateways 3` |
| `tests/` | `test_ona.py` (units, end to end, bpsim), `test_zone.py` (whole missions with faults), `test_vs_beaconnet.py` (byte-identical to the beacons' codec) |
