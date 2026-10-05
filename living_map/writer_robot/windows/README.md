# Writer robot tools on Windows (no ROS needed)

Install Python 3 from python.org (tick "Add python.exe to PATH"), then double-click:

| file | what it does |
|---|---|
| `install_python_deps.bat` | once: installs numpy, matplotlib, opencv-python |
| `sim_big_map.bat` | runs the explorer on the big map in the 2D simulator, opens the picture |
| `sim_retreat_test.bat` | same on the small "turn back" map |
| `sim_mission_big_map.bat` | the whole mission: the Writer drops beacons, then the Executor follows them to every hazard and back (picture of both runs) |
| `sim_mission_retreat_test.bat` | same on the small map |
| `run_all_tests.bat` | 18 Writer simulator runs, 21 whole missions (4 with a Command Post briefing), the ROS-node tests, the beacon-graph test and the robots + ONA test |

The 2D simulator runs the exact same code as the robots in Gazebo
(`writer_robot/explorer_core.py`, `executor_core.py`, `beacon_logic.py`), so you
can change parameters and try them here first, much faster than the VM.
