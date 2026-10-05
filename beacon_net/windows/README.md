# Working on Windows

Everything except the robot simulation runs natively on Windows: the dashboard, the beacon-network simulator, the tests and the ESP32 firmware. The Ubuntu VM only runs ROS 2 and Gazebo.

| Task | Where | How |
|---|---|---|
| Dashboard (Command Post) | **Windows** | `run_dashboard.bat` → http://localhost:3000 |
| Dashboard with a scripted fake mission | **Windows** | `run_dashboard_demo.bat` |
| Beacon network simulated live on the dashboard | **Windows** | `simulate_to_dashboard.bat` (start the dashboard first) |
| Simulation report + 13 checks | **Windows** | `run_simulation.bat` |
| The brief's record as a 44-byte frame | **Windows** | `example_frame.bat` |
| C ⇄ Python protocol tests | **Windows** | `run_tests.bat` |
| Editing all the code | **Windows** | VS Code (or any editor) on the unzipped folder |
| ESP32 firmware: build, flash, serial monitor | **Windows** | Arduino IDE 2, see below. USB ports appear as COM3, COM4… with no VirtualBox USB passthrough needed |
| Real gateway board → dashboard | **Windows** | `real_gateway_to_dashboard.bat` |
| The Outside Network Area (3 gateways, vote, robot positions, satellite, briefings) | **Windows** | `..\..\ona\windows\run_ona.bat`, then `simulate_zone.bat` or `simulate_faults.bat` (same folder) |
| This network simulator with 3 gateways through the ONA | **Windows** | `..\..\ona\windows\bpsim_to_ona.bat` |
| Robot simulation (ROS 2, Gazebo, SLAM, RViz) | **VM** | `~/writer_robot_ws/run_explorer.sh headless ona` |
| `beacon_record_node.py` (needs ROS) | **VM** | posts to the Windows dashboard at `http://10.0.2.2:3000` |
| Rebuilding after C changes | VM | `make -C host` (Linux) and `make -C host windows` (these .exe) |

## Install once on Windows

1. **Node.js LTS**: https://nodejs.org (for the dashboard).
2. **Python 3**: https://www.python.org/downloads/. Tick *Add python.exe to PATH* during setup.
3. Optional: **VS Code** for editing.
4. For hardware: **Arduino IDE 2**, plus the USB driver of your board (CP210x or CH340 for most TTGO / ESP32 boards).

## First run

1. Unzip `living_map.zip` to a short path, for example `C:\tsyp\` (you get `C:\tsyp\living_map\`).
2. Windows may warn about `bpsim.exe` / `bptool.exe` because they are unsigned. Click *More info → Run anyway*.
3. Double-click `beacon_net\windows\run_dashboard.bat`, then open http://localhost:3000.
4. Double-click `beacon_net\windows\simulate_to_dashboard.bat`. Beacons appear on the map at 20× real time.

The same bridge from a terminal (PowerShell or cmd), with simulator options:

```
py -3 ..\python\beaconnet\gateway_bridge.py --sim="--speed 20 --beacons 30 --rubble 2.5" --server http://localhost:3000
```

## Firmware on Windows (Arduino IDE 2)

1. *File → Preferences → Additional boards manager URLs*: add `https://espressif.github.io/arduino-esp32/package_esp32_index.json` (from the [arduino-esp32 docs](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)). Then *Boards Manager → esp32 by Espressif*.
2. *Library Manager → "LoRa" by Sandeep Mistry*.
3. Copy the folder `beacon_net\lib\BeaconProto` to `Documents\Arduino\libraries\BeaconProto`.
4. Open `beacon_net\firmware\gateway_node\gateway_node.ino` (or `beacon_node`), pick your board and COM port, and upload.
5. *Serial Monitor* at 115200 baud: type `KEY demo`, `TIME 1790000000`, `STATUS`.

## Make the VM lighter

* Run the robot with no Gazebo window: `~/writer_robot_ws/run_explorer.sh headless`. RViz still shows the map.
* With the VM powered off, open *Settings → System* and give it about half your CPU cores and 6–8 GB of RAM.
* Share this folder with the VM instead of copying files through Downloads. In VirtualBox: *Devices → Shared Folders*, add `C:\tsyp`, tick *Auto-mount*. In the VM, run `sudo usermod -aG vboxsf $USER` once, then reboot. The folder appears at `/media/sf_tsyp`. Keep the ROS workspace (`~/writer_robot_ws`) inside the VM, because builds on a shared folder are slow.
