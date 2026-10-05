#!/usr/bin/env bash
# =====================================================================
#  Living Map - install the Writer + Executor robots into ~/writer_robot_ws
#  and build them. ONE command:
#      bash ~/living_map/writer_robot/install.sh
#  It also copies the Outside Network Area (../ona) into the package, so
#  that the "ona" option of the run scripts works in the VM.
#  A package already in the workspace is moved to ~/writer_robot_backups/
#  (never deleted).
# =====================================================================
set -e
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/writer_robot_ws"
DEST="$WS/src/writer_robot"
mkdir -p "$WS/src" "$HOME/writer_robot_backups"

if [ "$SRC" != "$DEST" ]; then
  if [ -d "$DEST" ]; then
    BK="$HOME/writer_robot_backups/writer_robot_$(date +%Y%m%d_%H%M%S)"
    mv "$DEST" "$BK"
    echo ">> previous package saved to $BK"
  fi
  cp -r "$SRC" "$DEST"
fi
# the Outside Network Area, next to this folder in the delivery (living_map/ona)
if [ -d "$SRC/../ona/ona" ]; then
  rm -rf "$DEST/ona"
  cp -r "$SRC/../ona" "$DEST/ona"
  echo ">> Outside Network Area copied to $DEST/ona"
elif [ ! -d "$DEST/ona/ona" ]; then
  echo "!! ../ona not found next to writer_robot: the 'ona' option of the run scripts will not work"
fi
find "$DEST" -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null || true
find "$DEST" -name "*.sqlite3" ! -path "*tests/data*" -delete 2>/dev/null || true
# launch scripts in the workspace (an older copy is kept in the backups)
for f in run_explorer.sh run_executor.sh stop_demo.sh check_sim.sh; do
  if [ -f "$WS/$f" ] && ! cmp -s "$WS/$f" "$DEST/$f"; then
    cp "$WS/$f" "$HOME/writer_robot_backups/${f%.sh}_$(date +%Y%m%d_%H%M%S).sh"
  fi
  cp "$DEST/$f" "$WS/$f"
  chmod +x "$WS/$f" "$DEST/$f"
done
mkdir -p "$WS/missions"

echo ">> building..."
source /opt/ros/lyrical/setup.bash
cd "$WS"
rm -rf build/writer_robot install/writer_robot
colcon build --packages-select writer_robot --symlink-install
echo
echo "==================================================================="
echo " Installed. Open a NEW terminal, then:"
echo " 1) WRITER explores and drops beacons (saved to ~/writer_robot_ws/missions/):"
echo "      ~/writer_robot_ws/run_explorer.sh headless ona   big map"
echo "      ~/writer_robot_ws/run_explorer.sh retreat headless ona   small map"
echo " 2) EXECUTOR follows the beacons, briefed from the dashboard:"
echo "      ~/writer_robot_ws/run_executor.sh headless ona   the mission the Writer recorded"
echo "      ~/writer_robot_ws/run_executor.sh demo headless ona   ready-made mission"
echo " 'ona' = three gateways around the building + the Command Post dashboard"
echo " (http://10.0.2.2:3000 = Windows). Without 'ona' the robots run on their own."
echo " Without 'headless' you also get the Gazebo window (slower)."
echo " STOP everything:  bash ~/writer_robot_ws/stop_demo.sh"
echo "==================================================================="
