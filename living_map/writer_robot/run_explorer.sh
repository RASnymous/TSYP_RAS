#!/usr/bin/env bash
# =====================================================================
#  Writer Robot - FRONTIER EXPLORER demo (no Nav2)
#  sim + SLAM + frontier explorer + vision + beacons + RViz
#
#  The robot explores the whole map, turns back when the camera sees a
#  hazard block in its way (purple radiation / red fire / yellow gas),
#  checks every area with the camera, then returns to its start and stops.
#
#  Usage:  bash run_explorer.sh                 big map (default)
#          bash run_explorer.sh retreat         small map: watch it turn back
#          bash run_explorer.sh small           the original small arena
#          add "headless" (no Gazebo window, faster), e.g.
#          bash run_explorer.sh retreat headless
#          add "ona": the Outside Network Area - three gateways around the
#          building follow the Writer and every beacon it drops shows up on the
#          Command Post dashboard (COMMAND_POST=http://... to change where it is;
#          default http://10.0.2.2:3000 = the Windows host seen from the VM)
#          "mine": the GAFSA MINE - an old room-and-pillar phosphate panel.
#          The mine sensor suite (gas, radon, thermal camera, roof scanner,
#          wall probe, camera + XRF) replaces the colour-block vision, and the
#          beacons also mark phosphate seams and mineral finds, e.g.
#          bash run_explorer.sh mine headless ona
# =====================================================================
source /opt/ros/lyrical/setup.bash
source "$HOME/writer_robot_ws/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

PKG="$(ros2 pkg prefix writer_robot)/share/writer_robot"
PARAMS="$PKG/config"
WORLD="$PKG/worlds/contaminated_zone.world"
HEADLESS_ARG=""
ODOM_SRC="gz"
GPU=0
# `slow`: slower speeds (0.5 m/s), in case the robot is too fast for your computer
SLOW="-p v_max:=0.5 -p w_max:=1.2 -p accel:=0.6 -p look_w_max:=1.0 -p spin_rate:=0.8"
SPEED_ARGS=""
ONA=0
MINE=0
for a in "$@"; do
  case "$a" in
    headless) HEADLESS_ARG="headless:=true"; echo ">> headless mode" ;;
    gpu)      GPU=1; echo ">> using the graphics card (VirtualBox 3D acceleration must be ON)" ;;
    ekf)      ODOM_SRC="ekf"; echo ">> odometry: wheels + IMU (EKF)" ;;
    retreat)  WORLD="$PKG/worlds/retreat_test.world" ;;
    small)    WORLD="$PKG/worlds/contaminated_zone_small.world" ;;
    mine)     WORLD="$PKG/worlds/gafsa_mine.world"; MINE=1; echo ">> the Gafsa mine: mine sensors instead of the colour-block vision" ;;
    big)      ;;
    slow)     SPEED_ARGS="$SLOW"; echo ">> slow mode: 0.5 m/s, slower turns" ;;
    ona)      ONA=1; echo ">> with the Outside Network Area (3 gateways -> Command Post)" ;;
    *) echo "unknown option: $a (use: big | retreat | small | mine | headless | gpu | ekf | slow | ona)"; exit 1 ;;
  esac
done
echo ">> world: $(basename "$WORLD")"
[ "$GPU" = 1 ] && unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER

stop_all() {
  # patterns match the running programs only (not an editor with the same file open)
  local P='gz sim|gz-sim|ruby.*gz|gazebo_sim\.launch|ros_gz_bridge/parameter_bridge|async_slam_toolbox_node|lib/robot_state_publisher/|lib/robot_localization/ekf_node|lib/rviz2/rviz2|python3 -m ona --config'
  local N='(lib/writer_robot/|writer_robot )(frontier_explorer|beacon_drop_node|vision_event_detector|frame_translator_node|mission_recorder|executor_node|beacon_radio_sim|ona_link|mine_sensors)'
  pkill -INT -f "$N" 2>/dev/null ; pkill -INT -f "$P" 2>/dev/null ; sleep 1
  pkill -9 -f "$N" 2>/dev/null ; pkill -9 -f "$P" 2>/dev/null
  pkill -9 -f 'controller_server|planner_server|bt_navigator|smoother_server|behavior_server|lifecycle_manager' 2>/dev/null
}

trap 'echo; echo ">> stopping..."; stop_all; exit 0' INT TERM

echo "== [0/5] cleanup =="
stop_all ; sleep 3

echo "== [1/5] simulation =="
ros2 launch writer_robot gazebo_sim.launch.py world:="$WORLD" odom_source:=$ODOM_SRC $HEADLESS_ARG > /tmp/sim.log 2>&1 &
echo -n "   waiting for /scan "
OK=0; for i in $(seq 1 90); do ros2 topic info /scan 2>/dev/null | grep -q "Publisher count: [1-9]" && { echo " OK"; OK=1; break; }; printf '.'; sleep 2; done
[ "$OK" = 1 ] || echo " !! no /scan after 3 min - see /tmp/*sim.log (continuing anyway)"
sleep 3

SLAM_PARAMS="$PARAMS/slam_toolbox_params.yaml"
[ "$ODOM_SRC" = "ekf" ] && SLAM_PARAMS="$PARAMS/slam_toolbox_params_ekf.yaml"
echo -n "   waiting for the odometry TF odom -> base_footprint "
OK=0; for i in $(seq 1 20); do timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -q "Translation" && { echo " OK"; OK=1; break; }; printf '.'; sleep 1; done
[ "$OK" = 1 ] || echo " !! no odom -> base_footprint TF: the robot cannot localise (see /tmp/*sim.log, run: bash ~/writer_robot_ws/check_sim.sh)"

echo "== [2/5] SLAM (start -> configure -> activate) =="
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "$SLAM_PARAMS" -p use_sim_time:=true > /tmp/slam.log 2>&1 &
echo -n "   waiting for slam node "
for i in $(seq 1 45); do ros2 lifecycle get /slam_toolbox >/dev/null 2>&1 && { echo " OK"; break; }; printf '.'; sleep 2; done
ros2 lifecycle set /slam_toolbox configure ; sleep 3
ros2 lifecycle set /slam_toolbox activate ; sleep 2
ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q '^active' || { sleep 2; ros2 lifecycle set /slam_toolbox activate; sleep 2; }
echo -n "   waiting for map->odom "
OK=0; for i in $(seq 1 40); do timeout 8 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "Translation" && { echo " OK"; OK=1; break; }; printf '.'; sleep 1; done
[ "$OK" = 1 ] || echo " !! no map->odom yet - is SLAM active? (ros2 lifecycle get /slam_toolbox)"

ANCHOR_ARGS=""
if [ "$MINE" = 1 ]; then
  SITE="$PKG/worlds/gafsa_mine.json"
  ANCHOR_ARGS=$(python3 -c 'import json,sys; a=json.load(open(sys.argv[1]))["anchor"]; print("-p anchor_lat:=%s -p anchor_lon:=%s -p map_yaw_deg:=%s" % (a["lat"], a["lon"], a["map_yaw_deg"]))' "$SITE")
  echo "== [3/5] mine sensors + beacons + mission recorder + gps translator =="
  ros2 run writer_robot mine_sensors --ros-args -p use_sim_time:=true -p site_file:="$SITE" > /tmp/mine_sensors.log 2>&1 &
else
  echo "== [3/5] vision + beacons + mission recorder + gps translator =="
  ros2 run writer_robot vision_event_detector --ros-args -p use_sim_time:=true > /tmp/vision.log 2>&1 &
fi
ros2 run writer_robot beacon_drop_node --ros-args -p use_sim_time:=true > /tmp/beacon.log 2>&1 &
ros2 run writer_robot mission_recorder --ros-args -p use_sim_time:=true -p world:="$WORLD" $ANCHOR_ARGS > /tmp/mission.log 2>&1 &
ros2 run writer_robot frame_translator_node ${ANCHOR_ARGS:+--ros-args $ANCHOR_ARGS} > /tmp/frame.log 2>&1 &

if [ "$ONA" = 1 ]; then
  echo "== [3b/5] Outside Network Area: 3 gateways + ONA computer -> Command Post =="
  ONA_CFG="$PKG/config/ona_gazebo_big.json"
  case "$WORLD" in *retreat*) ONA_CFG="$PKG/config/ona_gazebo_retreat.json" ;; *gafsa_mine*) ONA_CFG="$PKG/config/ona_gazebo_mine.json" ;; esac
  CP="${COMMAND_POST:-http://10.0.2.2:3000}"
  if [ -z "$ONA_HOST" ]; then
    ONA_DIR="$HOME/writer_robot_ws/src/writer_robot/ona"
    [ -d "$ONA_DIR/ona" ] || ONA_DIR="$HOME/living_map/ona"
    [ -d "$ONA_DIR/ona" ] || { echo "!! the ONA is missing: run  bash ~/living_map/writer_robot/install.sh  again"; exit 1; }
    ( cd "$ONA_DIR" && exec python3 -m ona --config "$ONA_CFG" \
        --udp 127.0.0.1:47100 --command-post "$CP" --db /tmp/ona_queue.sqlite3 --fresh ) > /tmp/ona.log 2>&1 &
    echo "   ONA running here, posting to the Command Post at $CP (log /tmp/ona.log)"
  fi
  ros2 run writer_robot ona_link --ros-args -p use_sim_time:=true -p robot:=writer -p world_file:="$WORLD" \
    -p ona_config:="$ONA_CFG" -p ona_host:="${ONA_HOST:-127.0.0.1}" > /tmp/ona_link.log 2>&1 &
fi

echo "== [4/5] frontier explorer (drives the robot) =="
ros2 run writer_robot frontier_explorer --ros-args -p use_sim_time:=true $SPEED_ARGS > /tmp/explorer.log 2>&1 &

echo "== [5/5] RViz =="
rviz2 -d "$PKG/config/writer.rviz" --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1 &
sleep 2

echo
echo "==================================================================="
echo " RUNNING. The robot starts exploring a few seconds after SLAM is up."
echo "==================================================================="
echo " RViz: map grows, blue line = planned path, coloured discs = hazard"
echo "   keep-out zones, dots = dropped beacons (blue trail, magenta"
echo "   radiation, orange fire, yellow gas, green = exit)."
echo " Every beacon is saved to ~/writer_robot_ws/missions/latest.json:"
echo "   when the Writer is DONE, run  ~/writer_robot_ws/run_executor.sh"
echo " What the robot is doing:  tail -f /tmp/explorer.log"
echo "   (RETREAT = turning back the way it came, LOOK = camera check,"
echo "    EXPLORATION COMPLETE -> returns to the start and stops)"
echo " Where the time goes (a line every simulated minute):"
echo "   grep 'time so far' /tmp/explorer.log"
echo " Live status:  ros2 topic echo /writer/explorer_status"
[ "$ONA" = 1 ] && echo " ONA: dashboard at $CP - beacons appear as the Writer drops them. ONA log: tail -f /tmp/ona.log"
[ "$MINE" = 1 ] && echo " Mine sensors (air, sightings every 10 s): tail -f /tmp/mine_sensors.log    air now: ros2 topic echo /writer/air"
echo " Logs: /tmp/sim.log /tmp/slam.log /tmp/explorer.log /tmp/$([ "$MINE" = 1 ] && echo mine_sensors || echo vision).log /tmp/beacon.log /tmp/mission.log"
echo " STOP: press Ctrl-C here."
echo "==================================================================="

wait
