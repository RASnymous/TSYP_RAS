#!/usr/bin/env bash
# =====================================================================
#  Executor Robot - follows the Writer's beacons and completes the mission
#  sim + SLAM + LoRa receiver + camera + executor + RViz
#
#  The Executor enters at the exit knowing only the beacons' 44-byte LoRa
#  frames. It drives beacon by beacon to every hazard, treats each one from
#  1.6 m (a green disc appears on the floor), then follows the "next" hops
#  back to the exit: MISSION COMPLETE.
#
#  Usage:  bash run_executor.sh                  mission the Writer just recorded
#                                                (~/writer_robot_ws/missions/latest.json)
#          bash run_executor.sh demo             ready-made mission, big map
#          bash run_executor.sh demo retreat     ready-made mission, small map
#          bash run_executor.sh FILE.json        any mission file
#          add "headless" for no Gazebo window (faster), e.g.
#          bash run_executor.sh demo headless
#          add "ona": the Outside Network Area too - three gateways around
#          the building measure where the robot is, every beacon goes to the
#          Command Post dashboard, and the Executor WAITS for a briefing from it
#          (pick targets on the dashboard, "Dispatch mission"; after 60 s of
#          simulated time without one it goes for every hazard), e.g.
#          bash run_executor.sh demo headless ona
#          Dashboard elsewhere:  COMMAND_POST=http://192.168.1.20:3000 bash run_executor.sh demo ona
#          (default http://10.0.2.2:3000 = the Windows host seen from a VirtualBox VM)
#          ONA on another computer:  ONA_HOST=10.0.2.2 bash run_executor.sh demo ona
#          "mine": the ready-made GAFSA MINE mission. A mission the Writer
#          recorded in the mine is recognised by itself: the Executor then uses
#          the mine sensors and also treats unstable roofs (props), e.g.
#          bash run_executor.sh mine headless ona
# =====================================================================
source /opt/ros/lyrical/setup.bash
source "$HOME/writer_robot_ws/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

PKG="$(ros2 pkg prefix writer_robot)/share/writer_robot"
PARAMS="$PKG/config"
MISSION="$HOME/writer_robot_ws/missions/latest.json"
HEADLESS_ARG=""
ODOM_SRC="gz"
GPU=0
DEMO=""
ONA=0
MINE=0
# `slow`: slower speeds (0.5 m/s), in case the robot is too fast for your computer
SLOW="-p v_max:=0.5 -p w_max:=1.2 -p accel:=0.6 -p look_w_max:=1.0 -p spin_rate:=0.8"
SPEED_ARGS=""
for a in "$@"; do
  case "$a" in
    headless) HEADLESS_ARG="headless:=true"; echo ">> headless mode" ;;
    gpu)      GPU=1; echo ">> using the graphics card (VirtualBox 3D acceleration must be ON)" ;;
    ekf)      ODOM_SRC="ekf"; echo ">> odometry: wheels + IMU (EKF)" ;;
    demo)     [ -z "$DEMO" ] && DEMO="big" ;;
    retreat)  DEMO="retreat_test" ;;
    big)      DEMO="big" ;;
    mine)     DEMO="mine" ;;
    *.json)   MISSION="$a" ;;
    slow)     SPEED_ARGS="$SLOW"; echo ">> slow mode: 0.5 m/s, slower turns" ;;
    ona)      ONA=1; echo ">> with the Outside Network Area (3 gateways, Command Post briefing)" ;;
    *) echo "unknown option: $a (use: demo | retreat | mine | FILE.json | headless | gpu | ekf | slow | ona)"; exit 1 ;;
  esac
done
[ -n "$DEMO" ] && MISSION="$PKG/missions/demo_${DEMO}.json"
[ "$GPU" = 1 ] && unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
if [ ! -f "$MISSION" ]; then
  echo "!! no mission file: $MISSION"
  echo "   Run the Writer first (~/writer_robot_ws/run_explorer.sh) until it is DONE,"
  echo "   or try a ready-made mission:  bash run_executor.sh demo"
  exit 1
fi
echo ">> mission: $MISSION"
MWORLD=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('world') or '')" "$MISSION" 2>/dev/null)
case "$MWORLD" in *gafsa_mine*) MINE=1; echo ">> the Gafsa mine: mine sensors, unstable roofs treated too" ;; esac

WORLD=/tmp/executor_world.world
ros2 run writer_robot mission_world --mission "$MISSION" --out "$WORLD" || exit 1

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

echo "== [1/5] simulation (Executor robot, beacons in place) =="
ros2 launch writer_robot gazebo_sim.launch.py world:="$WORLD" robot:=executor odom_source:=$ODOM_SRC $HEADLESS_ARG mine:=$([ "$MINE" = 1 ] && echo true || echo false) > /tmp/exec_sim.log 2>&1 &
echo -n "   waiting for /scan "
OK=0; for i in $(seq 1 90); do ros2 topic info /scan 2>/dev/null | grep -q "Publisher count: [1-9]" && { echo " OK"; OK=1; break; }; printf '.'; sleep 2; done
[ "$OK" = 1 ] || echo " !! no /scan after 3 min - see /tmp/*sim.log (continuing anyway)"
sleep 3

SLAM_PARAMS="$PARAMS/slam_toolbox_params.yaml"
[ "$ODOM_SRC" = "ekf" ] && SLAM_PARAMS="$PARAMS/slam_toolbox_params_ekf.yaml"
echo -n "   waiting for the odometry TF odom -> base_footprint "
OK=0; for i in $(seq 1 20); do timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -q "Translation" && { echo " OK"; OK=1; break; }; printf '.'; sleep 1; done
[ "$OK" = 1 ] || echo " !! no odom -> base_footprint TF: the robot cannot localise (see /tmp/*sim.log, run: bash ~/writer_robot_ws/check_sim.sh)"

echo "== [2/5] SLAM (the Executor maps the zone for itself) =="
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "$SLAM_PARAMS" -p use_sim_time:=true > /tmp/exec_slam.log 2>&1 &
echo -n "   waiting for slam node "
for i in $(seq 1 45); do ros2 lifecycle get /slam_toolbox >/dev/null 2>&1 && { echo " OK"; break; }; printf '.'; sleep 2; done
ros2 lifecycle set /slam_toolbox configure ; sleep 3
ros2 lifecycle set /slam_toolbox activate ; sleep 2
ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q '^active' || { sleep 2; ros2 lifecycle set /slam_toolbox activate; sleep 2; }
echo -n "   waiting for map->odom "
OK=0; for i in $(seq 1 40); do timeout 8 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "Translation" && { echo " OK"; OK=1; break; }; printf '.'; sleep 1; done
[ "$OK" = 1 ] || echo " !! no map->odom yet - is SLAM active? (ros2 lifecycle get /slam_toolbox)"

EXEC_ARGS=""
if [ "$MINE" = 1 ]; then
  echo "== [3/5] LoRa receiver + mine sensors =="
  ros2 run writer_robot beacon_radio_sim --ros-args -p use_sim_time:=true -p mission_file:="$MISSION" > /tmp/exec_radio.log 2>&1 &
  ros2 run writer_robot mine_sensors --ros-args -p use_sim_time:=true -p site_file:="$PKG/worlds/gafsa_mine.json" \
    -r /writer/hazard:=/executor/hazard -r /writer/events:=/executor/vision_events -r /writer/air:=/executor/air \
    > /tmp/exec_mine_sensors.log 2>&1 &
  EXEC_ARGS="-p target_kinds:=RADIATION,THERMAL,GAS,VICTIM,STRUCTURAL -p site:=mine -p priority_kinds:=VICTIM"
else
  echo "== [3/5] LoRa receiver + camera =="
  ros2 run writer_robot beacon_radio_sim --ros-args -p use_sim_time:=true -p mission_file:="$MISSION" > /tmp/exec_radio.log 2>&1 &
  ros2 run writer_robot vision_event_detector --ros-args -p use_sim_time:=true \
    -r /writer/hazard:=/executor/hazard -r /writer/events:=/executor/vision_events \
    -r /writer/vision_target:=/executor/vision_target > /tmp/exec_vision.log 2>&1 &
fi

if [ "$ONA" = 1 ]; then
  echo "== [3b/5] Outside Network Area: 3 gateways + ONA computer -> Command Post =="
  ONA_CFG="$PKG/config/ona_gazebo_big.json"
  case "$MWORLD" in *retreat*) ONA_CFG="$PKG/config/ona_gazebo_retreat.json" ;; *gafsa_mine*) ONA_CFG="$PKG/config/ona_gazebo_mine.json" ;; esac
  CP="${COMMAND_POST:-http://10.0.2.2:3000}"
  if [ -z "$ONA_HOST" ]; then
    ONA_DIR="$HOME/writer_robot_ws/src/writer_robot/ona"
    [ -d "$ONA_DIR/ona" ] || ONA_DIR="$HOME/living_map/ona"
    [ -d "$ONA_DIR/ona" ] || { echo "!! the ONA is missing: run  bash ~/living_map/writer_robot/install.sh  again"; exit 1; }
    ( cd "$ONA_DIR" && exec python3 -m ona --config "$ONA_CFG" --udp 127.0.0.1:47100 --command-post "$CP" \
        --db /tmp/ona_queue.sqlite3 --fresh ) > /tmp/ona.log 2>&1 &
    echo "   ONA running here, posting to the Command Post at $CP (log /tmp/ona.log)"
  else
    echo "   ONA expected at $ONA_HOST:47100"
  fi
  ros2 run writer_robot ona_link --ros-args -p use_sim_time:=true -p robot:=executor -p world_file:="$WORLD" \
    -p mission_file:="$MISSION" -p ona_config:="$ONA_CFG" -p ona_host:="${ONA_HOST:-127.0.0.1}" > /tmp/ona_link.log 2>&1 &
  EXEC_ARGS="$EXEC_ARGS -p wait_briefing:=${WAIT_BRIEFING:-60.0}"
fi

echo "== [4/5] Executor (drives the robot) =="
ros2 run writer_robot executor_node --ros-args -p use_sim_time:=true $SPEED_ARGS $EXEC_ARGS > /tmp/executor.log 2>&1 &

echo "== [5/5] RViz =="
rviz2 -d "$PKG/config/executor.rviz" --ros-args -p use_sim_time:=true > /tmp/exec_rviz.log 2>&1 &
sleep 2

echo
echo "==================================================================="
echo " RUNNING. The Executor starts a few seconds after SLAM is up."
echo "==================================================================="
echo " RViz: beacons (green exit, blue trail, coloured hazards) and their"
echo "   links, green line = beacons still to visit, orange = current path,"
echo "   hazard zones turn green when treated."
echo " What the Executor is doing:  tail -f /tmp/executor.log"
echo "   (GOTO -> APPROACH -> treating -> next target ... RETURN ->"
echo "    MISSION COMPLETE)"
echo " Live status:  ros2 topic echo /executor/status"
if [ "$ONA" = 1 ]; then
echo " ONA: open the dashboard ($CP), pick targets, press Dispatch mission."
echo "   The Executor waits ${WAIT_BRIEFING:-60} s (simulated) for it. ONA log: tail -f /tmp/ona.log"
echo "   Link: ros2 topic echo /ona_link/status    briefing: ros2 topic echo /executor/briefing"
fi
echo " Logs: /tmp/exec_sim.log /tmp/exec_slam.log /tmp/executor.log /tmp/exec_radio.log /tmp/exec_$([ "$MINE" = 1 ] && echo mine_sensors || echo vision).log"
echo " STOP: press Ctrl-C here."
echo "==================================================================="
wait
