#!/usr/bin/env bash
# =====================================================================
#  Health check of a RUNNING simulation (Writer or Executor).
#  Run it in a second terminal while run_explorer.sh / run_executor.sh runs:
#      bash ~/writer_robot_ws/check_sim.sh
#  Send the whole output if something does not work.
# =====================================================================
source /opt/ros/lyrical/setup.bash
source "$HOME/writer_robot_ws/install/setup.bash"
echo "== topic rates (a few seconds each)"
for t in /clock /scan /odom /imu /map /camera/image_raw /camera/depth/image_raw /cmd_vel; do
  r=$(timeout 7 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
  printf "  %-28s %s\n" "$t" "${r:-NO DATA}"
done
echo "== TF (position in m, rotation in degrees)"
for pair in "odom base_footprint" "map odom" "map base_footprint"; do
  out=$(timeout 8 ros2 run tf2_ros tf2_echo $pair 2>/dev/null | grep -m2 -E "Translation|RPY \(degree\)" | sed 's/^ *- //' | tr '\n' ' ')
  printf "  %-22s %s\n" "$pair" "${out:-MISSING}"
done
echo "== nodes publishing /tf (default: robot_state_publisher, odom_tf_bridge, slam_toolbox - each once)"
ros2 topic info /tf -v 2>/dev/null | awk '/^Publisher count/{p=1} /^Subscription count/{p=0} p && /Node name/{print $3}' | sort | uniq -c
echo "== nodes"
ros2 node list 2>/dev/null | sort | tr '\n' ' '; echo
echo "== Gazebo real-time factor"
timeout 6 gz topic -e -t /world/contaminated_zone/stats -n 1 2>/dev/null | grep real_time_factor || echo "  (no stats)"
for f in /tmp/explorer.log /tmp/executor.log; do
  [ -f "$f" ] && { echo "== last lines of $f"; tail -n 6 "$f"; }
done
