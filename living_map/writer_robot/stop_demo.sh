#!/usr/bin/env bash
# Stop every Writer / Executor simulation process (Gazebo, bridge, SLAM, EKF,
# explorer, executor, vision, mine sensors, beacons, recorder, radio, ONA, RViz).
#   bash ~/writer_robot_ws/stop_demo.sh
# The patterns match the running programs only, never an editor that has one
# of the source files open.
echo ">> stopping the simulation..."
P='gz sim|gz-sim|ruby.*gz|gazebo_sim\.launch|ros_gz_bridge/parameter_bridge|async_slam_toolbox_node|lib/robot_state_publisher/|lib/robot_localization/ekf_node|lib/rviz2/rviz2|python3 -m ona --config'
N='(lib/writer_robot/|writer_robot )(frontier_explorer|beacon_drop_node|vision_event_detector|frame_translator_node|mission_recorder|executor_node|beacon_radio_sim|ona_link|mine_sensors)'
pkill -INT -f "$N" 2>/dev/null ; pkill -INT -f "$P" 2>/dev/null
sleep 1
pkill -9 -f "$N" 2>/dev/null ; pkill -9 -f "$P" 2>/dev/null
pkill -9 -f 'controller_server|planner_server|bt_navigator|smoother_server|behavior_server|lifecycle_manager' 2>/dev/null
sleep 1
echo ">> done. All stopped."
