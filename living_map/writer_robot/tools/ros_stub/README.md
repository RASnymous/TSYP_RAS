# ros_stub - a tiny stand-in for ROS 2 Python (tests only)

Just enough of `rclpy`, `tf2_ros`, the message packages and `cv_bridge` to run
the Writer robot's nodes **without ROS** (Windows, CI, a PC without ROS) and
check what they publish. Used by `tools/test_nodes.py` only - never put this
folder on the Python path of a real ROS 2 install.
