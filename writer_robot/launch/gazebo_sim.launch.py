import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            ExecuteProcess, OpaqueFunction)
try:                                   # keep any path the user already set
    from launch.actions import AppendEnvironmentVariable as _EnvVar
except ImportError:                    # very old launch
    from launch.actions import SetEnvironmentVariable as _EnvVar
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('writer_robot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg, 'urdf', 'writer_robot.urdf.xacro')
    ekf_params = os.path.join(pkg, 'config', 'ekf.yaml')
    bridge_cfg = os.path.join(pkg, 'config', 'gz_bridge.yaml')

    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    role = LaunchConfiguration('robot').perform(context).strip().lower()
    if role not in ('writer', 'executor'):
        role = 'writer'

    # -r run, -v1 quiet; -s = server only (no GUI) -> big speed-up on a VM.
    gz_args = '-r -v1 ' + world
    if headless:
        gz_args = '-r -s -v1 ' + world

    # the mine sensor head (Gafsa mine): mine:=true, or automatic from the world's name
    mine = LaunchConfiguration('mine').perform(context).strip().lower()
    if mine not in ('true', 'false'):
        mine = 'true' if 'gafsa_mine' in os.path.basename(world) else 'false'
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' role:=', role, ' mine:=', mine]),
                                       value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    create = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', f'{role}_robot',
                   '-x', '0', '-y', '0', '-z', '0.12'],
        output='screen')

    # gz <-> ROS bridge, run exactly as the known-good manual command.
    bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '--ros-args', '-p', 'config_file:=' + bridge_cfg],
        output='screen')

    odom_source = LaunchConfiguration('odom_source').perform(context).strip().lower()
    if odom_source == 'ekf':
        # wheel odometry + IMU gyro fused by robot_localization
        odom_tf = Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[ekf_params, {'use_sim_time': True}])
    else:
        # default: Gazebo's OdometryPublisher (the true motion) -> /tf
        odom_tf = Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='odom_tf_bridge',
            arguments=['/odom_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/odom_tf', '/tf')],
            parameters=[{'use_sim_time': True}], output='screen')

    return [gz, rsp, create, bridge, odom_tf]


def generate_launch_description():
    pkg = get_package_share_directory('writer_robot')
    default_world = os.path.join(pkg, 'worlds', 'contaminated_zone.world')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('robot', default_value='writer',
                              description='writer | executor'),
        DeclareLaunchArgument('odom_source', default_value='gz',
                              description='gz = true motion from Gazebo (default) | ekf = wheels + IMU EKF'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = no Gazebo GUI (faster on a VM)'),
        DeclareLaunchArgument('mine', default_value='auto',
                              description='true = mount the mine sensor head (auto: when the world is gafsa_mine)'),
        _EnvVar('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models') + ':' + os.path.dirname(pkg)),
        OpaqueFunction(function=launch_setup),
    ])
