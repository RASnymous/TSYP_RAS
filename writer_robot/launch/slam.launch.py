import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# On ROS 2 Jazzy+ / Lyrical, slam_toolbox is a LIFECYCLE node. Launching it
# as a plain Node leaves it UNCONFIGURED: it never subscribes to /scan and
# never publishes map->odom. It must be driven CONFIGURE -> ACTIVATE, which
# is what the event handlers below do (this mirrors slam_toolbox's own
# online_async_launch.py with autostart=true, use_lifecycle_manager=false).


def generate_launch_description():
    pkg = get_package_share_directory('writer_robot')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}])

    # When the node finishes CONFIGURE (reaches 'inactive'), fire ACTIVATE.
    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[slam] configured -> activating'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ]))

    # Kick off CONFIGURE as soon as the node is up.
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))

    # Register the activate handler BEFORE emitting configure.
    return LaunchDescription([slam, activate_on_configured, configure])
