from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch

def generate_launch_description():
    pkg_share = get_package_share_directory('nhk2026_bridge')
    canid_file = os.path.join(pkg_share, 'config', 'nhk206_canbridge.yml')

    name_space = ''
    ld = LaunchDescription()

    # canbridge node
    canbridgenode = LifecycleNode(
        package='nhk2026_bridge',
        executable='nhk2026_canbridge',
        name='nhk2026_canbridge',
        namespace=name_space,
        parameters=[canid_file],
        output='screen',
        emulate_tty=True, 
    )

    ld.add_action(canbridgenode)

    canbridge_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=canbridgenode,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(canbridgenode),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    canbridge_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=canbridgenode,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(canbridgenode),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    ld.add_action(canbridge_configure_event_handler)
    ld.add_action(canbridge_activate_event_handler)

    return ld