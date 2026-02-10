from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

from launch.actions import EmitEvent, RegisterEventHandler
from launch.actions import OpaqueFunction, Shutdown
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    name_space = 'r1'

    bridge_pkg =  get_package_share_directory('nhk2026_bridge')
    vel_file = os.path.join(bridge_pkg, 'config', 'joy2vel.yml')

    vel_joy_node = LifecycleNode(
        package='nhk2026_bridge',
        executable='joy_vel_converter',
        name='joy_vel_converter',
        namespace=name_space,
        output='screen',
        emulate_tty=True,
        parameters=[vel_file],
    )

    ld.add_action(vel_joy_node)

    vel_joy_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=vel_joy_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(vel_joy_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
    ))
    ld.add_action(vel_joy_configure_event_handler)

    vel_joy_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=vel_joy_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(vel_joy_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
    ))
    ld.add_action(vel_joy_activate_event_handler)

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        remappings=[('/joy', f'/{name_space}/joy')],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(joy_node)

    r1_1st_video_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nhk2026_bridge'),
                'launch',
                'nhk2026_can_r1.launch.py',
            )
        ),
    )
    ld.add_action(r1_1st_video_launch)

    return ld