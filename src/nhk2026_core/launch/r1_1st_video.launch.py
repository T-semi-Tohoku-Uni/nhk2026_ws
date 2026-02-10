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

def generate_launch_description():
    ld = LaunchDescription()
    name_space = 'r1'

    vel_joy_node = LifecycleNode(
        package='nhk2026_bridge',
        executable='joy_vel_converter',
        name='joy_vel_converter',
        namespace=name_space,
        output='screen',
        emulate_tty=True,
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
    return ld