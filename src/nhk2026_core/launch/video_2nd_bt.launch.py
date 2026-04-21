import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import launch_ros

import xacro
import math

def generate_launch_description():
    ld = LaunchDescription()
    
    bt_xml_file = LaunchConfiguration("bt_xml_file")
    bt_start_delay = LaunchConfiguration("bt_start_delay")
    wait_for_server_timeout_ms = LaunchConfiguration("wait_for_server_timeout_ms")
    k_pos_tolerance = LaunchConfiguration("k_pos_tolerance")
    name_space = 'aro'

    """ide_arm begin"""
    ide_arm_action_server_node = Node(
        package="nhk2026_system",
        executable="ide_arm_action_server",
        name="ide_arm_action_server",
        output="screen",
        parameters=[{"kPosTolerance": k_pos_tolerance}],
        namespace=name_space,
    )
    """ide_arm end"""

    """pursuit nodes begin"""
    pursuit_forest = Node(
        package="nhk2026_pursuit",
        executable="pursuit",
        output="screen",
        parameters=[{
            "max_linear_speed": 1.0,
            "max_angular_speed": 0.5,
            "max_linear_tolerance": 0.3,
            "max_theta_tolerance": 0.01,
            "max_reaching_distance": 0.02,
            "max_reaching_theta": 0.10,
            "lookahead_distance": 0.20,
            "resampleThreshold": 0.10,
            "Kp_tan": 0.80,
            "Ki_tan": 0.0,
            "Kd_tan": 0.0,
            "Kp_normal": 0.80,
            "Ki_normal": 0.0,
            "Kd_normal": 0.00,
            "Kp_theta": 1.0,
            "Ki_theta": 0.00,
            "Kd_theta": 0.00,
            "x": 10,
            "max_rotate_speed_": 0.5,
            "slow_rotate_speed_": 0.3,
            "accel_angle_": math.pi / 10,
            "stop_angle_": math.pi / 90,
            "offset_z_": 0.02,
        },
        ],
        remappings=[("follow", "follow_forest")],
        namespace=name_space,
    )
    pursuit_outline = Node(
        package="nhk2026_pursuit",
        executable="pursuit",
        output="screen",
        parameters=[{
            "max_linear_speed": 1.0,
            "max_angular_speed": 0.5,
            "max_linear_tolerance": 0.3,
            "max_theta_tolerance": 0.1,
            "max_reaching_distance": 1.00,
            "max_reaching_theta": 0.10,
            "lookahead_distance": 0.20,
            "resampleThreshold": 0.10,
            "Kp_tan": 0.80,
            "Ki_tan": 0.0,
            "Kd_tan": 0.0,
            "Kp_normal": 0.80,
            "Ki_normal": 0.0,
            "Kd_normal": 0.00,
            "Kp_theta": 1.0,
            "Ki_theta": 0.00,
            "Kd_theta": 0.00,
            "x": 10,
            "max_rotate_speed_": 0.5,
            "slow_rotate_speed_": 0.3,
            "accel_angle_": math.pi / 10,
            "stop_angle_": math.pi / 90,
            "offset_z_": 0.02,
        },
        ],
        remappings=[("follow", "follow_outline")],
        namespace=name_space,
    )
    pursuit_detail = Node(
        package="nhk2026_pursuit",
        executable="pursuit",
        output="screen",
        parameters=[{
            "max_linear_speed": 2.0,
            "max_angular_speed": 0.5,
            "max_linear_tolerance": 0.3,
            "max_theta_tolerance": 0.01,
            "max_reaching_distance": 0.02,
            "max_reaching_theta": 0.10,
            "lookahead_distance": 0.20,
            "resampleThreshold": 0.10,
            "Kp_tan": 0.80,
            "Ki_tan": 0.0,
            "Kd_tan": 0.0,
            "Kp_normal": 0.80,
            "Ki_normal": 0.0,
            "Kd_normal": 0.00,
            "Kp_theta": 1.0,
            "Ki_theta": 0.00,
            "Kd_theta": 0.00,
            "x": 10,
            "max_rotate_speed_": 0.5,
            "slow_rotate_speed_": 0.3,
            "accel_angle_": math.pi / 10,
            "stop_angle_": math.pi / 90,
            "offset_z_": 0.02,
        },
        ],
        remappings=[("follow", "follow_detail")],
        namespace=name_space,
    )
    """pursuit nodes end"""

    """takano hand begin"""
    takano_hand_sequencer = Node(
        package='nhk2026_system',
        executable='takano_hand_server',
        name='takano_hand_sequencer',
        output='screen',
        namespace=name_space,
    )
    """takano hand end"""

    """step leg begin"""
    step_leg_sequencer = Node(
        package='nhk2026_system',
        executable='step_action_server',
        name='step_leg_sequencer',
        output='screen',
        namespace=name_space,
    )

    step_legs_angle = Node(
        package='nhk2026_system',
        executable='step_leg_action_server',
        name='step_leg_action_server',
        output='screen',
        namespace=name_space,
    )
    """step leg end"""

    """bt start"""
    bt_node = Node(
        package="nhk2026_system",
        executable="video_2nd_bt_node",
        name="video_2nd_bt_node",
        output="screen",
        parameters=[
            {"bt_xml_file": bt_xml_file},
            {"wait_for_server_timeout_ms": wait_for_server_timeout_ms},
            {"k_pos_tolerance": k_pos_tolerance},
        ],
        namespace=name_space,
    )
    """bt end"""
    
    ld.add_action(DeclareLaunchArgument(
        "bt_xml_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("nhk2026_system"),
            "config",
            "video_2nd_bt.xml",
        ]),
        description="Behavior tree XML file path.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "bt_start_delay",
        default_value="4.0",
        description="Delay in seconds before starting bt_node.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "wait_for_server_timeout_ms",
        default_value="5000",
        description="Timeout in ms to wait for the ide_arm action server.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "k_pos_tolerance",
        default_value="0.06",
        description="Joint position tolerance used by ide_arm_action_server.",
    ))
    
    ld.add_action(ide_arm_action_server_node)
    ld.add_action(pursuit_forest)
    ld.add_action(pursuit_outline)
    ld.add_action(pursuit_detail)
    ld.add_action(TimerAction(
        period=bt_start_delay,
        actions=[bt_node],
    ))
    ld.add_action(takano_hand_sequencer)
    ld.add_action(step_leg_sequencer)
    ld.add_action(step_legs_angle)
    
    
    return ld
