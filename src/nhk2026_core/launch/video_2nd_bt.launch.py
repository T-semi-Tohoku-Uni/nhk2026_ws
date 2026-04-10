import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    arm_path_plan_node = Node(
        package="nhk2026_control",
        executable="arm_path_plan",
        name="arm_path_plan",
        output="screen",
    )
    ld.add_action(arm_path_plan_node)

    ide_arm_action_server_node = Node(
        package="nhk2026_system",
        executable="ide_arm_action_server",
        name="ide_arm_action_server",
        output="screen",
        parameters=[{"kPosTolerance": k_pos_tolerance}],
    )
    ld.add_action(ide_arm_action_server_node)

    vacuum_server_node = Node(
        package="nhk2026_control",
        executable="vacuum_server",
        name="vacuum_server",
        output="screen",
    )
    ld.add_action(vacuum_server_node)

    ide_arm_bt_node = Node(
        package="nhk2026_system",
        executable="ide_arm_bt_node",
        name="ide_arm_bt_node",
        output="screen",
        parameters=[
            {"bt_xml_file": bt_xml_file},
            {"wait_for_server_timeout_ms": wait_for_server_timeout_ms},
            {"k_pos_tolerance": k_pos_tolerance},
        ],
    )
    ld.add_action(TimerAction(
        period=bt_start_delay,
        actions=[ide_arm_bt_node],
    ))

    return ld
