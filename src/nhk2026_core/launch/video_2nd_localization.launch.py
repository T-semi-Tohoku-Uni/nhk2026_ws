import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable
import launch_ros

import xacro
import math
import random

def generate_launch_description():
    ld = LaunchDescription()

    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = ParameterValue(use_sim, value_type=bool)

    ld.add_action(DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="true: simulation, false: real robot",
    ))

    """localization begin"""
    lidar_filter_node = Node(
        package="nhk2026_localization",
        executable="lidar_filter",
        name="lidar_filter",
        output="screen",
        parameters=[{
            "filter_threshold": 0.98, 
            "use_sim_time": use_sim_time,
        }],
        remappings=[("clock", "/world/nhk2026/clock")],
    )
    ld.add_action(lidar_filter_node)

    x = -1.47
    y = 0.45
    theta = 0.0
    mcl_common_params = [{
        "particleNum": 50,
        "initial_x": x,
        "initial_y": y,
        "initial_theta": theta,
        "odomNoise1": 2.0,
        "odomNoise2": 0.5,
        "odomNoise3": 2.0,
        "odomNoise4": 5.0,
        "resampleThreshold": 0.9,
        "scanStep": 5,
        "lidar_threshold": 3.0 / 40.0 * math.pi,
        "use_sim_time": use_sim_time,
    }]

    mcl_sim_node = Node(
        package="nhk2026_localization",
        executable="mcl_node",
        output="screen",
        parameters=mcl_common_params,
        remappings=[("clock", "/world/nhk2026/clock")],
        condition=IfCondition(use_sim),
    )
    ld.add_action(mcl_sim_node)

    mcl_real_node = Node(
        package="nhk2026_localization",
        executable="mcl_node",
        output="screen",
        parameters=mcl_common_params,
        condition=UnlessCondition(use_sim),
    )
    ld.add_action(mcl_real_node)

    vel_feedback_node = Node(
        package="nhk2026_localization",
        executable="vel_feedback_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("clock", "/world/nhk2026/clock")],
        condition=IfCondition(use_sim),
    )
    ld.add_action(vel_feedback_node)
    """localization end"""

    return ld
