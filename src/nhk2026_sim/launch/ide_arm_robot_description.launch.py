#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import math
import os
import xacro

def generate_launch_description():


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("nhk2026_sim"),
                    "urdf",
                    "ide_arm.xacro",
                ]
            ),

        ]
    )
    robot_description = {"robot_description": robot_description_content}
    package_dir = get_package_share_directory("nhk2026_sim")
    x = 1.0
    y = 1.0
    z = 0.10
    theta = math.pi/2
    
    world = os.path.join(
        get_package_share_directory("nhk2026_sim"), "worlds", "field_nhk.world"
    )

    xacro_file = os.path.join(package_dir, "urdf", "r2_robot.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r {world}'])]
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-string', robot_desc,
                    '-name', 'robot',
                    '-allow_renaming', 'false',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                    '-Y', str(theta)
                    ],
        ),
        gazebo,
    ])