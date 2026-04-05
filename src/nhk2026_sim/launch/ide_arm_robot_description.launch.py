#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import xacro

def generate_launch_description():
    package_share = get_package_share_directory("nhk2026_sim")
    world = os.path.join(package_share, "worlds", "arm_plane.world")
    xacro_file = os.path.join(package_share, "urdf", "ide_arm.xacro")
    spawn_pose = (0.0, 0.0, 0.2, 0.0)

    doc = xacro.process_file(xacro_file, mappings={"use_sim": "true"})
    robot_desc = doc.toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_desc}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f'-s -r {world}'])]
    )
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_1',
        arguments=[
            '/joint_sim1@std_msgs/msg/Float64]ignition.msgs.Double',
            '/joint_sim2@std_msgs/msg/Float64]ignition.msgs.Double',
            '/joint_sim3@std_msgs/msg/Float64]ignition.msgs.Double',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/world/nhk2026/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/nhk2026/model/robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            ('/world/nhk2026/model/robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[('robot_description', 'robot_description_ide_arm')],
    )
    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                '-name', 'robot',
                '-allow_renaming', 'false',
                '-x', str(spawn_pose[0]),
                '-y', str(spawn_pose[1]),
                '-z', str(spawn_pose[2]),
                '-Y', str(spawn_pose[3])
                ],
    )

    sim_bridge_node = Node(
        package='nhk2026_control',
        executable='ide_arm_sim_bridge',
        name='ide_arm_sim_bridge',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        create_node,
        sim_bridge_node
    ])
