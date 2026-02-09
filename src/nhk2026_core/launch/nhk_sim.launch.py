import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import launch_ros

import xacro
import math
import random
def generate_launch_description():
    x = 1.0
    y = 1.0
    z = 0.0
    theta = 0.0

    package_dir = get_package_share_directory("nhk2026_sim")

    world = os.path.join(
        get_package_share_directory("nhk2026_sim"), "worlds", "field_nhk.world"
    )
    
    rviz_config_path = os.path.join(
        package_dir,
        "config",
        "nhk2026.rviz"
    )



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r -s {world}'])]
    )

    xacro_file = os.path.join(package_dir, "urdf", "r2_robot.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gz_spawn_entity = Node(
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
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_1',
        arguments=[
            '/scan_front@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/scan_back@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/world/nhk2026/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        remappings=[('clock', '/world/nhk2026/clock')]
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        remappings=[('clock', '/world/nhk2026/clock')]
    )

    mcl_node = Node(
        package="nhk2026_localization",
        executable="mcl_node",
        parameters=[
            {
                "initial_x": x,
                "initial_y": y,
                "initial_theta": theta,
                "scanStep": 5,
            },
        ],
        remappings=[('clock', '/world/nhk2026/clock')],
        output="screen"
    )

    vel_feedback_node = Node(
        package="nhk2026_localization",
        executable="vel_feedback_node",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    # joy
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    joy2Vel_node = Node(
        package="yasarobo2025_26",
        executable="joy2vel",
        name="joy2vel",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )


    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        SetEnvironmentVariable(name='WITH_lidar', value='0'),
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        rviz,
        static_from_map_to_odom,
        mcl_node,
        joy_node,
        joy2Vel_node,
        vel_feedback_node,
    ])
