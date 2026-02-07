import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

import xacro
import math
import random
def generate_launch_description():
    x = 1.0
    y = 1.0
    z = 0.10
    theta = math.pi/2

    package_dir = get_package_share_directory("nhk2026_sim")

    world = os.path.join(
        get_package_share_directory("nhk2026_sim"), "worlds", "field_nhk.world"
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r {world}'])]
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
            '/pointcloud2_front/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/pointcloud2_back/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/world/nhk2026/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'],
        output='screen'
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
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        # joy_node,
        joy2Vel_node,
    ])
