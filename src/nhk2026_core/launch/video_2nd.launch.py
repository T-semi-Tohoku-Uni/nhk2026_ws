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
    x = -1.47
    y = 0.45
    z = 0.2
    theta = 0.0

    sim_package_dir = get_package_share_directory("nhk2026_sim")
    world = os.path.join(
        sim_package_dir, "worlds", "field_nhk.world"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r {world}'])]
    )
    ld.add_action(gazebo)

    xacro_file = os.path.join(sim_package_dir, "urdf", "r2_all.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    ld.add_action(node_robot_state_publisher)

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', 'r2',
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-Y', str(theta),
        ]
    )
    ld.add_action(gz_spawn_entity)

    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_1',
        arguments=[
            '/scan_front@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/scan_back@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/nhk2026/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )
    ld.add_action(bridge)

    static_front_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_front_laser',
        arguments=['0', '0.0', '0', '0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )
    ld.add_action(static_front_laser)

    return ld
