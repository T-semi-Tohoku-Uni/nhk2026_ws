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
    frequency = 25.0


    # cpu simulation setting
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

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

    xacro_file = os.path.join(package_dir, "urdf", "robot.xacro")
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
        arguments=[
            '/ldlidar_node/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/nhk2026/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
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

    vel_feedback_node = Node(
        package="nhk2026_localization",
        executable="vel_feedback_node",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    gen_path = Node(
        package="yasarobo2025_26",
        executable="gen_path",
        output="screen",
        parameters=[
            {
                "initial_x": x,
                "initial_y": y,
                "initial_theta": theta,
                "sample_parameter": frequency,
            },
        ],
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    follow_node = Node(
        package="yasarobo2025_26",
        executable="follow_node",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.10,
            "max_angular_speed": 0.7,
            "max_linear_tolerance": 0.05,
            "max_theta_tolerance": 0.10,
            "max_reaching_distance": 0.05,
            "max_reaching_theta": 0.10,
            "lookahead_distance": 0.20,
            "resampleThreshold": 0.10,
            "Kp_tan": 0.80,
            "Ki_tan": 0.0,
            "Kd_tan": 0.0,
            "Kp_normal": 0.80,
            "Ki_normal": 0.00,
            "Kd_normal": 0.00,
            "Kp_theta": 1.0,
            "Ki_theta": 0.00,
            "Kd_theta": 0.00,
            "x": 10,
        },
        ],
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    bt_node = Node (
        package="yasarobo2025_26",
        executable="bt_node",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    rotate_node = Node(
        package="yasarobo2025_26",
        executable="rotate_node",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    vacume_node = Node(
        package="yasarobo2025_26",
        executable="dummy_vacume_uart",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    detect_node = Node(
        package="yasarobo2025_26",
        executable="ball_detect_node",
        output="screen",
        parameters=[{
            "min_pts": 10,
            "wall_threshold": -1.0,
        },
        ],
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    ball_path_node = Node(
        package="yasarobo2025_26",
        executable="ball_path_node",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
        parameters=[{
            "shorten": 0.15,
            "num_points_": 10
        }]
    )


    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
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
        gen_path,
        follow_node,
        rotate_node,
        bt_node,
        vacume_node,
        detect_node,
        ball_path_node
    ])
