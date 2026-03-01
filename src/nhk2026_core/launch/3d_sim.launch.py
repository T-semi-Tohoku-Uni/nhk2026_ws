import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import launch_ros
from launch.actions import TimerAction

import xacro
import math
import random

def generate_launch_description():
    x = -1.8
    y = 3.4
    z = 0.4
    theta = 0.0
    frequency = 25.0

    #これがないと動かん(gpuがのっていないやつは)
    # os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

    package_dir = get_package_share_directory("nhk2026_sim")

    world = os.path.join(
        get_package_share_directory("nhk2026_sim"), "worlds", "field_nhk.world"
    )
    
    rviz_config_path = os.path.join(
        package_dir,
        "config",
        "nhk3d.rviz"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r   {world}'])]
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

   
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_1',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/livox/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/world/nhk2026/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            ],
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

    mcl_3d_node = Node(
        package="nhk2026_localization",
        executable="mcl_3d_node", 
        name="mcl_3d_node",
        output="screen",
        parameters=[
            {
                # ロボットの初期位置(x, y, theta)とMCLの初期位置を一致させる
                "initial_x": x,
                "initial_y": y,
                "initial_z": z,
                "initial_theta": theta,
                
                
                # MCLのパラメータ
                "particleNum": 100,
                "mapResolution": 0.01,
                "lfmSigma": 0.03,
            },
        ],
        remappings=[
            ('clock', '/world/nhk2026/clock'),
            ('/livox/lidar', '/livox/points'),
        ],
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
        package="nhk2026_localization",
        executable="joy2vel",
        name="joy2vel",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    map_publisher = Node(
        package="nhk2026_localization",
        executable="map_mesh_publisher",
        name="map_mesh_publisher",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    path_planner = Node(
        package="nhk2026_pursuit",
        executable="path_planner",
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

    pursuit = Node(
        package="nhk2026_pursuit",
        executable="pursuit",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.15,
            "max_angular_speed": 0.7,
            "max_linear_tolerance": 0.15,
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
        parameters=[{"bt_xml_file" : os.path.join(get_package_share_directory("yasarobo2025_26"), "config", "blue_bt.xml")}]
    )

    delayed_bt_node = TimerAction(
        period=5.0, 
        actions=[bt_node]
    )

   


    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        SetEnvironmentVariable('WITH_SIM', '1'),
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        rviz,
        static_from_map_to_odom,
        # joy_node,
        # joy2Vel_node,
        vel_feedback_node,
        map_publisher,
        # path_planner,
        # pursuit,
        #bt_node,
        mcl_3d_node,
        #delayed_bt_node,
    ])