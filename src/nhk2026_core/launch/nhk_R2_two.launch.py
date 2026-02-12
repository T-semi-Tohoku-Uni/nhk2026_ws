from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node, LifecycleNode
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart


import os
import xacro
import math

def generate_launch_description():
    x = -1.47
    y = 0.45
    z = 0.0
    theata = 0.0

    package_dir = get_package_share_directory("nhk2026_sim")
    urg_node2_nl_pkg = get_package_share_directory('urg_node2_nl')

    xacro_file = os.path.join(package_dir, "urdf", "r2_robot.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}



    urg_node_front = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name="urg_node_front" ,
        remappings=[('scan', 'scan_front')],
        parameters=[PathJoinSubstitution([urg_node2_nl_pkg, "config", "params_ether.yaml"])],
        namespace='',
        output='screen',
    )

    urg_node_rear = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name='urg_node_rear',                    
        remappings=[('scan', 'scan_back')],       
        parameters=[PathJoinSubstitution([urg_node2_nl_pkg, "config", "params_ether_2nd.yaml"])],
        output='screen',
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            params,
        ]
    ) 


    # # #ldlidar
    # ldlidar_params = PathJoinSubstitution(
    #     [FindPackageShare("yasarobo2025_26"), "config", "ldlidar_settings.yaml"]
    # )

    # ldlidar_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [FindPackageShare("ldlidar_node"), "launch", "ldlidar_with_mgr.launch.py"]
    #         )
    #     ),
    #     launch_arguments={"params_file": ldlidar_params}.items(),
    # )

    # tf transfromer
    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

 

    # static_from_odom_to_basefootprint = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_odom_to_basefootprint",
    #     output="screen",
    #     arguments=[
    #         "-1.47",          # x  [m]
    #         "0.45",          # y  [m]
    #         "0.0",             # z  [m]
    #         "0",             # yaw   [rad]
    #         "0",             # pitch [rad]
    #         "0",             # roll  [rad]
    #         "odom",          # parent  frame
    #         "base_footprint" # child   frame
    #     ]
    # )

    mcl_node = Node(
        package="nhk2026_localization",
        executable="mcl_node",
        output="screen",
        parameters=[{
            "particleNum": 50,
            "initial_x": x,
            "initial_y": y,
            "initial_theta": theata,
            "odomNoise1": 2.0,
            "odomNoise2": 0.5,
            "odomNoise3": 2.0,
            "odomNoise4": 5.0,
            "resampleThreshold": 0.9,
            "scanStep": 5,
            "lidar_threshold": 3.0/40.0*math.pi,
            "mapFile":"src/nhk2026_localization/map/nhk2026_field_tamokuteki.h5",
        }],
    )

 

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joy2Vel_node = Node(
        package="nhk2026_localization",
        executable="joy2vel",
        name="joy2vel",
        output="screen"
    )

    vel_feedback_pass_through = Node(
        package="nhk2026_localization",
        executable="vel_feedback_pass_through",
        output="screen",
    )

 
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='urg_node2_nl'),
        SetEnvironmentVariable(name='WITH_SIM', value='0'),
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='WITH_lidar', value='2'),
        node_robot_state_publisher,
        static_from_map_to_odom,
        mcl_node,
        # joy_node,
        joy2Vel_node,
        urg_node_front,
        urg_node_rear,
        # static_from_odom_to_basefootprint,
        vel_feedback_pass_through,
    ])