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

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

livox_config_path = os.path.join(
    get_package_share_directory('livox_ros_driver2'), # または設定ファイルがあるパッケージ名
    'config',
    'MID360_config.json'
)
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": livox_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():
    x = -3.0
    y = 5.0
    z = 0.40
    theata = 0.0
    frequency = 25.0

    package_dir = get_package_share_directory("nhk2026_sim")
    urg_node2_nl_pkg = get_package_share_directory('urg_node2_nl')

    xacro_file = os.path.join(package_dir, "urdf", "r2_robot.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}




    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            params,
        ]
    ) 


    # tf transfromer
    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
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


    path_planner = Node(
        package="nhk2026_pursuit",
        executable="path_planner",
        output="screen",
        parameters=[
            {
                "initial_x": x,
                "initial_y": y,
                "initial_theta": theata,
                "sample_parameter": frequency,
            },
        ],
        
    )

    pursuit = Node(
        package="nhk2026_pursuit",
        executable="pursuit",
        output="screen",
        parameters=[{
            "max_linear_speed": 2.0,
            "max_angular_speed": 0.7,
            "max_linear_tolerance": 0.75,
            "max_theta_tolerance": 0.10,
            "max_reaching_distance": 0.05,
            "max_reaching_theta": 0.10,
            "lookahead_distance": 0.20,
            "resampleThreshold": 0.10,
            "Kp_tan": 3.0,
            "Ki_tan": 0.0,
            "Kd_tan": 0.0,
            "Kp_normal": 3.0,
            "Ki_normal": 0.00,
            "Kd_normal": 0.00,
            "Kp_theta": 1.0,
            "Ki_theta": 0.00,
            "Kd_theta": 0.00,
            "x": 10,
        },
        ],
    )

    bt_node = Node (
        package="yasarobo2025_26",
        executable="bt_node",
        output="screen",
        parameters=[{"bt_xml_file" : os.path.join(get_package_share_directory("yasarobo2025_26"), "config", "blue_bt.xml")}]
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
                "initial_theta": theata,
                
                
                # MCLのパラメータ
                "particleNum": 100,
                "mapResolution": 0.01,
                "lfmSigma": 0.03,
            },
        ],
    )

    map_publisher = Node(
        package="nhk2026_localization",
        executable="map_mesh_publisher",
        name="map_mesh_publisher",
        output="screen",
    )

   

 
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='WITH_lidar', value='2'),
        node_robot_state_publisher,
        static_from_map_to_odom,
        # joy_node,
        joy2Vel_node,
        mcl_3d_node,
        map_publisher,
        livox_driver,
    ])
