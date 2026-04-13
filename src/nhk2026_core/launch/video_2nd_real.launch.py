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

def generate_launch_description():
    x = -1.25
    y = 0.45
    z = 0.40
    theta = 0.0
    frequency = 25.0

    sim_package_dir = get_package_share_directory("nhk2026_sim")
    urg_node2_nl_pkg = get_package_share_directory('urg_node2_nl')

    xacro_file = os.path.join(sim_package_dir, "urdf", "r2_all.xacro")
    doc = xacro.process_file(xacro_file)
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

    livox_config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'), # または設定ファイルがあるパッケージ名
        'config',
        'MID360_config.json'
    )
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
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    
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
    
    mcl_node = Node(
        package="nhk2026_localization",
        executable="mcl_node",
        output="screen",
        parameters=[{
            "particleNum": 200,
            "initial_x": x,
            "initial_y": y,
            "initial_theta": theta,
            "odomNoise1": 1.2,
            "odomNoise2": 0.5,
            "odomNoise3": 1.3,
            "odomNoise4": 1.1,
            "resampleThreshold": 0.9,
            "scanStep": 5,
            "lidar_threshold": 3.0/40.0*math.pi,
	        "lfmSigma":0.05,
            "mapFile":"src/nhk2026_localization/map/nhk2026_field_tamokuteki.h5",
        }],
    )

    lidar_filter_node = Node(
        package="nhk2026_localization",
        executable="lidar_filter",
        name="lidar_filter",
        output="screen",
        parameters=[{
            "filter_threshold": 0.98, 
        }],
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
    )
    
    mcl_manage = Node(
        package="nhk2026_localization",
        executable="mcl_manage",
        name="mcl_manage",
        output="screen",
    )
    
    map_publisher = Node(
        package="nhk2026_localization",
        executable="map_mesh_publisher",
        name="map_mesh_publisher",
        output="screen",
    )

    return LaunchDescription([
        node_robot_state_publisher,
        static_from_map_to_odom,
        livox_driver,
        urg_node_front,
        urg_node_rear,
        mcl_node,
        lidar_filter_node,
        mcl_3d_node,
        mcl_manage,
        map_publisher,
    ])