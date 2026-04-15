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
    x = -1.25
    y = 0.45
    z = 0.40
    theta = 0.0
    frequency = 25.0

    package_dir = get_package_share_directory("nhk2026_sim")
    urg_node2_nl_pkg = get_package_share_directory('urg_node2_nl')

    xacro_file = os.path.join(package_dir, "urdf", "r2_robot.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    world = os.path.join(
        get_package_share_directory("nhk2026_sim"), "worlds", "field_nhk.world"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r  {world}'])]
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
            '/scan_front@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/scan_back@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/nhk2026/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )

    rviz_config_path = os.path.join(
        package_dir,
        "config",
        "nhk2026.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        remappings=[('clock', '/world/nhk2026/clock')]
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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joy_controller_node = Node(
        package="nhk2026_localization",
        executable="joy_controller_node",
        name="joy_controller_node",
        output="screen"
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

    step_leg_worker = Node(
        package='nhk2026_system',
        executable='step_leg_action_server',
        name='step_leg_worker',
        output='screen'
    )

    # 3. 上位ノード (Sequencer: 1〜6のシーケンス制御)
    step_leg_sequencer = Node(
        package='nhk2026_system',
        executable='step_action_server',
        name='step_leg_sequencer',
        output='screen'
    )

    # 4. Joint State Publisher (脚の現在角を配信)
    jsp_node = Node(
        package='nhk2026_control',
        executable='joint_state_publisher_step',
        name='joint_state_publisher_step',
        output='screen'
    )

    vel_feedback_node = Node(
        package="nhk2026_localization",
        executable="vel_feedback_node",
        output="screen",
        remappings=[('clock', '/world/nhk2026/clock')],
    )

    blossom_path_planner = Node(
        package="nhk2026_pursuit",
        executable="blossom_path_planner",
        output="screen",
        parameters=[{
            "num_points_": 50,
            "shorten_": 0.15,
            "theta_offset_": 0.0,
            "start_shorten_": 0.65,
            "end_shorten_": 0.45,
        }],
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
            "max_linear_speed": 1.75,
            "max_angular_speed": 0.7,
            "max_linear_tolerance": 0.75,
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
            "max_rotate_speed_": 0.7,
            "slow_rotate_speed_": 0.4,
            "accel_angle_": math.pi / 10,
            "stop_angle_": math.pi / 90,
            "offset_z_": 0.02,
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
    takano_hand_sequencer = Node(
        package='nhk2026_system',
        executable='takano_hand_server',
        name='takano_hand_sequencer',
        output='screen'
    )

   

 
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='WITH_lidar', value='2'),
         DeclareLaunchArgument('node_name', default_value='urg_node2_nl'),
        SetEnvironmentVariable(name='WITH_SIM', value='0'),
        # gazebo,
        # gz_spawn_entity,
        # rviz,
        # bridge,
        node_robot_state_publisher,
        static_from_map_to_odom,#
        # joy_node,
        mcl_3d_node,#
        map_publisher,#
        livox_driver,#
        mcl_node,#
        urg_node_front,#
        urg_node_rear,#
        lidar_filter_node,#
        joy_controller_node,
        vel_feedback_node,
        # blossom_path_planner,
        # path_planner,
        # pursuit,
        # bt_node,
        step_leg_worker,
        step_leg_sequencer,
        jsp_node,
        mcl_manage,#
        takano_hand_sequencer,
    ])
