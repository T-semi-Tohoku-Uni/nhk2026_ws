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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg
import launch

import os
import xacro
import math
import subprocess

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

def _require_can0(context, *args, **kwargs):
    try:
        subprocess.check_output(["/usr/sbin/ip", "link", "show", "can0"], stderr=subprocess.STDOUT, text=True)
        return []
    except subprocess.CalledProcessError as e:
        msg = f"can0 が見つかりません。launch を停止します。\n(ip output: {e.output.strip()})"
        launch.logging.get_logger(__name__).error(msg)
        return [Shutdown(reason="can0 not found")]
    except FileNotFoundError:
        msg = "ip コマンドが見つかりません。launch を停止します。"
        launch.logging.get_logger(__name__).error(msg)
        return [Shutdown(reason="ip command not found")]

def _ensure_can0_up(context, *args, **kwargs):
    try:
        out = subprocess.check_output(["/usr/sbin/ip", "-details", "link", "show", "can0"], text=True)
    except Exception:
        return []

    desired_ok = all(token in out for token in ("bitrate 1000000", "dbitrate 2000000", "fd on"))
    if "UP" in out and desired_ok:
        return []

    cmds = [
        ["sudo", "-n", "/usr/sbin/ip", "link", "set", "can0", "down"],
        [
            "sudo", "-n", "/usr/sbin/ip", "link", "set", "can0", "up",
            "type", "can",
            "bitrate", "1000000",
            "dbitrate", "2000000",
            "fd", "on",
        ],
    ]
    for cmd in cmds:
        try:
            subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        except subprocess.CalledProcessError as e:
            launch.logging.get_logger(__name__).error(f"can0 設定失敗: {e.output.strip()}")
            return [Shutdown(reason="can0 setup failed")]
    return []

def generate_launch_description():
    x = -1.25
    y = 0.45
    z = 0.40
    theta = 0.0
    frequency = 25.0

    name_space = "aro"

    """parameter begin"""
    sim_package_dir = get_package_share_directory("nhk2026_sim")
    urg_node2_nl_pkg = get_package_share_directory('urg_node2_nl')

    xacro_file = os.path.join(sim_package_dir, "urdf", "r2_all.xacro")
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    """parameter end"""

    """can node begin"""
    pkg_share_bridge = get_package_share_directory('nhk2026_bridge')
    canid_file = os.path.join(pkg_share_bridge, 'config', 'video_2nd_canbridge.yml')
    canbridgenode = LifecycleNode(
        package='nhk2026_bridge',
        executable='nhk2026_canbridge',
        name='nhk2026_canbridge',
        namespace=name_space,
        parameters=[canid_file],
        output='screen',
        emulate_tty=True, 
    )
    canbridge_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=canbridgenode,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(canbridgenode),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    canbridge_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=canbridgenode,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(canbridgenode),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )
    """can node end"""
    
    """parameter nodes begin"""
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            params,
        ],
        namespace=name_space,
    ) 

    # tf transfromer
    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    """parameter nodes end"""

    """lidar nodes begin"""
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
        parameters=livox_ros2_params,
        namespace=name_space,
    )
    
    urg_node_front = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name="urg_node_front" ,
        remappings=[('scan', 'scan_front')],
        parameters=[PathJoinSubstitution([urg_node2_nl_pkg, "config", "params_ether.yaml"])],
        namespace=name_space,
        output='screen',
    )

    urg_node_rear = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name='urg_node_rear',                    
        remappings=[('scan', 'scan_back')],       
        parameters=[PathJoinSubstitution([urg_node2_nl_pkg, "config", "params_ether_2nd.yaml"])],
        namespace=name_space,
        output='screen',
    )
    """lidar nodes end"""
    
    """localization nodes begin"""
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
            "mapFile":"src/nhk2026_localization/map/nhk2026_field.h5",
        }],
        namespace=name_space,
    )

    lidar_filter_node = Node(
        package="nhk2026_localization",
        executable="lidar_filter",
        name="lidar_filter",
        output="screen",
        parameters=[{
            "filter_threshold": 0.98, 
        }],
        namespace=name_space,
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
                "lfmSigma": 0.05,
                "odomNoise1": 1.5,
                "odomNoise2": 1.0,
                "odomNoise3": 0.5,
                "odomNoise4": 0.5,
                
                "mapFile":"src/nhk2026_localization/map/nhk2026_field.h5",
            },
        ],
        namespace=name_space,
    )
    
    mcl_manage = Node(
        package="nhk2026_localization",
        executable="mcl_manage",
        name="mcl_manage",
        output="screen",
        namespace=name_space,
    )
    
    map_publisher = Node(
        package="nhk2026_localization",
        executable="map_mesh_publisher",
        name="map_mesh_publisher",
        output="screen",
        namespace=name_space,
    )
    """localization nodes end"""

    """ide arm nodes begin"""
    joint_state_publisher_ide_arm_node = Node(
        package='nhk2026_control',
        executable='joint_state_publisher_ide',
        name='joint_state_publisher_ide_arm',
        output='screen',
        namespace=name_space,
    )
    arm_path_plan_node = Node(
        package="nhk2026_control",
        executable="arm_path_plan",
        name="arm_path_plan",
        output="screen",
        namespace=name_space,
    )
    vacuum_server_node = Node(
        package="nhk2026_control",
        executable="vacuum_server",
        name="vacuum_server",
        output="screen",
        namespace=name_space,
    )
    vacuum_stack_server_node = Node(
        package="nhk2026_control",
        executable="vacuum_stack_servier",
        name="vacuum_stack_server",
        output="screen",
        namespace=name_space,
    )
    """ide arm nodes end"""

    """pursuit nodes begin"""
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
        namespace=name_space,
    )
    """pursuit nodes end"""

    """step leg nodes begin"""
    jsp_node = Node(
        package='nhk2026_control',
        executable='joint_state_publisher_step',
        name='joint_state_publisher_step',
        output='screen',
        namespace=name_space,
    )
    """step leg nodes end"""

    return LaunchDescription([
        OpaqueFunction(function=_require_can0),
        OpaqueFunction(function=_ensure_can0_up),
        canbridgenode,
        canbridge_configure_event_handler,
        canbridge_activate_event_handler,
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
        joint_state_publisher_ide_arm_node,
        arm_path_plan_node,
        vacuum_server_node,
        vacuum_stack_server_node,
        path_planner,
        jsp_node,
    ])