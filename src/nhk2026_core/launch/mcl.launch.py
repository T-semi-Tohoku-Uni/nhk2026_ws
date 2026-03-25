import os
import math
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 初期位置・設定パラメータ ---
    init_x = -1.25
    init_y = 0.45
    init_z = 0.40
    init_theta = 0.0

    # パッケージディレクトリの取得
    pkg_sim = get_package_share_directory("nhk2026_sim")
    pkg_loc = get_package_share_directory("nhk2026_localization")
    pkg_urg = get_package_share_directory('urg_node2_nl')

    # --- ロボットモデル (URDF/Xacro) の読み込み ---
    xacro_file = os.path.join(pkg_sim, "urdf", "r2_robot.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    # --- Livox Lidar (3D) 設定 ---
    livox_config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config',
        'MID360_config.json'
    )
    livox_params = [
        {"xfer_format": 0},
        {"multi_topic": 0},
        {"data_src": 0},
        {"publish_freq": 10.0},
        {"output_data_type": 0},
        {"frame_id": 'livox_frame'},
        {"user_config_path": livox_config_path}
    ]

    # --- ノードの定義 ---

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Static TF (map -> odom)
    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 3. 2D Lidar Drivers (Front & Rear)
    urg_node_front = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name="urg_node_front",
        remappings=[('scan', 'scan_front')],
        parameters=[PathJoinSubstitution([pkg_urg, "config", "params_ether.yaml"])]
    )
    urg_node_rear = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name='urg_node_rear',
        remappings=[('scan', 'scan_back')],
        parameters=[PathJoinSubstitution([pkg_urg, "config", "params_ether_2nd.yaml"])]
    )

    # 4. 2D Lidar Filter (scan_front/back を統合して multi_scan を生成)
    lidar_filter_node = Node(
        package="nhk2026_localization",
        executable="lidar_filter",
        parameters=[{"filter_threshold": 0.98}]
    )

    # 5. Livox Driver (3D Lidar)
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        parameters=livox_params
    )

    # 6. Integrated MCL Node (統合版自己位置推定)
    # 2D/3D両方のマップパスとノイズパラメータを設定
    integrated_mcl_node = Node(
        package="nhk2026_localization",
        executable="integrated_mcl_node",
        output="screen",
        parameters=[{
            # 初期ポーズ
            "initial_x": init_x,
            "initial_y": init_y,
            "initial_z": init_z,
            "initial_theta": init_theta,
            "particleNum": 200,
            
            # マップ設定
            "mapFile2D": "src/nhk2026_localization/map/nhk2026_field.h5",
            "mapFile3D": "src/nhk2026_localization/map/nhk2026_field_tamokuteki.h5",
            "mapZIndex2D": 4,
            "mapResolution2D": 0.01,
            "mapResolution3D": 0.01,

            # 2D用オドメトリノイズ
            "odomNoise1_2D": 1.2,
            "odomNoise2_2D": 0.5,
            "odomNoise3_2D": 1.3,
            "odomNoise4_2D": 1.1,

            # 3D用オドメトリノイズ
            "odomNoise1_3D": 1.5,
            "odomNoise2_3D": 1.0,
            "odomNoise3_3D": 2.0,
            "odomNoise4_3D": 1.0,

            # 共通センサパラメータ
            "lfmSigma": 0.05,
            "zHit": 0.9,
            "zRand": 0.1,
            "scanStep": 5
        }]
    )

    # 7. MCL Management (Z軸レベルに応じたノード管理)
    mcl_manage = Node(
        package="nhk2026_localization",
        executable="mcl_manage",
        name="mcl_manage"
    )

    # 8. 可視化用マップメッシュ
    map_publisher = Node(
        package="nhk2026_localization",
        executable="map_mesh_publisher",
        name="map_mesh_publisher"
    )

    # 9. コントローラー系
    joy_node = Node(
        package="joy",
        executable="joy_node"
    )
    joy_controller_node = Node(
        package="nhk2026_localization",
        executable="joy_controller_node"
    )

    # 速度フィードバック変換ノード (OdometryをTwistに変換)
    vel_feedback_node = Node(
        package="nhk2026_localization",
        executable="vel_feedback_node"
    )

    return LaunchDescription([
        # 環境変数の設定
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='WITH_SIM', value='0'),
        
        # ノードの起動
        node_robot_state_publisher,
        static_tf_map_odom,
        urg_node_front,
        urg_node_rear,
        lidar_filter_node,
        livox_driver,
        integrated_mcl_node,
        map_publisher,
        joy_node,
        joy_controller_node,
        vel_feedback_node
    ])