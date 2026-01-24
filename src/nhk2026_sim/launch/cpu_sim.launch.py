from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import math
from launch_ros.actions import Node
import xacro
from launch import LaunchDescription


def generate_launch_description():
    x = 0.25
    y = 0.25
    z = 0.30
    theta = math.pi/2

    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

    package_dir = get_package_share_directory("nhk2026_sim")

    bridge_config = os.path.join(package_dir, 'config', 'bridge_config.yaml')

    world = os.path.join(
       package_dir, "worlds", "field.world"
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

    rviz_config_path = os.path.join(
        package_dir,
        "config",
        "default.rviz"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r   {world}'])]
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
        arguments=[
            '/ldlidar_node/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/yasarobo/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )

    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': bridge_config,  # YAMLファイルを指定
    #         'use_sim_time': True
    #     }],
    #     output='screen'
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_config_path],
        remappings=[('/clock','/world/yasarobo/clock')],
        output='log'
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                   '--yaw', '0', '--pitch', '0', '--roll', '0', 
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        remappings=[('/clock','/world/yasarobo/clock')],
        parameters=[{'use_sim_time': True}]
    )

    #mcl
    vel_feedback_node = Node(
        package="nhk2026_localization_ver2",
        executable="vel_feedback_node",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    mcl_node = Node(
        package="nhk2026_localization_ver2",
        executable="mcl_node",
        name='mcl_node',
        parameters=[
            {
                "initial_x": x,
                "initial_y": y,
                "initial_theta": theta,
                "scanStep": 5,
            },
        ],
        remappings=[('clock', '/world/yasarobo/clock')],
        output="screen"
    )


    return LaunchDescription([
        gazebo,
        gz_spawn_entity,
        bridge,
        node_robot_state_publisher, 
        static_tf_node,
        rviz_node,
        vel_feedback_node,
        mcl_node               
    ])
    