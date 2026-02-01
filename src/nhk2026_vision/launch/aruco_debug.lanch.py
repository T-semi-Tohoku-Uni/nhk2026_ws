from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch

import numpy as np


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory('nhk2026_vision')
    params_file = os.path.join(pkg_share, 'config', 'aruco_params.yml')

    name_space = ''

    aruco_node = LifecycleNode(
        package='nhk2026_vision',
        executable='aruco_node',
        name='aruco_node',
        namespace=name_space,
        parameters=[params_file],
        output='screen',
        emulate_tty=True, 
    )

    degree = -90
    rad = np.deg2rad(degree)
    tf2_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_publisher',
            arguments=[
                '0.0', '0.0', '0.5',
                '0.0', '0.0', str(rad),
                'base_link', 
                'camera_rgbd_optical_frame'
            ]
    )

    ld.add_action(aruco_node)

    ld.add_action(tf2_camera_node)

    return ld