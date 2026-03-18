from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bt_xml_file = LaunchConfiguration("bt_xml_file")
    wait_for_server_timeout_ms = LaunchConfiguration("wait_for_server_timeout_ms")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bt_xml_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("nhk2026_system"),
                "config",
                "ide_arm_bt.xml",
            ]),
            description="Behavior tree XML file path.",
        ),
        DeclareLaunchArgument(
            "wait_for_server_timeout_ms",
            default_value="5000",
            description="Timeout in ms to wait for the ide_arm action server.",
        ),
        Node(
            package="nhk2026_system",
            executable="ide_arm_bt_node",
            name="ide_arm_bt_node",
            output="screen",
            parameters=[
                {"bt_xml_file": bt_xml_file},
                {"wait_for_server_timeout_ms": wait_for_server_timeout_ms},
            ],
        ),
    ])
