from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bt_xml_file = LaunchConfiguration("bt_xml_file")
    bt_start_delay = LaunchConfiguration("bt_start_delay")
    k_pos_tolerance = LaunchConfiguration("k_pos_tolerance")

    arm_path_plan_node = Node(
        package="nhk2026_control",
        executable="arm_path_plan",
        name="arm_path_plan",
        output="screen",
    )

    ide_arm_action_server_node = Node(
        package="nhk2026_system",
        executable="ide_arm_action_server",
        name="ide_arm_action_server",
        output="screen",
        parameters=[{"kPosTolerance": k_pos_tolerance}],
    )

    ide_arm_bt_node = Node(
        package="nhk2026_system",
        executable="ide_arm_bt_node",
        name="ide_arm_bt_node",
        output="screen",
        parameters=[{"bt_xml_file": bt_xml_file}],
    )

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
            "bt_start_delay",
            default_value="4.0",
            description="Delay in seconds before starting ide_arm_bt_node.",
        ),
        DeclareLaunchArgument(
            "k_pos_tolerance",
            default_value="0.03",
            description="Joint position tolerance used by ide_arm_action_server.",
        ),
        arm_path_plan_node,
        ide_arm_action_server_node,
        TimerAction(
            period=bt_start_delay,
            actions=[ide_arm_bt_node],
        ),
    ])
