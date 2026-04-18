from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    name_space = "r1"

    unity_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
        namespace=name_space,
    )

    zenoh_node = Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        emulate_tty=True,
        namespace=name_space,
    )

    set_zenoh_env = SetEnvironmentVariable(
        "RMW_IMPLEMENTATION",
        "rmw_zenoh_cpp",
    )

    router_config = Path(
        get_package_share_directory("nhk2026_bridge")
    ) / "config" / "rmw_zenoh_config_router.json5"

    set_router_config = SetEnvironmentVariable(
        "ZENOH_ROUTER_CONFIG_URI",
        str(router_config),
    )

    session_config = Path(
        get_package_share_directory("nhk2026_bridge")
    ) / "config" / "rmw_zenoh_config_local.json5"

    set_session_config = SetEnvironmentVariable(
        "ZENOH_SESSION_CONFIG_URI",
        str(session_config),
    )

    return LaunchDescription([
        set_zenoh_env,
        set_router_config,
        set_session_config,
        unity_endpoint,
        zenoh_node,
    ])