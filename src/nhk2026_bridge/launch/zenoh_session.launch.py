from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    name_space = "r1"

    debug_listener = Node(
        package="demo_nodes_py",
        executable="listener",
        emulate_tty=True,
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

    zenoh_config = Path(
        get_package_share_directory("nhk2026_bridge")
    ) / "config" / "rmw_zenoh_config_lxd.json5"

    set_env_var = SetEnvironmentVariable(
        "ZENOH_SESSION_CONFIG_URI",
        str(zenoh_config),
    )

    return LaunchDescription([
        set_zenoh_env,
        set_env_var,
        zenoh_node,
        debug_listener,
    ])