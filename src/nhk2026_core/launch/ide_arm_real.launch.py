import os
import subprocess
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnProcessStart
import lifecycle_msgs.msg
import launch

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
    bt_xml_file = LaunchConfiguration("bt_xml_file")
    bt_start_delay = LaunchConfiguration("bt_start_delay")
    wait_for_server_timeout_ms = LaunchConfiguration("wait_for_server_timeout_ms")
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

    vacuum_server_node = Node(
        package="nhk2026_control",
        executable="vacuum_server",
        name="vacuum_server",
        output="screen",
    )

    ide_arm_bt_node = Node(
        package="nhk2026_system",
        executable="ide_arm_bt_node",
        name="ide_arm_bt_node",
        output="screen",
        parameters=[
            {"bt_xml_file": bt_xml_file},
            {"wait_for_server_timeout_ms": wait_for_server_timeout_ms},
        ],
    )

    pkg_share_bridge = get_package_share_directory('nhk2026_bridge')
    canid_file = os.path.join(pkg_share_bridge, 'config', 'ide_arm_canbridge.yml')
    canbridgenode = LifecycleNode(
        package='nhk2026_bridge',
        executable='nhk2026_canbridge',
        name='nhk2026_canbridge_step',
        namespace='',
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
    
    sim_package_share = get_package_share_directory("nhk2026_sim")
    xacro_file = os.path.join(sim_package_share, "urdf", "ide_arm.xacro")
    doc = xacro.process_file(xacro_file, mappings={"use_sim": "true"})
    robot_desc = doc.toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_desc}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_ide_arm_node = Node(
        package='nhk2026_control',
        executable='joint_state_publisher_ide',
        name='joint_state_publisher_ide_arm',
        output='screen'
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
            "wait_for_server_timeout_ms",
            default_value="5000",
            description="Timeout in ms to wait for the ide_arm action server.",
        ),
        DeclareLaunchArgument(
            "k_pos_tolerance",
            default_value="0.03",
            description="Joint position tolerance used by ide_arm_action_server.",
        ),
        OpaqueFunction(function=_require_can0),
        OpaqueFunction(function=_ensure_can0_up),
        canbridgenode,
        canbridge_configure_event_handler,
        canbridge_activate_event_handler,
        arm_path_plan_node,
        ide_arm_action_server_node,
        vacuum_server_node,
        TimerAction(
            period=bt_start_delay,
            actions=[ide_arm_bt_node],
        ),
        joint_state_publisher_ide_arm_node,
        robot_state_publisher,
    ])