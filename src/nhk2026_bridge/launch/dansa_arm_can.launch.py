import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, OpaqueFunction, Shutdown
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch

# --- CANハードウェア設定用関数 (変更なし) ---
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

def _run_cmd(cmd):
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return True, out
    except subprocess.CalledProcessError as e:
        return False, e.output
    
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
        ok, out = _run_cmd(cmd)
        if not ok:
            launch.logging.get_logger(__name__).error(
                "can0 設定に失敗しました。cmd: %s\n(output: %s)"
                % (" ".join(cmd), out.strip())
            )
            return [Shutdown(reason="can0 setup failed")]
    return []

# --- メインのLaunch構成 ---
def generate_launch_description():
    # ★1. YAMLファイルのパス指定 (事前に作成した段差用YAMLを指定)
    pkg_share_bridge = get_package_share_directory('nhk2026_bridge')
    canid_file = os.path.join(pkg_share_bridge, 'config', 'dansa_arm_canbridge.yml')

    ld = LaunchDescription()
    
    # CANインターフェースの自動セットアップを実行
    ld.add_action(OpaqueFunction(function=_require_can0))
    ld.add_action(OpaqueFunction(function=_ensure_can0_up))

    # ★2. CANブリッジノード（LifecycleNode）の定義
    canbridgenode = LifecycleNode(
        package='nhk2026_bridge',
        executable='nhk2026_canbridge',
        name='nhk2026_canbridge_dansa',
        namespace='',
        parameters=[canid_file],
        output='screen',
        emulate_tty=True, 
    )
    ld.add_action(canbridgenode)

    # ★3. 先ほど作った「段差用 JointStatePublisher」ノードの定義
    joint_state_publisher_dansa_node = Node(
        package='nhk2026_control',
        executable='joint_state_publisher_dansa',
        name='joint_state_publisher_dansa',
        output='screen'
    )
    ld.add_action(joint_state_publisher_dansa_node)

    # --- 以下、LifecycleNodeの自動状態遷移イベントハンドラ (変更なし) ---
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

    ld.add_action(canbridge_configure_event_handler)
    ld.add_action(canbridge_activate_event_handler)

    return ld