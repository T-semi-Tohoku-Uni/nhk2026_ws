import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch

def generate_launch_description():
    pkg_share_bridge = get_package_share_directory('nhk2026_bridge')
    # CANブリッジの設定ファイル (ID 0x211-0x214 などが定義されているもの)
    canid_file = os.path.join(pkg_share_bridge, 'config', 'step_leg_canbridge.yml')

    ld = LaunchDescription()

    # 1. CANブリッジ (LifecycleNode)
    canbridgenode = LifecycleNode(
        package='nhk2026_bridge',
        executable='nhk2026_canbridge',
        name='nhk2026_canbridge_step',
        namespace='',
        parameters=[canid_file],
        output='screen'
    )

    # 2. 下位ノード (Worker: 実際に脚を動かす)
    step_leg_worker = Node(
        package='nhk2026_system',
        executable='step_leg_action_server',
        name='step_leg_worker',
        output='screen'
    )

    # 3. 上位ノード (Sequencer: 1〜6のシーケンス制御)
    step_leg_sequencer = Node(
        package='nhk2026_system',
        executable='step_action_server',
        name='step_leg_sequencer',
        output='screen'
    )

    # 4. Joint State Publisher (脚の現在角を配信)
    jsp_node = Node(
        package='nhk2026_control',
        executable='joint_state_publisher_step',
        name='joint_state_publisher_step',
        output='screen'
    )

    ld.add_action(canbridgenode)
    ld.add_action(jsp_node)
    ld.add_action(step_leg_worker)
    ld.add_action(step_leg_sequencer)

    # CANブリッジの自動有効化設定 (Lifecycle遷移)
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=canbridgenode,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(canbridgenode),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))])) )

    ld.add_action(RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=canbridgenode,
            start_state='configuring', goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(canbridgenode),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))])) )

    return ld