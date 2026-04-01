from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ns = 'r1'

    joy_controller_node = Node(
        package="nhk2026_localization",
        executable="joy_r1",
        name="joy_r1",
        namespace=ns,
        output="screen"
    )

   
    nakamura_hand_server = Node(
        package='nhk2026_system',
        executable='nakamura_hand_server',
        name='nakamura_hand_sequencer',
        namespace=ns,
        output='screen'
    )
    return LaunchDescription([
        joy_controller_node,
        nakamura_hand_server,
        
    ])