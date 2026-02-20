from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    urg_node2_nl_pkg = get_package_share_directory('urg_node2_nl')

   
    urg_node_front = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name='urg_node_front',                    
        remappings=[('scan', 'scan_front')],    
        parameters=[PathJoinSubstitution([urg_node2_nl_pkg, "config", "params_ether.yaml"])],
        output='screen',
    )

    
    urg_node_rear = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name='urg_node_rear',                    
        remappings=[('scan', 'scan_back')],       
        parameters=[PathJoinSubstitution([urg_node2_nl_pkg, "config", "params_ether_2nd.yaml"])],
        output='screen',
    )

    return LaunchDescription([
        urg_node_front,
        urg_node_rear,
    ])