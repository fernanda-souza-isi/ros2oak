from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakrgb',
            executable='rgb_publisher',
            name='oakrgb_node',
            parameters=[{'frame_rate': 5.0}], 
            output='screen'
        )
    ])
