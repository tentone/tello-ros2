from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello',
            executable='tello',
            namespace='/',
            name='tello',
            respawn=True,
            output='screen'
        )
    ])