from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publisher',
            executable='publisher',
            namespace='/',
            name='publisher'
        ),
        Node(
            package='subscriber',
            executable='subscriber',
            namespace='/',
            name='subscriber'
        )
    ])