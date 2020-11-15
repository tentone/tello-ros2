from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publisher',
            namespace='publisher',
            executable='publisher',
            name='sim'
        ),
        Node(
            package='subscriber',
            namespace='subscriber',
            executable='subscriber',
            name='sim'
        )
    ])