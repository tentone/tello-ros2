from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            namespace='control',
            executable='control',
            name='sim'
        ),
        Node(
            package='echo',
            namespace='echo',
            executable='echo',
            name='sim'
        )
    ])