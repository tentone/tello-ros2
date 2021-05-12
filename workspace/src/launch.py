
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Tello driver node
        Node(
            package='tello',
            executable='tello',
            output='screen',
            namespace='/',
            name='tello',
            parameters=[
                {'connect_timeout': 10.0},
                {'tello_ip': '192.168.10.1'},
                {'tf_base': 'map'},
                {'tf_drone': 'drone'}
            ],
            remappings=[],
            respawn=False
        ),
        # Tello control node
        Node(
            package='tello_control',
            executable='tello_control',
            namespace='/',
            name='control',
            output='screen'
        ),
        # RQT topic debug tool
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            namespace='/',
            name='rqt',
            respawn=True
        ),
        # RViz data visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            namespace='/',
            name='rviz2',
            respawn=True,
            arguments=['-d', '/home/tentone/Git/tello-slam/workspace/src/rviz.rviz']
        ),

        # Static TF publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='/',
            name='tf',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'drone'],
            respawn=True
        )
        # ORB SLAM
        Node(
            package='ros2_orbslam',
            executable='mono',
            output='screen',
            namespace='/',
            name='rqt',
            respawn=True
        ),
        # Camera calibration node
        # Node(
        #     package='camera_calibration',
        #     executable='cameracalibrator',
        #     output='screen',
        #     respawn=True,
        #     namespace='/',
        #     name='calibration',
        #     arguments=['--size', '7x9', '--square', '0.20'],
        #     parameters=[
        #         {'image': '/image_raw'},
        #         {'camera': '/camera_info'}
        #     ]
        # )
    ]


    return LaunchDescription(nodes)