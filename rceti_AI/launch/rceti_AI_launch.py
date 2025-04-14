from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rceti_AI',
            executable='rceti_AI',
            name='rceti_AI',
            parameters=[
                {'model_path': '/absolute/path/to/best.pt'},
                {'video_path': '0'}  # '0' for webcam
            ],
            output='screen'
        )
    ])


