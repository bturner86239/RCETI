import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rceti_continuum_path = get_package_share_directory('rceti_continuum')

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="False", description="Enable GUI"),
        # Launch the core_node
        launch_ros.actions.Node(
            package="rceti_continuum",
            executable="core_node",
            name="core_node",
            output="screen",
            parameters=[{
                "robot_description": os.path.join(rceti_continuum_path, "urdf", "robot_model.urdf"),
                "number_of_sections": 2
            }]
        ),

        # Launch RViz
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", os.path.join(rceti_continuum_path, "urdf", "continuum.rviz")],
            output="screen"
        ),
    ])
