import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    continuum_robot_path = get_package_share_directory('continuum_robot')

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="False", description="Enable GUI"),
        
        launch_ros.actions.Node(
            package="continuum_robot",
            executable="core_node",
            name="core_node",
            output="screen",
            parameters=[{
                "robot_description": os.path.join(continuum_robot_path, "urdf", "robot_model.urdf"),
                "number_of_sections": 1
            }]
        ),
        
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", os.path.join(continuum_robot_path, "urdf", "continuum.rviz")],
            output="screen"
        ),
    ])
