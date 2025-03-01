import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_arduinobot_description = get_package_share_directory('arduinobot_description')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world',
            default_value='mancave.sdf',
            description='Name of the Gazebo world file to load'
        ),
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_arduinobot_description, 'worlds', LaunchConfiguration('world')]),
                TextSubstitution(text=' -r -v -v1 --physics-engine gz-physics-bullet-featherstone-plugin')],
                'on_exit_shutdown': 'true'
            }.items()
        )
    ])