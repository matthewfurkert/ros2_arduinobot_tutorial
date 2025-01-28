import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_arduinobot_description = get_package_share_directory('arduinobot_description')

    urdf_arg = DeclareLaunchArgument(
        'model', default_value='arduinobot.urdf.xacro',
        description='Name of the URDF file to load'
    )
    urdf_file_path = PathJoinSubstitution(
        [pkg_arduinobot_description, "urdf",
         LaunchConfiguration('model')]
    )

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
        parameters=[{'robot_description': Command(['xacro', ' ', urdf_file_path])}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", 
            "/controller_manager", 
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller", 
            "--controller-manager", 
            "/controller_manager", 
        ]
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller", 
            "--controller-manager", 
            "/controller_manager", 
        ]
    )

    return LaunchDescription([
        urdf_arg,
        is_sim_arg,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])