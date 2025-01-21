import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_arduinobot_descripton = get_package_share_directory('arduinobot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_arg = DeclareLaunchArgument(
        'model', default_value='arduinobot.urdf.xacro',
        description='Name of the URDF file to load'
    )
    urdf_file_path = PathJoinSubstitution(
        [pkg_arduinobot_descripton, "urdf",
         LaunchConfiguration('model')]
    )
    
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Name of the Gazebo world file to load'
    )
    world_file_path = PathJoinSubstitution(
        [pkg_arduinobot_descripton, "worlds",
         LaunchConfiguration('world')]
    )
    
    time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Setting time based of simulation'
    )

    gz_resoure_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_arduinobot_descripton).parent.resolve())]
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [world_file_path,
        TextSubstitution(text=' -r -v -v1 --physics-engine gz-physics-bullet-featherstone-plugin')],
        'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro', ' ', urdf_file_path]),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
             "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"  # Initial spawn position
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge messages like /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/rgb_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Launches rviz with the specified robot model
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(pkg_arduinobot_descripton, 
                               'rviz', 'display.rviz')
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    ld = LaunchDescription()

    ld.add_action(urdf_arg)
    ld.add_action(world_arg)
    ld.add_action(time_arg)
    ld.add_action(gz_resoure_path)
    ld.add_action(gz_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_bridge_node)
    ld.add_action(rviz_node)
    
    return ld