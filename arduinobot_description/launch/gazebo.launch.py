from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_arduinobot_descripton = get_package_share_directory('arduinobot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    urdf_arg = DeclareLaunchArgument(
        name='model',
        default_value='arduinobot.urdf.xacro',
        description='Name of the URDF file to load'
    )

    urdf_file_path = PathJoinSubstitution([pkg_arduinobot_descripton, "urdf", LaunchConfiguration('model')])
    
    gz_world_arg = DeclareLaunchArgument(
        name='world', 
        default_value='empty.sdf',
        description='Name of the Gazebo world file to load'
    )
    world_file_path = PathJoinSubstitution([pkg_arduinobot_descripton, "worlds", LaunchConfiguration('world')])

    gz_config_arg = DeclareLaunchArgument(
        name='config',
        default_value='basic.config',
        description='Name of the gz client configuration file to load'
    )
    config_file_path = PathJoinSubstitution([pkg_arduinobot_descripton, "config", LaunchConfiguration('config')])
    
    time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='true',
        description='Setting time based of simulation'
    )

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_arduinobot_descripton).parent.resolve())]
    )

    gz_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': [world_file_path,
            TextSubstitution(text=' --gui-config '), config_file_path,
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
             "-x", "0.0", "-y", "0.0", "-z", "0.0", "-Y", "0.0"  # Initial spawn position
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
            # "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            # "/rgb_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            # "/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # # Launches rviz with the specified robot model
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=[
    #         '-d', os.path.join(pkg_arduinobot_descripton, 
    #                            'rviz', 'display.rviz')
    #     ],
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #     ]
    # )
    
    return LaunchDescription([
        urdf_arg,
        gz_world_arg,
        gz_config_arg,
        time_arg,
        gz_resource_path,
        gz_launch,
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_bridge_node,
        # rviz_node
    ])
   