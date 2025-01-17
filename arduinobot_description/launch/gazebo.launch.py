import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    
    urdf_arg = DeclareLaunchArgument(name="urdf_path", default_value=os.path.join(get_package_share_directory(
                         'arduinobot_description'), 'urdf', 'arduinobot.urdf.xacro'))
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("urdf_path")]), value_type=str)
    
    gazebo_resoure_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(get_package_share_directory('arduinobot_description')).parent.resolve())
        ]
    )
    
    ros_distro = os.environ["ROS_DISTRO"]
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"
    
    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": True}]
        )

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py"]
            ),
            launch_arguments=[
                ("gz_args", [" -v 4 -r empty.sdf", physics_engine])
            ]
        )
    
    gazebo_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "arduinobot"]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            ]
    )
    
    return LaunchDescription([
        urdf_arg,
        gazebo_resoure_path,
        robot_state_publisher,
        gazebo,
        gazebo_spawn_entity,
        gz_ros2_bridge
    ])
    


