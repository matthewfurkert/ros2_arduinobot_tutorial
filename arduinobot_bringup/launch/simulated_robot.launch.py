from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_arduinobot_description = get_package_share_directory('arduinobot_description')
    pkg_arduinobot_controller = get_package_share_directory('arduinobot_controller')
    pkg_arduinobot_moveit = get_package_share_directory('arduinobot_moveit')
    pkg_arduinobot_remote = get_package_share_directory('arduinobot_remote')

    gz_launch_path = PathJoinSubstitution([pkg_arduinobot_description, 'launch', 'gazebo.launch.py'])
    controller_launch_path = PathJoinSubstitution([pkg_arduinobot_controller, 'launch', 'controller.launch.py'])
    moveit_launch_path = PathJoinSubstitution([pkg_arduinobot_moveit, 'launch', 'moveit.launch.py'])
    remote_launch_path = PathJoinSubstitution([pkg_arduinobot_remote, 'launch', 'remote_interface.launch.py'])

    

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(remote_launch_path)),
    ])