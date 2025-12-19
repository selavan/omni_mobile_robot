from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_launch_dir = get_package_share_directory('gazebo_ros')
    world_file = get_package_share_directory('my_dynamic_environment') + '/worlds/my_dynamic_world.world'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/launch/gazebo.launch.py']),
            launch_arguments={'world': world_file}.items(),
        ),
    ])
