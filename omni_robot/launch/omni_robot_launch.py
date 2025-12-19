from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    description_pkg_share = FindPackageShare('omni_robot_description').find('omni_robot_description')
    lidar_pkg_share = FindPackageShare('rtf_lds_driver').find('rtf_lds_driver')

    #description_launch_file = os.path.join(description_pkg_share, 'launch', 'display.launch.py')
    
    lidar_launch_file = os.path.join(lidar_pkg_share, 'launch', 'hlds_laser.launch.py')

    #robot_description_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(description_launch_file),
    #)
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file),
    )

    return LaunchDescription([
        #robot_description_launch,
        lidar_launch,
        
        DeclareLaunchArgument(
            'run_ps4',
            default_value='false',
            description='run ps4 node'
        ),
        #Node(
        #    package='can_bus',
        #    executable='can_bus_node',
        #    name='can_bus_node'
        #),
        Node(
            package='omni_robot',
            executable='body_velocity_node',
            name='body_velocity_node'
        ),
        #Node(
        #   package='map_publisher',
        #    executable='map_publisher_node',
        #    name='map_publisher_node'
        #),
        Node(
            package='ds4_driver',
            executable='ds4_driver_node.py',
            name='ds4_driver_node',
            #condition=IfCondition(LaunchConfiguration('run_ps4'))
        ),
        Node(
            package='ds4_teleop',
            executable='controller_node',
            name='controller_node',
            #condition=IfCondition(LaunchConfiguration('run_ps4'))
            #if the above line comment, it will be autonomous
        ),
        #Node(
        #    package='ds4_teleop',
        #    executable='controller_node_without_imu',
        #    name='controller_node_without_imu',
            #condition=IfCondition(LaunchConfiguration('run_ps4'))
            #if the above line comment, it will be autonomous
        #),
        Node(
            package='hfi_a9',
            executable='hfi_a9_node',
            name='hfi_a9_node'
        ),
        Node(
            package='omni_robot',
            executable='odom_node',
            name='odom_node'
        ),
        Node(
            package='can_motor',
            executable='can_motor_node',
            name='can_motor_node'
        ),
    ])

    
    
    
    
    
