import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import xacro

def generate_launch_description():
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # remappings = [('/tf', 'tf'),
    #             ('/tf_static', 'tf_static')]
    # pkg_share = FindPackageShare(package='omni_robot_navigation').find('omni_robot_navigation')
    share_dir = get_package_share_directory('omni_robot_navigation')
    xacro_file = os.path.join(share_dir, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'robot.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('omni_robot_navigation'),
        'urdf',
        urdf_file_name)
    rviz_config_dir = os.path.join(get_package_share_directory('omni_robot_navigation'),
                                   'rviz')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    model = LaunchConfiguration('model')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

    declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=urdf, 
    description='Absolute path to robot urdf file')

    start_joint_state_publisher_cmd = Node(
    # condition=UnlessCondition(gui),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    # remappings=[("/robot_description", "/unicycle_bot_robot_description")]
    )
    joint_state_publisher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui'
  )


    return LaunchDescription([
        
    # DeclareLaunchArgument(
    # 'use_sim_time',
    # default_value='true',
    # description='Use simulation (Gazebo) clock if true'),

    start_joint_state_publisher_cmd,
    declare_use_robot_state_pub_cmd,
    declare_model_path_cmd,
    joint_state_publisher_gui_node,

        Node(
            condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 
            'robot_description': robot_urdf}],
            # remappings=remappings,
            arguments=[xacro_file]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        

#         Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         output='screen',
#         arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
#     ),
#      Node(
#   package='gazebo_ros',
#   executable='spawn_entity.py',
#   arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
#   output='screen'
#   ),
        #   Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '-0.0325', '0', '0', '0', 'base_link', 'base_footprint'],
        #     output='screen'
        # ),
        
        IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    # condition=IfCondition(use_simulator),
    # launch_arguments={'world': world}.items()
    ),
    
    

])
