import os
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = FindPackageShare(package='omni_robot_navigation').find('omni_robot_navigation')

    # world_file_name = 'world12.sdf'
    world = os.path.join(get_package_share_directory('omni_robot_navigation'),
                         'world', 'world12.sdf')
    launch_file_dir = os.path.join(get_package_share_directory('omni_robot_navigation'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')
    
#     spawn_entity =Node(
#   package='gazebo_ros',
#   executable='spawn_entity.py',
#   arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
#   output='screen'
#   )
    return LaunchDescription([
#         Node(
#        package='robot_localization',
#        executable='ekf_node',
#        name='ekf_filter_node',
#        output='screen',
#        parameters=[os.path.join('foxy_description', 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
# ),
 Node(
  package='gazebo_ros',
  executable='spawn_entity.py',
  arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
  output='screen'
  ),
        DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator'),
        DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient'),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        #     ),
        #     launch_arguments={'world': world}.items(),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        #     ),
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        #     output='screen'),

# Start Gazebo server
        IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    # condition=IfCondition(use_simulator),
    # launch_arguments={'world': world}.items()
    ),

  # Start Gazebo client    
    #     IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    # condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))


        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/omo_r1mini_launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),
        # spawn_entity,
        #  robot_localization_node,

    ])
