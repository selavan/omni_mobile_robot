from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rrt_star_planner',
            executable='rrt_star_planner',
            name='rrt_star_planner'
        ),
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_world',
            name='turtlebot3_world',
            output='screen'
        ),
        Node(
            package='turtlebot3_navigation2',
            executable='navigation2',
            name='navigation2',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '<path_to_rviz_config>']
        )
    ])
