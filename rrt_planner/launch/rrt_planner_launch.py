from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rrt_planner',
            executable='rrt_planner',
            name='rrt_planner',
            output='screen',
            parameters=[
                {'start_x': 0.0},
                {'start_y': 0.0},
                {'goal_x': 10.0},
                {'goal_y': 10.0}
            ],
            remappings=[
                ('/goal_pose', '/goal_pose'),
                ('/initialpose', '/initialpose'),
                ('/map', '/map')
            ]
        ),
    ])
