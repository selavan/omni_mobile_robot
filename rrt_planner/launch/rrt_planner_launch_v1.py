from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rrt_planner',
            executable='rrt_planner_launch_v1',
            name='rrt_planner_launch_v1',
            output='screen',
            parameters=[
                {'start_x': 0.0},
                {'start_y': 0.0},
                {'goal_x': 5.0},
                {'goal_y': 5.0},
                {'expand_dis': 0.3},
                {'path_resolution': 0.1},
                {'goal_sample_rate': 5},
                {'max_iter': 500},
                {'robot_radius': 0.2},
                {'rand_area_min': -2.0},
                {'rand_area_max': 10.0}
            ],
            remappings=[
                ('/goal_pose', '/goal_pose'),
                ('/map', '/map'),
                ('/odom', '/odom'),
                ('/planned_path', '/planned_path')
            ]
        ),
    ])
