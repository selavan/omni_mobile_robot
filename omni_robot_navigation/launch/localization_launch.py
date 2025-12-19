# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml



def generate_launch_description():

    #############################################
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('omni_robot_navigation'),
            'map',
            'map012.yaml')) # map_dronp.yaml

    nav2_yaml=os.path.join(get_package_share_directory('omni_robot_navigation'),'config','localization.yaml')

    lifecycle_nodes = ['map_server', 'amcl']

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'yaml_filename':map_dir}]),
        Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
        ),

        Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
        ),
        
              Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0', '0.0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
    ])
