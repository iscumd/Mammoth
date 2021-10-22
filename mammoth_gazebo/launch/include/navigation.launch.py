# MIT License
#
# Copyright (c) 2021 Intelligent Systems Club
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ROS Packages
    pkg_mammoth_gazebo = get_package_share_directory('mammoth_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_params_file_path = os.path.join(pkg_mammoth_gazebo, 'config', 'slam_online_async.yaml')
    slam_params_file = LaunchConfiguration('slam_params_file', default=slam_params_file_path)

    nav2_params_file_path = os.path.join(pkg_mammoth_gazebo, 'config', 'nav2_params.yaml')
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=nav2_params_file_path)

    # Nodes

    async_slam_toolbox = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': 'navigation',
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': nav2_params_file
        }.items(),
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file_path,
                              description='The file path of the params file for SLAM ToolBox'),

        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params_file_path,
                              description='The file path of the params file for Navigation2'),

        # Nodes
        async_slam_toolbox,
        nav2_stack,  # 5/23/21 dcutting133: currently this is very buggy.
        # i have tried running navigation2 compiled from source to get
        # rid of weirdness, and it kinda didnt work?
    ])
