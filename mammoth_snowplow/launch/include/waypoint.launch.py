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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    pkg_mammoth_snowplow = get_package_share_directory('mammoth_snowplow')

    # Config
    waypoints = os.path.join(pkg_mammoth_snowplow, 'config/waypoints',
                             'single-I.csv')
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    follow_waypoints = LaunchConfiguration('follow_waypoints', default='true')

    # Nodes
    waypoint_publisher = Node(package='waypoint',
                              executable='waypoint',
                              name='waypoint',
                              output='screen',
                              parameters=[{
                                  'filename': waypoints
                              }, {
                                  'use_sim_time': use_sim_time
                              }],
                              condition=IfCondition(follow_waypoints))

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation time if true'),
        DeclareLaunchArgument('follow_waypoints',
                              default_value='true',
                              description='Follow waypoints if true'),
        # Nodes
        waypoint_publisher,
    ])
