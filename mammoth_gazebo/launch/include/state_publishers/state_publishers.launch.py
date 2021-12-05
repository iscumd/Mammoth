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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_robot_model(pkg_description):
    urdf_dir = os.path.join(pkg_description, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'mammoth.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    return robot_desc, urdf_file


def generate_launch_description():
    # ROS packages
    pkg_mammoth_description = get_package_share_directory('mammoth_description')
    pkg_mammoth_gazebo = get_package_share_directory('mammoth_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_desc, urdf_file = generate_robot_model(pkg_mammoth_description)

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock if true'),       
        # Nodes
        robot_state_publisher,
        joint_state_publisher,
    ])