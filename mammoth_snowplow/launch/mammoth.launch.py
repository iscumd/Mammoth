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


def generate_launch_description():
    # ROS packages
    pkg_mammoth_snowplow = get_package_share_directory('mammoth_snowplow')
    pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

    # Config
    joy_config = os.path.join(pkg_mammoth_snowplow, 'config/joystick', 'xbone.config.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Nodes
    state_publishers = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_mammoth_snowplow, 'launch'),
         '/include/state_publishers/state_publishers.launch.py']),
         launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )

    joy_with_teleop_twist = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         os.path.join(pkg_teleop_twist_joy, 'launch', 'teleop-launch.py')
      ),
      launch_arguments={
          'joy_config': 'xbox',
          'joy_dev': '/dev/input/js0',
          'config_filepath': joy_config
      }.items(),
    )

    roboteq = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_mammoth_snowplow, 'launch'),
         '/include/roboteq/roboteq.launch.py']),
         launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )

    ouster_os1 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_mammoth_snowplow, 'launch'),
         '/include/ouster_os1/os1.launch.py']),
         launch_arguments={
            'use_sim_time': use_sim_time
        }.items(), 
    )   

    realsense = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_mammoth_snowplow, 'launch'),
         '/include/realsense/rs_t265_launch.py']),
         launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )    

    lidar_processor = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_mammoth_snowplow, 'launch'),
         '/include/lidar_processor/lidar_processor.launch.py']),
         launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )

    pointcloud_to_laserscan = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_mammoth_snowplow, 'launch'),
         '/include/pointcloud_to_laserscan/pointcloud_to_laserscan.launch.py']),
         launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )

    navigation = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_mammoth_snowplow, 'launch'),
         '/include/navigation/navigation.launch.py']),
         launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    ) 

    rviz = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_mammoth_snowplow, 'launch'),
         '/include/rviz/rviz.launch.py']),
         launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
         }.items(),
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),

        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Open rviz if true'),

        # Nodes
        state_publishers,
        joy_with_teleop_twist,
        roboteq,
        ouster_os1,
 
        realsense,
        lidar_processor,
        pointcloud_to_laserscan,
        navigation,

        rviz,
    ])