# # MIT License
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

# import xarco

# xacro_file = os.path.join(urdf_dir, 'test-desc.urdf.xacro')
# doc = xacro.process_file(xacro_file)
# robot_desc = doc.toprettyxml(indent='  ')
# https://answers.ros.org/question/361623/ros2-robot_state_publisher-xacro-python-launch/


def generate_robot_model(pkg_description):
    urdf_dir = os.path.join(pkg_description , 'urdf')
    urdf_file = os.path.join(urdf_dir, 'mammoth.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    return robot_desc, urdf_file


def generate_launch_description():
    # ROS packages
    pkg_mammoth_description = get_package_share_directory('mammoth_description')
    pkg_mammoth_snowplow = get_package_share_directory('mammoth_snowplow')
    pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

    # Config
    joy_config = os.path.join(pkg_mammoth_snowplow, 'config/joystick', 'xbone.config.yaml')
    laserscan_config = os.path.join(pkg_mammoth_snowplow, 'config/Sensors/Lidar', 'pointcloud_to_laserscan.yaml')
    roboteq_config = os.path.join(pkg_mammoth_snowplow, 'config/Roboteq', 'roboteq.yaml')   
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    robot_desc, urdf_file = generate_robot_model(pkg_mammoth_snowplow)

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
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/mammoth/filtered_points'),
            ('scan', '/scan')],
        parameters=[laserscan_config],
        name='pointcloud_to_laserscan'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_mammoth_description, 'rviz', 'mammoth_gazebo.rviz')],
        condition=IfCondition(use_rviz)
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

    lidar_processor = Node(
        package='lidar_processor',
        executable='lidar_processor',
        name='lidar_processor',
        output='screen',
        remappings=[
            ('/lidar/raw_points', '/mammoth/raw_points'),
            ('/lidar/filtered_points', '/mammoth/filtered_points'),
            ('/lidar/unfiltered_points', '/mammoth/unfiltered_points'),
            ('/lidar/raw_scan', '/mammoth/raw_scan'),
            ('/lidar/filtered_scan', '/mammoth/filtered_scan'),
            ('/lidar/unfiltered_scan', '/mammoth/unfiltered_scan'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    roboteq = Node(
        package='isc_roboteq',
        executable='isc_roboteq',
        name='isc_roboteq',
        output='screen',
        parameters=[roboteq_config],
    )
    
    mammoth_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mammoth_snowplow, 'launch/include', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )
    
    return LaunchDescription([
        # Launch Arguments
     
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Open rviz if true'),

        DeclareLaunchArgument(name='scanner', default_value='scanner',
                              description='Namespace for sample topics'),

        # Nodes
        robot_state_publisher,
        joint_state_publisher,
        joy_with_teleop_twist,
        lidar_processor,
        pointcloud_to_laserscan,
        mammoth_navigation,
        rviz,

        roboteq,
    ])
