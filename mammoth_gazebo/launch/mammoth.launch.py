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

# import xarco

# xacro_file = os.path.join(urdf_dir, 'test-desc.urdf.xacro')
# doc = xacro.process_file(xacro_file)
# robot_desc = doc.toprettyxml(indent='  ')
#  https://answers.ros.org/question/361623/ros2-robot_state_publisher-xacro-python-launch/


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
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

    # Config arguments
    joy_config = os.path.join(pkg_mammoth_gazebo, 'config/joystick', 'xbone.config.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
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
    )

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={
            'ign_args': '-r ' + pkg_mammoth_gazebo + '/worlds/test.sdf'
        }.items(),
    )

    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/world/test/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/model/mammoth/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                   '/model/mammoth/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   '/model/mammoth/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                   '/model/mammoth/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                   '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                   '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
        output='screen',
        remappings=[
            ('/world/test/clock', '/clock'),
            ('/model/mammoth/tf', '/tf'),
            ('/model/mammoth/cmd_vel', '/cmd_vel'),
            ('/model/mammoth/odometry', '/mammoth/odom'),
            ('/model/mammoth/joint_state', 'joint_states'),
            ('/lidar', '/mammoth/raw_scan'),
            ('/lidar/points', '/mammoth/raw_points'),
            ('/imu', '/mammoth/imu'),
        ]
    )

    ign_spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'mammoth',
            '-x', '0',
            '-z', '0',
            '-Y', '0',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/mammoth/unfiltered_points'),
            ('scan', '/scan')],
        parameters=[{
             'target_frame': 'laser_link',
             'transform_tolerance': 0.01,
             'min_height': 0.0,
             'max_height': 20.0,
             'angle_min': -1.5708,
             'angle_max':  1.5708,
             'angle_increment': 0.0087,
             'scan_time': 0.01,
             'range_min': 0.45,
             'range_max': 35.0,
             'use_inf': False,
             'inf_epsilon': 1.0
        }],
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
        ]
    )

    mammoth_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mammoth_gazebo, 'launch/include', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items(),
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
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

        ign_gazebo,
        ign_bridge,
        ign_spawn_robot,

        mammoth_navigation,

        rviz,
    ])
