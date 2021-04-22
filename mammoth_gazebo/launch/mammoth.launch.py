import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

#import xarco

#xacro_file = os.path.join(urdf_dir, 'test-desc.urdf.xacro')
#doc = xacro.process_file(xacro_file)
#robot_desc = doc.toprettyxml(indent='  ')
# https://answers.ros.org/question/361623/ros2-robot_state_publisher-xacro-python-launch/

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
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r ' + pkg_mammoth_gazebo + '/worlds/test.sdf'
        }.items(),
    )

    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/mammoth/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/mammoth/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/model/mammoth/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
        output='screen'
    )

    ign_spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'mammoth',
            '-x', '0',
            '-z', '0.25',
            '-Y', '0',
            '-file', urdf_file
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_mammoth_description, 'rviz', 'mammoth_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='Open rviz if true'
        ),

        # Nodes
        robot_state_publisher,
        joint_state_publisher,

        ign_gazebo,
        ign_bridge,
        ign_spawn_robot,
        
        rviz,
    ])