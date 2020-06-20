cmake_minimum_required(VERSION 3.0.2)
project(mammoth_simulation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  roscpp
  rospy
)

catkin_package(
    DEPENDS
        gazebo_ros
#  INCLUDE_DIRS include
#  LIBRARIES mammoth_simulation
#  CATKIN_DEPENDS gazebo_ros roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
