cmake_minimum_required(VERSION 2.8.3)
project(mammoth_snowplow)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/waypoint_class.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )

add_executable(fake_laser_node src/map_laser_scan.cpp)
target_link_libraries(fake_laser_node ${catkin_LIBRARIES} )

add_executable(transform_publisher_node src/transform_publisher.cpp)
target_link_libraries(transform_publisher_node ${catkin_LIBRARIES} )
