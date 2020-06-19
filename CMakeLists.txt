cmake_minimum_required(VERSION 2.8.3)
project(Mammoth)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options("-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  isc_joy
  isc_roboteq_hdc2460
  isc_ouster
  robot_state_controller
  mammoth_snowplow
  mammoth_simulation
)

catkin_package(
  CATKIN_DEPENDS
  isc_joy
  isc_roboteq_hdc2460
  isc_ouster
  robot_state_controller
  mammoth_snowplow
  mammoth_simulation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
