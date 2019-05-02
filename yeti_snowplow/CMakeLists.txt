cmake_minimum_required(VERSION 2.8.3)
project(yeti_snowplow)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
# compile with C++11
# https://github.com/ros-planning/geometric_shapes/pull/50#issuecomment-236972774
add_compile_options("-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  isc_joy
  isc_roboteq_hdc2460
  isc_shared_msgs
  isc_sick
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_library(wiringPi wiringPi ./src/wiringPi)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  gpio_inputs.msg
  gpio_leds.msg
  lidar_point.msg
  location_point.msg
  obstacle.msg
  obstacles.msg
  target.msg
  turn.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  waypoint.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
  isc_joy
  isc_shared_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yeti_snowplow
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs isc_joy isc_roboteq_hdc2460 isc_shared_msgs isc_sick
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(yeti_snowplow
#   src/${PROJECT_NAME}/yeti_snowplow.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(yeti_snowplow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(yeti_snowplow_node src/yeti_snowplow_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(yeti_snowplow_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(yeti_snowplow_node
#   ${catkin_LIBRARIES}
# )

# ADD NEW CPPs HERE
add_executable(auto_control_logic src/auto_control_logic.cpp)
add_dependencies(auto_control_logic ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(auto_control_logic ${catkin_LIBRARIES})

add_executable(yeti_drive_mode_control src/yeti_drive_mode_control.cpp)
add_dependencies(yeti_drive_mode_control ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(yeti_drive_mode_control ${catkin_LIBRARIES})

#add_executable(navigation_pid src/navigation_pid.cpp)
#add_dependencies(navigation_pid ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(navigation_pid ${catkin_LIBRARIES})

add_executable(navigation_pid_speed src/navigation_pid_speed.cpp)
add_dependencies(navigation_pid_speed ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(navigation_pid_speed ${catkin_LIBRARIES})

add_executable(navigation_pid_turn src/navigation_pid_turn.cpp)
add_dependencies(navigation_pid_turn ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(navigation_pid_turn ${catkin_LIBRARIES})

add_executable(navigation_pid_velocity src/navigation_pid_velocity.cpp)
add_dependencies(navigation_pid_velocity ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(navigation_pid_velocity ${catkin_LIBRARIES})

add_executable(navigation_pid_test src/navigation_pid_test.cpp)
add_dependencies(navigation_pid_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(navigation_pid_test ${catkin_LIBRARIES})

add_executable(obstacle_detection src/obstacle_detection.cpp)
add_dependencies(obstacle_detection ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(obstacle_detection ${catkin_LIBRARIES})

add_executable(obstacle_reaction src/obstacle_reaction.cpp src/buffer.h)
add_dependencies(obstacle_reaction ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(obstacle_reaction ${catkin_LIBRARIES})

#add_executable(gpio_monitor src/gpio_monitor.cpp)
#add_dependencies(gpio_monitor ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(gpio_monitor ${catkin_LIBRARIES} ${wiringPi})

add_executable(yeti_waypoint_selection src/yeti_waypoint_selection.cpp)
add_dependencies(yeti_waypoint_selection ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(yeti_waypoint_selection ${catkin_LIBRARIES})

add_executable(localization_node src/localization_node.cpp)
add_dependencies(localization_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(localization_node ${catkin_LIBRARIES})

add_executable(stall_detection src/stall_detection_node.cpp)
add_dependencies(stall_detection ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(stall_detection ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS yeti_snowplow yeti_snowplow_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yeti_snowplow.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
