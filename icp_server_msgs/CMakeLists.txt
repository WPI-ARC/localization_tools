cmake_minimum_required(VERSION 2.8.3)
project(icp_server_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation sensor_msgs std_msgs shape_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  ICPRequest.msg
  ICPResponse.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  RunICP.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  shape_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES icp_server_msgs
  CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs shape_msgs
#  DEPENDS system_lib
)

