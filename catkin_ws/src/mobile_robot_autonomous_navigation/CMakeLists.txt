cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot_autonomous_navigation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  hardware_interface
  controller_manager
  joint_limits_interface
  angles
  rosserial_python
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    hardware_interface
    controller_manager
    joint_limits_interface
    angles
    rosserial_python
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(robot_hw_interface_node
  src/robot_hardware_interface.cpp
)

## Specify libraries to link executable against
target_link_libraries(robot_hw_interface_node
  ${catkin_LIBRARIES}
)
