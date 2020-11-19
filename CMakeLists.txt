cmake_minimum_required(VERSION 2.8.3)
project(tennis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ball_state_msgs
  hrpsys_ros_bridge
  roseus
)

find_package(cmake_modules REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp rospy ball_state_msgs
)

