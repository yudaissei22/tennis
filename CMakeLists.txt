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
  CATKIN_DEPENDS roscpp rospy ball_state_msgs message_runtime
)

configure_file(${PROJECT_SOURCE_DIR}/launch/tennis.launch.in ${PROJECT_SOURCE_DIR}/launch/tennis.launch @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/model/grass_court.body.in ${PROJECT_SOURCE_DIR}/model/grass_court.body @ONLY)
