cmake_minimum_required(VERSION 2.8.3)
project(tennis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ball_state_msgs
  hrpsys_ros_bridge
  hrpsys_choreonoid
  hrpsys_choreonoid_tutorials
  jvrc_models
  roseus
)

find_package(cmake_modules REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp rospy ball_state_msgs message_runtime
)

set(JVRC_RTC_DIRECTORY ${hrpsys_choreonoid_SOURCE_PREFIX})
set(JVRC_CONF_DIRECTORY ${hrpsys_choreonoid_tutorials_SOURCE_PREFIX}/config)
set(JVRC_MODELS_DIRECTORY ${jvrc_models_SOURCE_PREFIX})

configure_file(${PROJECT_SOURCE_DIR}/model/grass_court.body.in ${PROJECT_SOURCE_DIR}/model/grass_court.body @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_TENNIS.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_TENNIS.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_FOREHAND.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_FOREHAND.cnoid @ONLY)
