cmake_minimum_required(VERSION 3.0.2)
project(experiment)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/goal_selector_wrapper.py
  src/success_counter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)