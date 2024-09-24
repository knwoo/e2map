cmake_minimum_required(VERSION 3.0.2)
project(foundations)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  goal_selector_web.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
