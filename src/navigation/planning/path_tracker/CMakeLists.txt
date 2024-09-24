cmake_minimum_required(VERSION 3.0.2)
project(path_tracker)

# add_compile_options(-std=c++17)
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  dynamic_reconfigure
  message_filters
)

generate_dynamic_reconfigure_options(
 config/dynamic_reconfig_for_mppi.cfg
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   visualization_msgs
# )

set(Eigen_INCLUDE_DIRS /usr/include/eigen3)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib std_msgs nav_msgs geometry_msgs visualization_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/misc.cpp
  src/cost.cpp
  src/state.cpp
  src/action.cpp
  src/transform.cpp
  src/dynamics.cpp
  src/path_tracker.cpp
  src/visualization.cpp
  src/communication.cpp
  src/dummy_path.cpp
  src/regression.cpp
)

add_executable(${PROJECT_NAME}_node src/path_tracker.cpp)

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
${PROJECT_NAME}_gencfg
)