cmake_minimum_required(VERSION 3.0.2)
project(dynamic_objects)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  geometry_msgs
  nav_msgs
  roscpp
  rospy
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    gazebo_ros_actor_command
    actor_follower_node
    door_plugin
  CATKIN_DEPENDS
    gazebo_ros
    gazebo_plugins
    geometry_msgs
    nav_msgs
    roscpp
    rospy
  DEPENDS
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_ros_actor_command src/gazebo_ros_actor_command.cpp)
target_link_libraries(gazebo_ros_actor_command ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(actor_follower_node src/actor_follower.cpp)
target_link_libraries(actor_follower_node ${catkin_LIBRARIES})

add_library(door_plugin src/DynamicDoorPlugin.cc)
target_link_libraries(door_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PROTOBUF_LIBRARY})

catkin_install_python(PROGRAMS
  src/reset_actor.py
  src/reset_go1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)