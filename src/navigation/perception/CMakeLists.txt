cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
    rospy    
    roscpp
    std_msgs
    sensor_msgs
    gazebo_msgs
    pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(collision_detection
 src/collision_detection.cpp
)
target_link_libraries(collision_detection
  ${catkin_LIBRARIES}
)

add_executable(collision_detection_gazebo
 src/collision_detection_gazebo.cpp
)
target_link_libraries(collision_detection_gazebo
  ${catkin_LIBRARIES}
)