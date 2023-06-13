cmake_minimum_required(VERSION 2.8.3)
project(sigverse_tools)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sigverse_ros_bridge
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


link_directories(/usr/local/lib)

add_executable(transform_transferer src/transform_transferer.cpp)
target_link_libraries(transform_transferer ${catkin_LIBRARIES})
