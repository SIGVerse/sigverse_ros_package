cmake_minimum_required(VERSION 2.8.3)
project(pr2_joint_state_2_joint_trajectory)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  pr2_controllers_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pr2_joint_state_2_joint_trajectory src/pr2_joint_state_2_joint_trajectory.cpp)
target_link_libraries(pr2_joint_state_2_joint_trajectory ${catkin_LIBRARIES})
