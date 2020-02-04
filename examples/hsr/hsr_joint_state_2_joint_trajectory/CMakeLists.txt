cmake_minimum_required(VERSION 2.8.3)
project(hsr_joint_state_2_joint_trajectory)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(hsr_joint_state_2_joint_trajectory src/hsr_joint_state_2_joint_trajectory.cpp)
target_link_libraries(hsr_joint_state_2_joint_trajectory ${catkin_LIBRARIES})
