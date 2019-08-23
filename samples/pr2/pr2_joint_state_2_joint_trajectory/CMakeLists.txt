cmake_minimum_required(VERSION 2.8.3)
project(pr2_joint_state_2_joint_trajectory)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

if (pr2_controllers_msgs_FOUND)
  find_package(pr2_controllers_msgs REQUIRED)
endif()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})


if (pr2_controllers_msgs_FOUND)
  add_executable(pr2_joint_state_2_joint_trajectory src/pr2_joint_state_2_joint_trajectory.cpp)
  target_link_libraries(pr2_joint_state_2_joint_trajectory ${catkin_LIBRARIES})
endif()

