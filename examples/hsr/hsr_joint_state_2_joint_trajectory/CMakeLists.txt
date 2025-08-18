cmake_minimum_required(VERSION 3.5)
project(hsr_joint_state_2_joint_trajectory)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(hsr_joint_state_2_joint_trajectory src/hsr_joint_state_2_joint_trajectory.cpp)
ament_target_dependencies(hsr_joint_state_2_joint_trajectory rclcpp std_msgs sensor_msgs trajectory_msgs)

install(
  TARGETS hsr_joint_state_2_joint_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

