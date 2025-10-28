cmake_minimum_required(VERSION 3.5)
project(sigverse_turtlebot3)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# teleop_key
add_executable(turtlebot3_teleop_key src/turtlebot3_teleop_key.cpp)
ament_target_dependencies(turtlebot3_teleop_key rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs builtin_interfaces tf2_ros)

# recognize_pointed_direction
add_executable(recognize_pointed_direction src/recognize_pointed_direction.cpp)
ament_target_dependencies(recognize_pointed_direction rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs builtin_interfaces tf2_ros yolo_msgs visualization_msgs)

# grasping_auto
add_executable(grasping_auto src/grasping_auto.cpp)
ament_target_dependencies(grasping_auto rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs builtin_interfaces tf2_ros yolo_msgs visualization_msgs moveit_ros_planning_interface)

install(
  TARGETS turtlebot3_teleop_key 
  recognize_pointed_direction 
  grasping_auto
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

