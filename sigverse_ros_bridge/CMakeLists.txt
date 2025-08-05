cmake_minimum_required(VERSION 3.5)
project(sigverse_ros_bridge)

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
find_package(tf2_ros REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  thread
)

include_directories(
  include
  /usr/local/include
  /usr/local/include/bsoncxx/v_noabi
)

link_directories(/usr/local/lib)

add_executable(sigverse_ros_bridge src/sigverse_ros_bridge.cpp)

target_link_libraries(sigverse_ros_bridge
  mongocxx
  bsoncxx
)

ament_target_dependencies(sigverse_ros_bridge rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros)

install(
  TARGETS sigverse_ros_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

