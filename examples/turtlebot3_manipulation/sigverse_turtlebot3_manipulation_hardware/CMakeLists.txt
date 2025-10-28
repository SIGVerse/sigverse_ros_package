cmake_minimum_required(VERSION 3.5)
project(sigverse_turtlebot3_manipulation_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(sigverse_turtlebot3_system SHARED src/sigverse_turtlebot3_system.cpp)
ament_target_dependencies(sigverse_turtlebot3_system hardware_interface pluginlib rclcpp realtime_tools)

target_include_directories(sigverse_turtlebot3_system PUBLIC include)
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS sigverse_turtlebot3_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(sigverse_turtlebot3_system)
ament_package()
