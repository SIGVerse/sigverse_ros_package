cmake_minimum_required(VERSION 3.5)
project(sigverse_turtlebot3_manipulation_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
