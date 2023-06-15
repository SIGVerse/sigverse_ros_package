cmake_minimum_required(VERSION 2.8.3)
project(tmc_suction)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
  rostest)
add_action_files(
  DIRECTORY action FILES SuctionControl.action
)
generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime roscpp std_msgs actionlib_msgs rostest
    INCLUDE_DIRS include
    LIBRARIES
    CFG_EXTRAS
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(suction_server src/suction_server.cpp)
add_dependencies(suction_server ${catkin_EXPORTED_TARGETS} ${tmc_suction_EXPORTED_TARGETS})

target_link_libraries(suction_server ${catkin_LIBRARIES})

install(TARGETS suction_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

if (CATKIN_ENABLE_TESTING)
  add_rostest_gtest(
    suction_server_test
    test/suction_server-test.test
    test/suction_server-test.cpp)

  target_link_libraries(suction_server_test ${catkin_LIBRARIES})
endif()
