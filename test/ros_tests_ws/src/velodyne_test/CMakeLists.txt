cmake_minimum_required(VERSION 3.0.2)
project(velodyne_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(FASTSENSE_DIR ../../../../../fastsense)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(ZeroMQ 4.3.2 REQUIRED)
find_package(cppzmq 4.4.1 REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${FASTSENSE_DIR}/src
)

add_executable(${PROJECT_NAME}_node
  src/velodyne_test_node.cpp
  ${FASTSENSE_DIR}/src/driver/lidar/velodyne.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  libzmq
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
