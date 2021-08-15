cmake_minimum_required(VERSION 3.0.2)
project(imu_test)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

find_package(ZeroMQ 4.3.2 REQUIRED EXACT)
find_package(cppzmq 4.4.1 REQUIRED EXACT)

catkin_package()

set(FASTSENSE_DIR ../../../../../fastsense)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${FASTSENSE_DIR}/src/driver/imu 
  ${FASTSENSE_DIR}/src
)

add_executable(${PROJECT_NAME}_node
  ${FASTSENSE_DIR}/src/driver/imu/api/phidget.cpp
  ${FASTSENSE_DIR}/src/driver/imu/imu.cpp
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  phidget21
  libzmq
)

