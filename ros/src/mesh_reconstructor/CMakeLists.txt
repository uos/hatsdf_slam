cmake_minimum_required(VERSION 3.0.2)
project(mesh_reconstructor)

add_compile_options(-std=c++17)

include(cmake/helpers.cmake)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(LVR2 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX C HL)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs message_filters
)

set_fastsense_root_dir(FASTSENSE_DIR)
get_absolute(FASTSENSE_ROS_INCLUDE_PATH include)

include_directories(
  ${FASTSENSE_DIR}/src
  ${catkin_INCLUDE_DIRS}
  ${FASTSENSE_DIR}/ext/HighFive/include
  ${HDF5_INCLUDE_DIRS}
)

set(HIGHFIVE_EXAMPLES FALSE CACHE INTERNAL "")
set(HIGHFIVE_UNIT_TESTS FALSE CACHE INTERNAL "")

if (NOT TARGET HighFive)
    add_subdirectory(${FASTSENSE_DIR}/ext/HighFive build)
endif()

add_executable(mesh_reconstructor src/mesh_reconstructor.cpp)
target_link_libraries(mesh_reconstructor
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${LVR2_LIBRARIES}
)
