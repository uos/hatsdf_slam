if (DEFINED ENV{XILINX_VITIS})
 message(FATAL_ERROR "Can't compile in bash session in which Vitis Tools have been sourced!\nCompile from a clean bash environment")
endif()

cmake_minimum_required(VERSION 3.0.2)
project(evaluation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

find_package(PCL REQUIRED)

add_message_files(
    FILES
    SavePoseStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES evaluation
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

add_executable(save_poses_node src/save_poses_node.cpp)
target_include_directories(save_poses_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(save_poses_node PRIVATE ${catkin_LIBRARIES})
add_dependencies(save_poses_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(loam_to_pose_node src/loam_to_pose_node.cpp)
target_include_directories(loam_to_pose_node PRIVATE ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(loam_to_pose_node PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(loam_to_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pcl2pcl2 src/pcl2pcl2.cpp)
target_include_directories(pcl2pcl2 PRIVATE ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl2pcl2 PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcl2pcl2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
