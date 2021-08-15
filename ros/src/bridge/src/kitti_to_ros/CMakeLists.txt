add_executable(kitti_to_ros 
kitti_to_ros.cpp
)

target_compile_options(kitti_to_ros PRIVATE -Wall -Wpedantic)

target_include_directories(kitti_to_ros PRIVATE
  ${FASTSENSE_DIR}/src
  ${FASTSENSE_ROS_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(kitti_to_ros
  ${catkin_LIBRARIES}
)