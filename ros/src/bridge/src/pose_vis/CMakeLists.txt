add_executable(pose_vis 
  pose_vis.cpp
)

target_compile_options(pose_vis PRIVATE -Wall -Wpedantic)

target_include_directories(pose_vis PRIVATE
  ${FASTSENSE_DIR}/src
  ${FASTSENSE_ROS_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(pose_vis
  ${catkin_LIBRARIES}
)