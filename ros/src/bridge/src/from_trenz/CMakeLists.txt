add_executable(from_trenz
  imu_bridge.cpp
  tsdf_bridge.cpp
  velodyne_bridge.cpp
  bridge.cpp
  transform_bridge.cpp
)

target_compile_options(from_trenz PRIVATE ${OpenMP_FLAGS} -Wall -Wpedantic)

target_include_directories(from_trenz PRIVATE
  ${FASTSENSE_DIR}/src
  ${FASTSENSE_ROS_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_dependencies(from_trenz ${catkin_EXPORTED_TARGETS})

target_link_libraries(from_trenz
  ${catkin_LIBRARIES}
  libzmq
  ${OpenMP_LIBS}
)