add_executable(to_trenz 
  ${FASTSENSE_DIR}/src/registration/imu_accumulator.cpp
  bridge.cpp
)

target_compile_options(to_trenz PRIVATE ${OpenMP_FLAGS} -Wall -Wpedantic)

target_include_directories(to_trenz PRIVATE
  ${FASTSENSE_DIR}/src
  ${FASTSENSE_ROS_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(to_trenz
  ${catkin_LIBRARIES}
  libzmq
  ${OpenMP_LIBS}
)