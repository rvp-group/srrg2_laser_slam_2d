#ds preprocessor definition used for tests
add_definitions(-DCURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(benchmark_carmen benchmark_carmen.cpp)
target_link_libraries(benchmark_carmen
  srrg2_laser_slam_2d_library
  ${catkin_LIBRARIES}
)

add_executable(benchmark_simul benchmark_simul.cpp)
target_link_libraries(benchmark_simul
  srrg2_laser_slam_2d_library
  ${catkin_LIBRARIES}
)
