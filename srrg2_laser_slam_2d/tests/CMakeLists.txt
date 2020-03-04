#ds preprocessor definition used for tests
add_definitions(-DCURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

catkin_add_gtest(test_measurement_adaptor test_measurement_adaptor.cpp)
target_link_libraries(test_measurement_adaptor
  srrg2_laser_slam_2d_library
)

catkin_add_gtest(test_slam test_slam.cpp)
target_link_libraries(test_slam
  srrg2_laser_slam_2d_library
)
