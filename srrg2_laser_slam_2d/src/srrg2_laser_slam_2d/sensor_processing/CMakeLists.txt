add_library(srrg2_laser_slam_2d_sensor_processing_library SHARED
  raw_data_preprocessor_projective_2d.cpp
)

target_link_libraries(srrg2_laser_slam_2d_sensor_processing_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
