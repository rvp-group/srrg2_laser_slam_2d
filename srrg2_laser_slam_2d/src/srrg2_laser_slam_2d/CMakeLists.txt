add_subdirectory(sensor_processing)
add_subdirectory(mapping)
add_subdirectory(registration)

add_library(srrg2_laser_slam_2d_library
  instances.cpp
)
  
target_link_libraries(srrg2_laser_slam_2d_library
  srrg2_laser_slam_2d_sensor_processing_library
  srrg2_laser_slam_2d_mapping_library
  srrg2_laser_slam_2d_registration_library
  ${catkin_LIBRARIES}
)
