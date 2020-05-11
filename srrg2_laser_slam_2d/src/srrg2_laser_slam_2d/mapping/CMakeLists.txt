add_library(srrg2_laser_slam_2d_mapping_library SHARED
  merger_projective_2d.cpp
  scene_clipper_projective_2d.cpp 
  scene_clipper_ball_2d.cpp
)

target_link_libraries(srrg2_laser_slam_2d_mapping_library
  srrg2_laser_slam_2d_sensor_processing_library
  ${catkin_LIBRARIES}
)
