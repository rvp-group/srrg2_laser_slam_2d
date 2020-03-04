add_library(srrg2_laser_slam_2d_library
#  projector_spherical_2d.cpp

  correspondence_finder_2d.cpp
  correspondence_finder_projective_2d.cpp 
  correspondence_finder_nn_2d.cpp 
  correspondence_finder_kd_tree_2d.cpp 

  merger_2d.cpp 
  merger_projective_2d.cpp
  measurement_adaptor_projective_2d.cpp 
  scene_clipper_2d.cpp 
  scene_clipper_projective_2d.cpp 
  scene_clipper_ball_2d.cpp
  
  instances.cpp
)
  
target_link_libraries(
  srrg2_laser_slam_2d_library
  ${catkin_LIBRARIES}
)
