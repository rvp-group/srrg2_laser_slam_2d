add_library(srrg2_laser_slam_2d_registration_library SHARED
  correspondence_finder_projective_2d.cpp
  correspondence_finder_nn_2d.cpp
  correspondence_finder_kd_tree_2d.cpp
)

target_link_libraries(srrg2_laser_slam_2d_registration_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
