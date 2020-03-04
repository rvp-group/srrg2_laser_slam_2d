add_executable(slam_app slam_app.cpp)
target_link_libraries(slam_app 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_laser_slam_2d_library
                      pthread
                      ${QGLViewer_LIBRARIES}
                      ${SRRG_QT_LIBRARIES}
                      ${GLUT_LIBRARIES})
                      
                      
add_executable(synthetic_scene_generator synthetic_scene_generator.cpp)
target_link_libraries(synthetic_scene_generator 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_laser_slam_2d_library
                      srrg2_point_cloud_library
                      pthread
                      ${QGLViewer_LIBRARIES}
                      ${SRRG_QT_LIBRARIES}
                      ${GLUT_LIBRARIES})
            
add_executable(visual_test_measurement_adaptor_projective_2d visual_test_measurement_adaptor_projective_2d.cpp)
target_link_libraries(visual_test_measurement_adaptor_projective_2d 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_laser_slam_2d_library
                      srrg2_point_cloud_library
                      pthread
                      ${QGLViewer_LIBRARIES}
                      ${SRRG_QT_LIBRARIES}
                      ${GLUT_LIBRARIES})
                      
add_executable(visual_test_scene_clipper_projective_2d visual_test_scene_clipper_projective_2d.cpp)
target_link_libraries(visual_test_scene_clipper_projective_2d 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_laser_slam_2d_library
                      srrg2_point_cloud_library
                      pthread
                      ${QGLViewer_LIBRARIES}
                      ${SRRG_QT_LIBRARIES}
                      ${GLUT_LIBRARIES}) 
                                       
add_executable(visual_test_correspondence_finder_projective_2d visual_test_correspondence_finder_projective_2d.cpp)
target_link_libraries(visual_test_correspondence_finder_projective_2d 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_laser_slam_2d_library
                      srrg2_point_cloud_library
                      pthread
                      ${QGLViewer_LIBRARIES}
                      ${SRRG_QT_LIBRARIES}
                      ${GLUT_LIBRARIES})
                      
add_executable(visual_test_merger_projective_2d visual_test_merger_projective_2d.cpp)
target_link_libraries(visual_test_merger_projective_2d 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_laser_slam_2d_library
                      srrg2_point_cloud_library
                      pthread
                      ${QGLViewer_LIBRARIES}
                      ${SRRG_QT_LIBRARIES}
                      ${GLUT_LIBRARIES})
                      
add_executable(visual_test_aligner_2d visual_test_aligner_2d.cpp)
target_link_libraries(visual_test_aligner_2d 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_laser_slam_2d_library
                      srrg2_point_cloud_library
                      pthread
                      ${QGLViewer_LIBRARIES}
                      ${SRRG_QT_LIBRARIES}
                      ${GLUT_LIBRARIES})

add_executable(visual_test_tracker_2d visual_test_tracker_2d.cpp)
target_link_libraries(visual_test_tracker_2d 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_laser_slam_2d_library
                      srrg2_point_cloud_library
                      pthread
                      ${QGLViewer_LIBRARIES}
                      ${SRRG_QT_LIBRARIES}
                      ${GLUT_LIBRARIES})


add_executable(generic_bench_app generic_bench_app.cpp)
target_link_libraries(generic_bench_app 
                      srrg2_slam_interfaces_library
                      srrg2_system_utils_library
                      srrg2_config_library
                      srrg2_solver_core_library
                      srrg2_solver_types2d_library 
                      srrg2_point_cloud_library
                      srrg2_laser_slam_2d_library)