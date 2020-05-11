cmake_minimum_required(VERSION 2.8.3)
project(srrg2_laser_slam_2d)

find_package(catkin REQUIRED COMPONENTS
  tf
  sensor_msgs
  srrg_cmake_modules
  srrg2_core
  srrg2_core_ros
  srrg2_solver
  srrg2_qgl_viewport
  srrg2_slam_interfaces
)

#ds fight sloppyness: extend cmake compile flags to be more restrictive
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)

find_package(Eigen3 REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_laser_slam_2d_library
    srrg2_laser_slam_2d_registration_library
    srrg2_laser_slam_2d_sensor_processing_library
    srrg2_laser_slam_2d_mapping_library
  CATKIN_DEPENDS
    srrg2_slam_interfaces
    srrg2_core
    srrg2_core_ros
    srrg2_solver
    srrg2_qgl_viewport
    sensor_msgs
    tf
    srrg_cmake_modules
  DEPENDS
    QGLViewer
    GLUT
)

#ds includes provided by this package (restrictive compile flags will be applied!)
include_directories(src)

#ds specify additional locations of header files
#ds treating them as system includes to suppress a massive wave of warnings originating from those header files (!)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

# set sources
add_subdirectory(src)

add_subdirectory(apps)
#ds gtest directory
add_subdirectory(tests)

#ds regression testing
add_subdirectory(benchmarks)
