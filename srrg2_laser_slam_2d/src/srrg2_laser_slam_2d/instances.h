#pragma once
#include "mapping/merger_projective_2d.h"
#include "mapping/scene_clipper_ball_2d.h"
#include "mapping/scene_clipper_projective_2d.h"
#include "registration/aligner_slice_processor_laser_2d.h"
#include "registration/correspondence_finder_kd_tree_2d.h"
#include "registration/correspondence_finder_nn_2d.h"
#include "registration/correspondence_finder_projective_2d.h"
#include "sensor_processing/raw_data_preprocessor_projective_2d.h"
#include "tracking/tracker_slice_processor_laser_2d.h"

namespace srrg2_laser_slam_2d {

  void srrg2_laser_slam_2d_registerTypes() __attribute__((constructor));

} // namespace srrg2_laser_slam_2d
