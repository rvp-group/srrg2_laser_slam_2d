#pragma once
#include "srrg2_slam_interfaces/registration/correspondence_finder.h"
#include "srrg_config/configurable.h"
#include "srrg_data_structures/correspondence.h"
#include "srrg_pcl/point_types.h"

namespace srrg2_laser_slam_2d {

  using CorrespondenceFinderNormal2f =
    srrg2_slam_interfaces::CorrespondenceFinder_<srrg2_core::Isometry2f,
                                                 srrg2_core::PointNormal2fVectorCloud,
                                                 srrg2_core::PointNormal2fVectorCloud>;
  using CorrespondenceFinderNormal2fPtr = std::shared_ptr<CorrespondenceFinderNormal2f>;
} // namespace srrg2_laser_slam_2d
