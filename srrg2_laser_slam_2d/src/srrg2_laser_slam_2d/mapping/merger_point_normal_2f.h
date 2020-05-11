#pragma once
#include <srrg_config/configurable.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_pcl/point_types.h>
#include <srrg2_slam_interfaces/mapping/merger.h>

namespace srrg2_laser_slam_2d {

  using MergerPointNormal2f = srrg2_slam_interfaces::Merger_<srrg2_core::Isometry2f,
                                                             srrg2_core::PointNormal2fVectorCloud,
                                                             srrg2_core::PointNormal2fVectorCloud>;

  using MergerPointNormal2fPtr = std::shared_ptr<MergerPointNormal2f>;

} // namespace srrg2_laser_slam_2d
