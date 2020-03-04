#pragma once
#include "srrg_config/configurable.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_pcl/point_types.h"
#include "srrg_slam_interfaces/merger.h"

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_slam_interfaces;
  using namespace srrg2_core;

  class Merger2D : public Merger_<Isometry2f, PointNormal2fVectorCloud, PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~Merger2D();
  };

  using Merger2DPtr = std::shared_ptr<Merger2D>;

} // namespace srrg2_laser_tracker_2d
