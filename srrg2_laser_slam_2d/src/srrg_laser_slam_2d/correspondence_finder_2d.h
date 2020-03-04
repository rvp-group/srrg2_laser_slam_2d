#pragma once
#include "srrg_config/configurable.h"
#include "srrg_data_structures/correspondence.h"
#include "srrg_pcl/point_types.h"
#include "srrg_slam_interfaces/correspondence_finder.h"

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  struct CorrespondenceFinder2D
    : public CorrespondenceFinder_<Isometry2f, PointNormal2fVectorCloud, PointNormal2fVectorCloud> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setEstimate(const TransformType& est_) override {
      _estimate = est_;
    }

  protected:
    Isometry2f _estimate = Isometry2f::Identity();
  };
  using CorrespondenceFinder2DPtr = std::shared_ptr<CorrespondenceFinder2D>;
} // namespace srrg2_laser_tracker_2d
