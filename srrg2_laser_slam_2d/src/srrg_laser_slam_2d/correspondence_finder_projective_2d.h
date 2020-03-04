#pragma once
#include "correspondence_finder_2d.h"
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_projector_types.h>

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;

  class CorrespondenceFinderProjective2D : public CorrespondenceFinder2D {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType = CorrespondenceFinder2D;
    using ThisType = CorrespondenceFinderProjective2D;

    PARAM(PropertyFloat, point_distance, "max distance between corresponding points", 0.5, 0);
    PARAM(PropertyFloat, normal_cos, "min cosinus between normals", 0.8, 0);
    PARAM(PropertyConfigurable_<PointNormal2fProjectorPolar>,
          projector,
          "projector to compute correspondences",
          PointNormal2fProjectorPolarPtr(new PointNormal2fProjectorPolar),
          &_projector_changed_flag);

    void compute() override;
    void printInfo() const;

  protected:
    bool _projector_changed_flag = true;
    PointNormal2fProjectorPolar::TargetMatrixType _fixed_matrix, _moving_matrix;
  };

  using CorrespondenceFinderProjective2DPtr = std::shared_ptr<CorrespondenceFinderProjective2D>;

} // namespace srrg2_laser_tracker_2d
