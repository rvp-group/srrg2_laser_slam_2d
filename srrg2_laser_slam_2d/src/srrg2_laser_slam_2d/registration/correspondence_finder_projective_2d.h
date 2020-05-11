#pragma once
#include "correspondence_finder_normal_2f.h"
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_projector_types.h>

namespace srrg2_laser_slam_2d {
  using namespace srrg2_core;

  class CorrespondenceFinderProjective2f : public CorrespondenceFinderNormal2f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType = CorrespondenceFinderNormal2f;
    using ThisType = CorrespondenceFinderProjective2f;

    PARAM(srrg2_core::PropertyFloat,
          point_distance,
          "max distance between corresponding points",
          0.5,
          0);
    PARAM(srrg2_core::PropertyFloat, normal_cos, "min cosinus between normals", 0.8, 0);
    PARAM(srrg2_core::PropertyConfigurable_<srrg2_core::PointNormal2fProjectorPolar>,
          projector,
          "projector to compute correspondences",
          srrg2_core::PointNormal2fProjectorPolarPtr(new srrg2_core::PointNormal2fProjectorPolar),
          &_projector_changed_flag);

    void compute() override;
    void printInfo() const;

  protected:
    bool _projector_changed_flag = true;
    srrg2_core::PointNormal2fProjectorPolar::TargetMatrixType _fixed_matrix, _moving_matrix;
  };

  using CorrespondenceFinderProjective2DPtr = std::shared_ptr<CorrespondenceFinderProjective2f>;

} // namespace srrg2_laser_slam_2d
