#pragma once
#include "merger_point_normal_2f.h"
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_projector_types.h>

namespace srrg2_laser_slam_2d {

  class MergerProjective2D : public MergerPointNormal2f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = MergerPointNormal2f;
    using ThisType = MergerProjective2D;

    PARAM(srrg2_core::PropertyFloat,
          merge_threshold,
          "max distance for merging the points in the scene and the moving",
          0.2f,
          0);

    PARAM(srrg2_core::PropertyConfigurable_<srrg2_core::PointNormal2fProjectorPolar>,
          projector,
          "projector to compute correspondences",
          srrg2_core::PointNormal2fProjectorPolarPtr(new srrg2_core::PointNormal2fProjectorPolar),
          &_projector_changed_flag);

    virtual ~MergerProjective2D();

    void compute() override;

  protected:
    bool _projector_changed_flag = true;
    MovingMeasurementType _transformed_measurement;
    srrg2_core::PointNormal2fProjectorPolar::TargetMatrixType _scene_matrix, _measurement_matrix;
  };

  using MergerProjective2DPtr = std::shared_ptr<MergerProjective2D>;

} // namespace srrg2_laser_slam_2d
