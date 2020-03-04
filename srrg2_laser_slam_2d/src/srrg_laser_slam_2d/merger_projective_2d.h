#pragma once
#include "merger_2d.h"
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_projector_types.h>

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  class MergerProjective2D : public Merger2D {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = Merger2D;
    using ThisType = MergerProjective2D;

    PARAM(PropertyFloat,
          merge_threshold,
          "max distance for merging the points in the scene and the moving",
          0.2f,
          0);

    PARAM(PropertyConfigurable_<PointNormal2fProjectorPolar>,
          projector,
          "projector to compute correspondences",
          PointNormal2fProjectorPolarPtr(new PointNormal2fProjectorPolar),
          &_projector_changed_flag);

    virtual ~MergerProjective2D();

    void compute() override;

  protected:
    bool _projector_changed_flag = true;
    MovingType _transformed_moving;
    PointNormal2fProjectorPolar::TargetMatrixType _scene_matrix, _moving_matrix;
  };

  using MergerProjective2DPtr = std::shared_ptr<MergerProjective2D>;

} // namespace srrg2_laser_tracker_2d
