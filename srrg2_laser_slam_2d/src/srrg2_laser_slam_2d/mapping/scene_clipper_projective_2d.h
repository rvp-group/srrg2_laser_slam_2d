#pragma once
#include "scene_clipper_point_normal_2f.h"
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_projector_types.h>

namespace srrg2_laser_slam_2d {

  class SceneClipperProjective2D : public SceneClipperPointNormal2f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType  = SceneClipperProjective2D;
    using BaseType  = SceneClipperPointNormal2f;
    using SceneType = typename BaseType::SceneType;

    PARAM(srrg2_core::PropertyConfigurable_<srrg2_core::PointNormal2fProjectorPolar>,
          projector,
          "projector used to remap the points",
          srrg2_core::PointNormal2fProjectorPolarPtr(new srrg2_core::PointNormal2fProjectorPolar),
          nullptr);

    PARAM(srrg2_core::PropertyFloat,
          voxelize_resolution,
          "resolution used to decimate the points in the scan on a grid [meters]",
          0.1,
          nullptr);

    virtual ~SceneClipperProjective2D();

    void compute() override;

  protected:
    srrg2_core::PointNormal2fVectorCloud _local_scene_non_voxelized;
  };

  using SceneClipperProjective2DPtr = std::shared_ptr<SceneClipperProjective2D>;

} // namespace srrg2_laser_slam_2d
