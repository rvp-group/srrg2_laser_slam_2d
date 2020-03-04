#pragma once
#include "scene_clipper_2d.h"
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_projector_types.h>

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  class SceneClipperProjective2D : public SceneClipper2D {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType  = SceneClipperProjective2D;
    using BaseType  = SceneClipper2D;
    using SceneType = typename BaseType::SceneType;

    PARAM(PropertyConfigurable_<PointNormal2fProjectorPolar>,
          projector,
          "projector used to remap the points",
          PointNormal2fProjectorPolarPtr(new PointNormal2fProjectorPolar),
          nullptr);

    PARAM(PropertyFloat,
          voxelize_resolution,
          "resolution used to decimate the points in the scan on a grid [meters]",
          0.1,
          nullptr);

    virtual ~SceneClipperProjective2D();

    void compute() override;

  protected:
    PointNormal2fVectorCloud _local_scene_non_voxelized;
  };

  using SceneClipperProjective2DPtr = std::shared_ptr<SceneClipperProjective2D>;

} // namespace srrg2_laser_tracker_2d
