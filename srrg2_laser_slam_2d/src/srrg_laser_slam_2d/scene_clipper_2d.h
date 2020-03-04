#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_types.h>
#include <srrg_slam_interfaces/scene_clipper.h>

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  class SceneClipper2D : public SceneClipper_<Isometry2f, PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType  = SceneClipper2D;
    using BaseType  = SceneClipper_<Isometry2f, PointNormal2fVectorCloud>;
    using SceneType = typename BaseType::SceneType;
    virtual ~SceneClipper2D();
  };
  using SceneClipper2DPtr = std::shared_ptr<SceneClipper2D>;
} // namespace srrg2_laser_tracker_2d
