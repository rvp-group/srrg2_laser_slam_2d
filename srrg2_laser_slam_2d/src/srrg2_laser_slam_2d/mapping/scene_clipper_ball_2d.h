#pragma once
#include "scene_clipper_point_normal_2f.h"

namespace srrg2_laser_slam_2d {

  class SceneClipperBall2D : public SceneClipperPointNormal2f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(srrg2_core::PropertyFloat,
          max_range,
          "maximum distance of points from origin",
          10,
          nullptr);

    virtual ~SceneClipperBall2D() = default;
    void compute() override;
  };

} // namespace srrg2_laser_slam_2d
