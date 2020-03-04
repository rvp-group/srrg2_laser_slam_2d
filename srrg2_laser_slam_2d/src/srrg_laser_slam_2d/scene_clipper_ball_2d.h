#pragma once
#include "scene_clipper_2d.h"

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  class SceneClipperBall2D : public SceneClipper2D {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyFloat, max_range, "maximum distance of points from origin", 10, nullptr);

    virtual ~SceneClipperBall2D();
    void compute() override;
  };

} // namespace srrg2_laser_tracker_2d
