#include "scene_clipper_ball_2d.h"

#include <srrg_geometry/geometry2d.h>

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;

  SceneClipperBall2D::~SceneClipperBall2D() {
  }

  void SceneClipperBall2D::compute() {
    if (!_local_scene || !_global_scene) {
      _status = Error;
      return;
    }

    PointNormal2f center;
    center.coordinates() = _transform.translation();
    center.normal().setZero();
    // PointNormal2f scales;
    // scales.coordinates().fill(1./max_range.value());
    // scales.normal().setZero();
    Vector4f scales_vec;
    scales_vec << param_max_range.value(), param_max_range.value(), 0.f, 0.f;

    // PointInBallPredicate_<PointNormal2f> in_ball(center,scales);
    // PointCloudOps::applyPredicate(_indices, *_local_scene,  *_global_scene, in_ball);

    _global_scene->clip(
      std::back_insert_iterator<PointNormal2fVectorCloud>(*_local_scene), center, scales_vec);

    // gg: obtain a local view with origin at the current pose
    _local_scene->transformInPlace(_transform.inverse());
  }

} // namespace srrg2_laser_tracker_2d
