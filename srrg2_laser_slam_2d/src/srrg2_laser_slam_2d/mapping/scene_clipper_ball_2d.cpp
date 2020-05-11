#include "scene_clipper_ball_2d.h"

#include <srrg_geometry/geometry2d.h>

namespace srrg2_laser_slam_2d {
  using namespace srrg2_core;

  void SceneClipperBall2D::compute() {
    if (!_clipped_scene_in_robot || !_full_scene) {
      _status = Error;
      return;
    }

    EstimateType sensor_in_local_map = _robot_in_local_map * _sensor_in_robot;

    PointNormal2f center;
    center.coordinates() = sensor_in_local_map.translation();
    center.normal().setZero();
    // PointNormal2f scales;
    // scales.coordinates().fill(1./max_range.value());
    // scales.normal().setZero();
    Vector4f scales_vec;
    scales_vec << param_max_range.value(), param_max_range.value(), 0.f, 0.f;

    // PointInBallPredicate_<PointNormal2f> in_ball(center,scales);
    // PointCloudOps::applyPredicate(_indices, *_local_scene,  *_global_scene, in_ball);

    _full_scene->clip(std::back_insert_iterator<PointNormal2fVectorCloud>(*_clipped_scene_in_robot),
                      center,
                      scales_vec);

    // gg: obtain a local view with origin at the current pose
    assert(false && "SceneClipperBall2D::compute| must be tested");
    // if full_scene clip gives points in scene coords use following
    _clipped_scene_in_robot->transformInPlace(_local_map_in_robot);

    // if full_scene clip gives points in clipping coords use following
    //    _clipped_scene_in_robot->transformInPlace(_sensor_in_robot);

    _status = Successful;
  }

} // namespace srrg2_laser_slam_2d
