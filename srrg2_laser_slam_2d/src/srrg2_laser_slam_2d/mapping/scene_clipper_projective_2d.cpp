#include "scene_clipper_projective_2d.h"
#include <srrg_system_utils/shell_colors.h>

namespace srrg2_laser_slam_2d {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  SceneClipperProjective2D::~SceneClipperProjective2D() {
  }

  void SceneClipperProjective2D::compute() {
    if (!_clipped_scene_in_robot || !_full_scene) {
      _status = Error;
      std::cerr << FG_RED("SceneClipperProjective2D::compute| missing local OR global scene")
                << std::endl;
      return;
    }

    if (!param_projector.value()) {
      throw std::runtime_error("SceneClipperProjective2D::compute| Missing Projector");
    }
    PointNormal2fProjectorPolarPtr projector = param_projector.value();

    PointNormal2fProjectorPolar::TargetMatrixType scene_in_sensor;

    size_t max_size = projector->param_canvas_cols.value();
    scene_in_sensor.resize(1, max_size);

    EstimateType sensor_in_local_map = _robot_in_local_map * _sensor_in_robot;

    projector->setCameraPose(sensor_in_local_map);
    projector->compute(scene_in_sensor, _full_scene->begin(), _full_scene->end());

    float voxelize_res = param_voxelize_resolution.value();
    _clipped_scene_in_robot->clear();
    if (voxelize_res > 0) {
      _local_scene_non_voxelized.clear();
      for (const auto& t : scene_in_sensor) {
        if (t.source_idx < 0) {
          continue;
        }
        const auto& point_in_sensor = t.transformed;
        _local_scene_non_voxelized.push_back(point_in_sensor);
      }
      SceneType::PlainVectorType res_coeffs;
      res_coeffs << voxelize_res, voxelize_res, 0.1, 0.1;
      _local_scene_non_voxelized.voxelize(
        std::back_insert_iterator<PointNormal2fVectorCloud>(*_clipped_scene_in_robot), res_coeffs);
    } else {
      for (const auto& t : scene_in_sensor) {
        if (t.source_idx < 0) {
          continue;
        }
        const auto& point_in_sensor = t.transformed;
        _clipped_scene_in_robot->push_back(point_in_sensor);
      }
    }

    // srrg move the local scene in robot's coords
    if (ThisType::_sensor_in_robot.matrix() != EstimateType::Identity().matrix()) {
      _clipped_scene_in_robot->transformInPlace<Isometry>(ThisType::_sensor_in_robot);
    }

    _status = Successful;
  }

} // namespace srrg2_laser_slam_2d
