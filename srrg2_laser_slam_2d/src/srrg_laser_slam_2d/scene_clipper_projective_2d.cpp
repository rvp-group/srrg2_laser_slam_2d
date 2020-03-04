#include "scene_clipper_projective_2d.h"
#include <srrg_system_utils/shell_colors.h>

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  SceneClipperProjective2D::~SceneClipperProjective2D() {
  }

  void SceneClipperProjective2D::compute() {
    if (!_local_scene || !_global_scene) {
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

    projector->setCameraPose(ThisType::_transform * ThisType::_sensor_in_robot);
    projector->compute(scene_in_sensor, _global_scene->begin(), _global_scene->end());

    float voxelize_res = param_voxelize_resolution.value();
    _local_scene->clear();
    if (voxelize_res > 0) {
      _local_scene_non_voxelized.clear();
      for (const auto& t : scene_in_sensor) {
        if (t.source_idx < 0) {
          continue;
        }
        _local_scene_non_voxelized.push_back(t.transformed);
      }
      SceneType::PlainVectorType res_coeffs;
      res_coeffs << voxelize_res, voxelize_res, 0.1, 0.1;
      _local_scene_non_voxelized.voxelize(
        std::back_insert_iterator<PointNormal2fVectorCloud>(*_local_scene), res_coeffs);
    } else {
      for (const auto& t : scene_in_sensor) {
        if (t.source_idx < 0) {
          continue;
        }
        _local_scene->push_back(t.transformed);
      }
    }

    // srrg move the local scene in robot's coords
    _local_scene->transformInPlace<Isometry>(ThisType::_sensor_in_robot);
    _status = Ready;
  }

} // namespace srrg2_laser_tracker_2d
