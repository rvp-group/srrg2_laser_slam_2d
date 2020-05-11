#include "merger_projective_2d.h"

namespace srrg2_laser_slam_2d {

  using namespace srrg2_core;
  MergerProjective2D::~MergerProjective2D() {
  }

  void MergerProjective2D::compute() {
    if (!param_projector.value()) {
      throw std::runtime_error("MergerProjective2D::compute| Missing Projector");
    }
    PointNormal2fProjectorPolarPtr projector = param_projector.value();
    const int num_beams                      = projector->param_canvas_cols.value();

    _scene_matrix.resize(1, num_beams);
    _measurement_matrix.resize(1, num_beams);

    projector->setCameraPose(_measurement_in_scene);
    projector->compute(_scene_matrix, _scene->begin(), _scene->end());

    _transformed_measurement = *_measurement;
    _transformed_measurement.transformInPlace(_measurement_in_scene);
    projector->compute(
      _measurement_matrix, _transformed_measurement.begin(), _transformed_measurement.end());

    // bdc get current scene size
    size_t scene_size = _scene->size();
    // bdc increase the size with the number of measures
    // std::cerr << "nsc: " << _scene->size() << ", ";
    _scene->resize(scene_size + _measurement_matrix.size());
    auto scene_it  = _scene_matrix.begin();
    auto moving_it = _measurement_matrix.begin();

    int new_points      = 0;
    int merged_points   = 0;
    int replaced_points = 0;

    for (; scene_it != _scene_matrix.end() && moving_it != _measurement_matrix.end();
         ++scene_it, ++moving_it) {
      auto& scell = *scene_it;
      auto& mcell = *moving_it;

      // srrg check if point is close to range limit (sometimes this introduces fake measurements if
      // not handled)
      if (mcell.depth > .9f * projector->param_range_max.value()) {
        mcell.source_idx = -1;
      }

      // bdc measure is useless, continue
      if (mcell.source_idx < 0) {
        continue;
      }

      auto& moving_point = _transformed_measurement[mcell.source_idx];
      // bdc source is useless, add measure to the scene as it is
      if (scell.source_idx < 0 && mcell.source_idx >= 0) {
        _scene->at(scene_size) = moving_point;
        ++scene_size;
        ++new_points;
        continue;
      }

      auto& scene_point = _scene->at(scell.source_idx);
      // bdc both measure and source are useful, check how distant they are
      const float dr           = (mcell.depth - scell.depth);
      const float abs_dr       = fabs(dr);
      const bool moving_behind = (dr > 0);

      // bdc they are close, merge
      if (abs_dr < param_merge_threshold.value()) {
        scene_point += moving_point;
        scene_point *= 0.5f;
        scene_point.normalize();
        ++merged_points;
        continue;
      }

      // bdc they are not close, if measure is behind, replace source w/ measure
      if (moving_behind) {
        ++replaced_points;
        scene_point = moving_point;
        continue;
      }

      // bdc they are not close, source is behind, add measure as new element of the full scene
      _scene->at(scene_size) = moving_point;
      ++scene_size;
    }
    // std::cerr << "n: " <<  new_points
    //           << ", m: " <<  merged_points
    //           << ", r: " <<  replaced_points
    //           << ", os: " <<  _scene->size()
    //           << ", ns: " << scene_size <<std::endl;

    // bdc resize to the new size
    _scene->resize(scene_size);

    _status = MergerBase::Status::Success;
  }

} // namespace srrg2_laser_slam_2d
