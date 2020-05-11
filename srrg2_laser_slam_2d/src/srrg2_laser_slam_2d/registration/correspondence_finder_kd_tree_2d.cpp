#include "correspondence_finder_kd_tree_2d.h"

namespace srrg2_laser_slam_2d {

  void CorrespondenceFinderKDTree2D::compute() {
    if (this->_fixed_changed_flag) {
      reset();
    }

    _correspondences->resize(_moving->size());
    std::size_t k = 0;
    for (std::size_t moving_idx = 0; moving_idx < _moving->size(); ++moving_idx) {
      int fixed_idx = -1;
      KDTree2D::VectorTD response_coord;
      MovingPointType moved =
        _moving->at(moving_idx).transform<TRANSFORM_CLASS::Isometry>(_local_map_in_sensor);

      float distance = _kd_tree->findNeighbor(
        response_coord, fixed_idx, moved.coordinates(), param_max_distance_m.value());

      if (distance != -1) {
        if (moved.normal().dot(_fixed->at(fixed_idx).normal()) < param_normal_cos.value()) {
          continue;
        }
        _correspondences->at(k++) = Correspondence(fixed_idx, moving_idx);
      }
    }
    _correspondences->resize(k);
  }

  void CorrespondenceFinderKDTree2D::reset() {
    KDTree2D::VectorTDVector coords_vector(_fixed->size());
    for (size_t i = 0; i < coords_vector.size(); ++i) {
      coords_vector[i] = (*_fixed)[i].coordinates();
    }
    _kd_tree = std::move(std::unique_ptr<KDTree2D>(
      new KDTree2D(coords_vector, param_max_leaf_range.value(), param_min_leaf_points.value())));
  }

} // namespace srrg2_laser_slam_2d
