#include "correspondence_finder_nn_2d.h"

namespace srrg2_laser_slam_2d {

  using namespace srrg2_core;
  CorrespondenceFinderNN2D::CorrespondenceFinderNN2D() {
    _search.setPathMatrix(&_path_map);
  }

  void CorrespondenceFinderNN2D::_recomputeInternals() {
    if (param_resolution.value() <= 0) {
      std::cerr << __PRETTY_FUNCTION__ << std::endl;
      throw std::runtime_error("resolution must be > 0");
    }
    if (param_max_distance_m.value() < 0) {
      std::cerr << __PRETTY_FUNCTION__ << std::endl;
      throw std::runtime_error("please set max_distance_m > 0");
    }

    _inv_resolution    = 1.f / param_resolution.value();
    const float& mdm   = param_max_distance_m.value();
    float mds_in_pixel = mdm * mdm * _inv_resolution * _inv_resolution;
    int padding        = static_cast<int>(std::sqrt(mds_in_pixel) + 75.5);
    _padding_vector << padding, padding;
    _search.param_max_distance_squared_pxl.setValue(mds_in_pixel);
  }

  void CorrespondenceFinderNN2D::_adjustSize() {
    FixedScalarType lx = std::numeric_limits<FixedScalarType>::max();
    FixedScalarType ly = std::numeric_limits<FixedScalarType>::max();
    FixedScalarType ux = std::numeric_limits<FixedScalarType>::min();
    FixedScalarType uy = std::numeric_limits<FixedScalarType>::min();

    for (const FixedPointType& p : (*_fixed)) {
      const FixedScalarType& px = p.coordinates().x();
      const FixedScalarType& py = p.coordinates().y();

      lx = std::min(lx, px);
      ly = std::min(ly, py);
      ux = std::max(ux, px);
      uy = std::max(uy, py);
    }
    _bounding_box << (ux - lx), (uy - ly);
    _lowest_point << lx, ly;

    Vector2f bb     = _bounding_box * _invResolution() + _paddingVector();
    size_t new_rows = std::max(static_cast<size_t>(std::ceil(bb.x())), _path_map.rows());
    size_t new_cols = std::max(static_cast<size_t>(std::ceil(bb.y())), _path_map.cols());
    if (new_rows > _path_map.rows() || new_cols > _path_map.cols()) {
      _path_map.resize(new_rows, new_cols);
    }
  }

  void CorrespondenceFinderNN2D::compute() {
    if (ThisType::_fixed_changed_flag || ThisType::_config_changed) {
      reset();
    }

    _correspondences->resize(_moving->size());
    const MatrixInt& parent_map = _search.parentMap();

    size_t k = 0;
    for (std::size_t m = 0; m < _moving->size(); ++m) {
      int moving_idx = static_cast<int>(m);
      PointNormal2f moved =
        _moving->at(moving_idx).transform<TRANSFORM_CLASS::Isometry>(_local_map_in_sensor);
      Vector2_<size_t> coord = _pointInMapCoords(moved.coordinates()).cast<size_t>();
      if (!parent_map.inside(coord.x(), coord.y())) {
        continue;
      }
      const int fixed_idx = parent_map.at(coord.x(), coord.y());
      if (fixed_idx == -1) {
        continue;
      }
      if (moved.normal().dot(_fixed->at(fixed_idx).normal()) < param_normal_cos.value()) {
        continue;
      }

      _correspondences->at(k++) = Correspondence(fixed_idx, moving_idx);
    }
    _correspondences->resize(k);
  }

  void CorrespondenceFinderNN2D::reset() {
    _recomputeInternals();
    _adjustSize();

    _points_in_distance_map.resize(_fixed->size());
    for (std::size_t f = 0; f < _fixed->size(); ++f) {
      _points_in_distance_map[f].coordinates() = _pointInMapCoords((*_fixed)[f].coordinates());
    }
    _search.reset();
    _search.setGoals(_points_in_distance_map);
    _search.compute();
    _fixed_changed_flag = false;
    _config_changed     = false;
  }

} // namespace srrg2_laser_slam_2d
