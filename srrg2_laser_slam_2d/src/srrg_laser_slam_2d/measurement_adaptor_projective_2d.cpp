#include <srrg_data_structures/matrix.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_messages/instances.h>
#include <srrg_pcl/point_projector_types.h>

// ia scan matcher things
#include "measurement_adaptor_projective_2d.h"

namespace srrg2_laser_tracker_2d {

  using namespace srrg2_core;

  void MeasurementAdaptorProjective2D::compute() {
    if (!_measurement || !_dest) {
      _status = Error;
      return;
    }

    if (!param_unprojector.value()) {
      throw std::runtime_error("MeasurementAdaptorProjective2D::compute| missing unprojector");
    }

    Matrix_<float> range_matrix;
    range_matrix.resize(1, _ranges->size());
    for (size_t i = 0; i < _ranges->size(); ++i) {
      range_matrix.at(0, i) = _ranges->at(i);
    }

    DestType dest_matrix;
    std::back_insert_iterator<DestType> fixed_back_insertor(dest_matrix);
    param_unprojector->compute<WithNormals>(fixed_back_insertor, range_matrix);
    param_normal_computator_sliding->computeNormals(dest_matrix);

    _dest->clear();
    _dest->reserve(_ranges->size());
    const float& res = param_voxelize_resolution.value();
    if (res > 0) {
      // ds the resolution is too high we have to voxelize
      DestType::PlainVectorType res_coeffs;
      res_coeffs << res, res, 1, 1;
      dest_matrix.voxelize(std::back_insert_iterator<DestType>(*_dest), res_coeffs);
    } else {
      // ds gobble up each valid point
      for (auto it = dest_matrix.begin(); it != dest_matrix.end(); ++it) {
        if (it->status == POINT_STATUS::Valid) {
          _dest->emplace_back(*it);
        }
      }
    }
    _status = Ready;
  }

  bool MeasurementAdaptorProjective2D::setMeasurement(BaseSensorMessagePtr measurement_) {
    if (!measurement_) {
      throw std::runtime_error(
        "MeasurementAdaptorProjective2D::setMeasurement|measurement is not set");
    }

    BaseType::setMeasurement(measurement_);
    _status = Error;

    LaserMessagePtr laser_message =
      srrg2_slam_interfaces::extractMessage<LaserMessage>(measurement_, param_scan_topic.value());
    if (!laser_message) {
      std::cerr << FG_RED("MeasurementAdaptorProjective2D::setMeasurement|measurement does not "
                          "contain a laser message")
                << std::endl;
      return false;
    }
    _processLaserMessage(laser_message);

    // ia we are ready to go
    _status = Ready;
    return true;
  }

  void MeasurementAdaptorProjective2D::_processLaserMessage(LaserMessagePtr laser_message) {
    _ranges = &laser_message->ranges.value();
    if (!_ranges) {
      throw std::runtime_error(
        "MeasurementAdaptorProjective2D::_processLaserMessage|ranges buffer not set");
    }
    const float range_max  = std::min(laser_message->range_max.value(), param_range_max.value());
    const float range_min  = std::max(laser_message->range_min.value(), param_range_min.value());
    const float angle_max  = laser_message->angle_max.value();
    const float angle_min  = laser_message->angle_min.value();
    const float sensor_res = (angle_max - angle_min) / _ranges->size();

    Matrix2f sensor_matrix(Matrix2f::Identity());
    sensor_matrix << 1.f / sensor_res, _ranges->size() / 2.f, 0, 0;

    // bdc instantiate the projectors (if not loaded from the config and
    // adjust their parameters based on the processed laser message
    if (!param_unprojector.value()) {
      throw std::runtime_error(
        "MeasurementAdaptorProjective2D::_processLaserMessage|missing unprojector");
    }
    PointNormal2fUnprojectorPolarPtr unprojector = param_unprojector.value();
    unprojector->param_range_min.setValue(range_min);
    unprojector->param_range_max.setValue(range_max);
    unprojector->param_angle_max.setValue(angle_max);
    unprojector->param_angle_min.setValue(angle_min);
    unprojector->setCameraMatrix(sensor_matrix);
  }

} // namespace srrg2_laser_tracker_2d
