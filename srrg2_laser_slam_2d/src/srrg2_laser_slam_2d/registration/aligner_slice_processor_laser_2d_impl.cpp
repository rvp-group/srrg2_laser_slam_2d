#include "aligner_slice_processor_laser_2d.h"

#include <srrg2_slam_interfaces/registration/aligners/aligner_slice_processor_impl.cpp>
#include <srrg2_slam_interfaces/registration/aligners/aligner_termination_criteria_impl.cpp>

namespace srrg2_laser_slam_2d {
  void AlignerSliceProcessorLaser2DWithSensor::setupFactor() {
    BaseType::setupFactor();
    BaseType::setupFactorWithSensor("MultiAlignerLaser2DWithSensorSlice");
  }
} // namespace srrg2_laser_slam_2d
