#pragma once
#include "srrg2_laser_slam_2d/registration/correspondence_finder_projective_2d.h"
#include <srrg2_slam_interfaces/registration/aligners/aligner_slice_processor.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_plane2plane_error_factor.h>

namespace srrg2_laser_slam_2d {
  class AlignerSliceProcessorLaser2D
    : public srrg2_slam_interfaces::AlignerSliceProcessor_<srrg2_solver::SE2Plane2PlaneErrorFactor,
                                                           PointNormal2fVectorCloud,
                                                           PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AlignerSliceProcessorLaser2D() {
      param_finder.setValue(
        CorrespondenceFinderProjective2DPtr(new CorrespondenceFinderProjective2f));
    }
  };

  using AlignerSliceProcessorLaser2DPtr = std::shared_ptr<AlignerSliceProcessorLaser2D>;

  class AlignerSliceProcessorLaser2DWithSensor
    : public srrg2_slam_interfaces::AlignerSliceProcessor_<
        srrg2_solver::SE2Plane2PlaneWithSensorErrorFactor,
        PointNormal2fVectorCloud,
        PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = srrg2_slam_interfaces::AlignerSliceProcessor_<
      srrg2_solver::SE2Plane2PlaneWithSensorErrorFactor,
      PointNormal2fVectorCloud,
      PointNormal2fVectorCloud>;
    AlignerSliceProcessorLaser2DWithSensor() {
      param_finder.setValue(
        CorrespondenceFinderProjective2DPtr(new CorrespondenceFinderProjective2f));
    }

    virtual ~AlignerSliceProcessorLaser2DWithSensor() {
    }

  protected:
    virtual void setupFactor() override;
  };

  using AlignerSliceProcessorLaser2DWithSensorPtr =
    std::shared_ptr<AlignerSliceProcessorLaser2DWithSensor>;
} // namespace srrg2_laser_slam_2d
