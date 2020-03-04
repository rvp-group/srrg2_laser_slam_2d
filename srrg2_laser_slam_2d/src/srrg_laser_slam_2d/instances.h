#pragma once
#include "correspondence_finder_kd_tree_2d.h"
#include "correspondence_finder_nn_2d.h"
#include "correspondence_finder_projective_2d.h"
#include "measurement_adaptor_projective_2d.h"
#include "merger_projective_2d.h"
#include "scene_clipper_ball_2d.h"
#include "scene_clipper_projective_2d.h"
#include "srrg_slam_interfaces/multi_aligner_slice.h"
#include "srrg_slam_interfaces/multi_tracker_slice.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_plane2plane_error_factor.h"

namespace srrg2_laser_tracker_2d {

  class MultiAlignerLaser2DSlice
    : public srrg2_slam_interfaces::MultiAlignerSlice_<SE2Plane2PlaneErrorFactor,
                                                       PointNormal2fVectorCloud,
                                                       PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MultiAlignerLaser2DSlice() {
      param_finder.setValue(
        CorrespondenceFinderProjective2DPtr(new CorrespondenceFinderProjective2D));
    }
  };

  using MultiAlignerLaser2DSlicePtr = std::shared_ptr<MultiAlignerLaser2DSlice>;

  class MultiAlignerLaser2DWithSensorSlice
    : public srrg2_slam_interfaces::MultiAlignerSlice_<SE2Plane2PlaneWithSensorErrorFactorAD,
                                                       PointNormal2fVectorCloud,
                                                       PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      srrg2_slam_interfaces::MultiAlignerSlice_<SE2Plane2PlaneWithSensorErrorFactorAD,
                                                PointNormal2fVectorCloud,
                                                PointNormal2fVectorCloud>;
    MultiAlignerLaser2DWithSensorSlice() {
      param_finder.setValue(
        CorrespondenceFinderProjective2DPtr(new CorrespondenceFinderProjective2D));
    }

  protected:
    virtual void setupFactor() override;
  };

  using MultiAlignerLaser2DWithSensorSlicePtr = std::shared_ptr<MultiAlignerLaser2DWithSensorSlice>;

  class MultiTrackerLaser2DSlice
    : public srrg2_slam_interfaces::
        MultiTrackerSlice_<Isometry2f, PointNormal2fVectorCloud, PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MultiTrackerLaser2DSlice() {
      param_scene_slice_name.setValue("points");
      param_measurement_slice_name.setValue("points");
      param_adaptor.setValue(MeasurementAdaptorProjective2DPtr(new MeasurementAdaptorProjective2D));
      param_merger.setValue(MergerProjective2DPtr(new MergerProjective2D));
      param_clipper.setValue(SceneClipperProjective2DPtr(new SceneClipperProjective2D));
    }
  };

  using MultiTrackerLaser2DSlicePtr = std::shared_ptr<MultiTrackerLaser2DSlice>;

  void laser_tracker_2d_registerTypes() __attribute__((constructor));

} // namespace srrg2_laser_tracker_2d
