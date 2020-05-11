#pragma once
#include <srrg2_slam_interfaces/trackers/tracker_slice_processor.h>

namespace srrg2_laser_slam_2d {
  class TrackerSliceProcessorLaser2D
    : public srrg2_slam_interfaces::TrackerSliceProcessor_<srrg2_core::Isometry2f,
                                                           srrg2_core::PointNormal2fVectorCloud,
                                                           srrg2_core::PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TrackerSliceProcessorLaser2D() {
      param_scene_slice_name.setValue("points");
      param_measurement_slice_name.setValue("points");
      param_adaptor.setValue(
        RawDataPreprocessorProjective2DPtr(new RawDataPreprocessorProjective2D));
      param_merger.setValue(MergerProjective2DPtr(new MergerProjective2D));
      param_clipper.setValue(SceneClipperProjective2DPtr(new SceneClipperProjective2D));
    }
  };

  using TrackerSliceProcessorLaser2DPtr = std::shared_ptr<TrackerSliceProcessorLaser2D>;

} // namespace srrg2_laser_slam_2d
