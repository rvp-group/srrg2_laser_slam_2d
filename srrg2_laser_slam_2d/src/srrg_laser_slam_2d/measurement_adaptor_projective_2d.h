#pragma once
// ia config
#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_pcl/point_projector_types.h>
#include <srrg_pcl/point_unprojector_types.h>

// ia processing module
#include <srrg_slam_interfaces/measurement_adaptor.h>

#include <srrg_pcl/normal_computator.h>

namespace srrg2_laser_tracker_2d {

  class MeasurementAdaptorProjective2D
    : public srrg2_slam_interfaces::MeasurementAdaptor_<srrg2_core::PointNormal2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = MeasurementAdaptorProjective2D;
    using BaseType =
      srrg2_slam_interfaces::MeasurementAdaptor_<srrg2_core::PointNormal2fVectorCloud>;
    using DestType             = typename BaseType::DestType;
    using NormalComputatorType = srrg2_core::NormalComputator1DSlidingWindow<DestType, 1>;

    PARAM(
      srrg2_core::PropertyConfigurable_<srrg2_core::PointNormal2fUnprojectorPolar>,
      unprojector,
      "un-projector used to compute the scan from the cloud",
      srrg2_core::PointNormal2fUnprojectorPolarPtr(new srrg2_core::PointNormal2fUnprojectorPolar()),
      nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<NormalComputatorType>,
          normal_computator_sliding,
          "normal computator object",
          std::shared_ptr<NormalComputatorType>(new NormalComputatorType()),
          nullptr);
    PARAM(srrg2_core::PropertyFloat, range_min, "range_min [meters]", 0.0, nullptr);
    PARAM(srrg2_core::PropertyFloat, range_max, "range_max [meters]", 1000.0, nullptr);
    PARAM(srrg2_core::PropertyFloat,
          voxelize_resolution,
          "unproject voxelization resolution",
          0.02,
          nullptr);
    PARAM(srrg2_core::PropertyString, scan_topic, "topic of the scan", "/scan", nullptr);

    void compute() override;
    bool setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_) override;

  protected:
    //! @brief shitty aux function to process messages
    void _processLaserMessage(srrg2_core::LaserMessagePtr message_);

    //! @brief laser scan fields - mandatory
    std::vector<float>* _ranges = nullptr;
  };

  using MeasurementAdaptorProjective2DPtr = std::shared_ptr<MeasurementAdaptorProjective2D>;

} // namespace srrg2_laser_tracker_2d
