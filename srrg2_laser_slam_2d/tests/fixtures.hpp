#include <srrg_benchmark/slam_benchmark_suite_carmen.hpp>
#include <srrg_pcl/instances.h>
#include <srrg2_slam_interfaces/instances.h>
#include <srrg_test/test_helper.hpp>

#include "srrg2_laser_slam_2d/instances.h"

class Synthetic : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void SetUp() override {
    // ds generate range and intensity measurements for a circle measurement
    range_measurements_circle.reserve(number_of_measurements);
    intensity_measurements_circle.reserve(number_of_measurements);
    for (size_t i = 0; i < number_of_measurements; ++i) {
      range_measurements_circle.emplace_back(1.0 /*1 meter*/);
      intensity_measurements_circle.emplace_back(0 /*black*/);
    }

    // ds populate laser message with synthetic data
    laser_message_circle = srrg2_core::LaserMessagePtr(new srrg2_core::LaserMessage(scan_topic));
    laser_message_circle->angle_min.setValue(angle_min_radians);
    laser_message_circle->angle_max.setValue(angle_max_radians);
    laser_message_circle->angle_increment.setValue(angle_increment);
    laser_message_circle->time_increment.setValue(time_increment);
    laser_message_circle->scan_time.setValue(scan_time);
    laser_message_circle->range_min.setValue(range_min);
    laser_message_circle->range_max.setValue(range_max);
    laser_message_circle->ranges.setValue(range_measurements_circle);
    laser_message_circle->intensities.setValue(intensity_measurements_circle);
  }
  void TearDown() override {
  }

  // ds synthetic sensor configuration
  const float angle_min_radians       = -1;
  const float angle_max_radians       = 1;
  const float angle_increment         = 0.02; // ds 100 measurements per sweep
  const float time_increment          = 0;
  const float scan_time               = 0;
  const float range_min               = 0;
  const float range_max               = 1000;
  const std::string scan_topic        = "/scan";
  const float voxelize_resolution     = 0.01;
  const size_t number_of_measurements = (angle_max_radians - angle_min_radians) / angle_increment;

  // ds testables
  std::vector<float> range_measurements_circle;
  std::vector<float> intensity_measurements_circle;
  srrg2_core::LaserMessagePtr laser_message_circle = nullptr;
};

class AIS : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void SetUp() override {
    srrg2_core::messages_registerTypes();
    srrg2_core::point_cloud_registerTypes();
    srrg2_laser_slam_2d::srrg2_laser_slam_2d_registerTypes();
    benchamin = srrg2_core::SLAMBenchmarkSuiteSE2Ptr(new srrg2_core::SLAMBenchmarkSuiteCARMEN());
  }

  void TearDown() override {
  }

  void loadDataset(const std::string& filepath_dataset_,
                   const std::string& filepath_ground_truth_,
                   const size_t& number_of_message_packs_to_read_ = -1,
                   const size_t& number_of_message_to_start_      = 0) {
    ASSERT_NOTNULL(benchamin);

    // ds load dataset from local path (provided by CI unittest environment)
    benchamin->loadDataset(
      filepath_dataset_, number_of_message_packs_to_read_, number_of_message_to_start_);
    benchamin->loadGroundTruth(filepath_ground_truth_);

    // ds grab first message to set laser message for configuration
    srrg2_core::MessagePackPtr message_pack =
      std::dynamic_pointer_cast<srrg2_core::MessagePack>(benchamin->getMessage());
    ASSERT_NOTNULL(message_pack);
    laser_message_0 =
      std::dynamic_pointer_cast<srrg2_core::LaserMessage>(message_pack->messages[0]);
    ASSERT_NOTNULL(laser_message_0);
    benchamin->reset();
  }

  // ds external configuration
  const float voxelize_resolution = 0.01;

  // ds testables
  srrg2_core::LaserMessagePtr laser_message_0    = nullptr;
  srrg2_core::SLAMBenchmarkSuiteSE2Ptr benchamin = nullptr;
};

// ds TODO implement
class Intel : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void SetUp() override {
  }
  void TearDown() override {
  }
};

// ds TODO implement
class M3500 : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void SetUp() override {
  }
  void TearDown() override {
  }
};
