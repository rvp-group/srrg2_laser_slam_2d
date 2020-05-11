#include "srrg2_laser_slam_2d/instances.h"
#include <srrg_benchmark_ros/slam_benchmark_suite_simul.hpp>
#include <srrg_pcl/instances.h>
#include <srrg2_slam_interfaces/instances.h>

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;

#ifdef CURRENT_SOURCE_DIR
const std::string current_source_directory = CURRENT_SOURCE_DIR;
#else
const std::string current_source_directory = "./";
#endif

const Vector2f maximum_mean_translation_rmse               = Vector2f(.1, .1);
const Vector2f maximum_standard_deviation_translation_rmse = Vector2f(.5, .5);
const Vector1f maximum_mean_rotation_rmse                  = Vector1f(1.0);
const Vector1f maximum_standard_deviation_rotation_rmse    = Vector1f(1.0);

int main(int argc, char** argv) {
  srrg2_core::srrgInit(argc, argv, "benchmark_simul");

  srrg2_core::messages_registerTypes();
  srrg2_core::point_cloud_registerTypes();
  srrg2_laser_slam_2d::srrg2_laser_slam_2d_registerTypes();
  srrg2_core_ros::messages_ros_registerTypes();
  Profiler::enable_logging = true;

  // ds load a laser slam assembly from configuration
  ConfigurableManager manager;
  manager.read(current_source_directory + "/../../configs/simul.conf");
  MultiGraphSLAM2DPtr slammer = manager.getByName<MultiGraphSLAM2D>("slam");
  assert(slammer);
  // ds instantiate the benchmark utility for AIS
  SLAMBenchmarkSuiteSE2Ptr benchamin(new SLAMBenchmarkSuiteSimul());

  benchamin->loadDataset("simul_laser_2d.bag", 2100);

  // ds process all messages and feed benchamin with computed estimates
  // ds TODO clearly this does not account for PGO/BA which happens retroactively
  while (BaseSensorMessagePtr message = benchamin->getMessage()) {
    SystemUsageCounter::tic();
    slammer->setRawData(message);
    slammer->compute();
    const double processing_duration_seconds = SystemUsageCounter::toc();
    benchamin->setPoseEstimate(
      slammer->robotInWorld(), message->timestamp.value(), processing_duration_seconds);
  }

  // ds run benchmark evaluation
  benchamin->compute();

  // ds save trajectory for external benchmark plot generation
  benchamin->writeTrajectoryToFile("trajectory_simul.txt");

  // ds evaluate if target metrics have not been met
  if (benchamin->isRegression(maximum_mean_translation_rmse,
                              maximum_standard_deviation_translation_rmse,
                              maximum_mean_rotation_rmse,
                              maximum_standard_deviation_rotation_rmse)) {
    return -1;
  } else {
    return 0;
  }
}
