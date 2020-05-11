#include <srrg_benchmark_ros/slam_benchmark_suite_simul.hpp>
#include <srrg_pcl/instances.h>
#include <srrg2_slam_interfaces/instances.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include "srrg2_laser_slam_2d/instances.h"

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;

std::string exe_name("generic_bench_app");
#define LOG std::cerr << exe_name + "|"

using TumContainer =
  std::map<double, Vector7f, std::less<double>, Eigen::aligned_allocator<Vector7f>>;

void retrievePlatformSource(MultiGraphSLAM2DPtr sink, MessageSourceBasePtr source) {
  MessagePlatformSinkPtr platform_sink = std::dynamic_pointer_cast<MessagePlatformSink>(sink);
  if (platform_sink == nullptr) {
    return;
  }
  MessageSourcePlatformPtr platform_source;
  std::shared_ptr<MessageFilterBase> s = std::dynamic_pointer_cast<MessageFilterBase>(source);
  while (s) {
    platform_source = std::dynamic_pointer_cast<MessageSourcePlatform>(s);
    if (platform_source) {
      PlatformPtr platform = platform_source->platform(platform_sink->param_tf_topic.value());
      if (platform && platform->isWellFormed()) {
        platform_sink->setPlatform(platform);
        std::cerr << "CommandRunRunner::retrievePlatformSource|"
                  << FG_GREEN("platform assigned:\n"
                              << platform_sink->platform())
                  << std::endl;
      }
      return;
    }
    std::shared_ptr<MessageFilterBase> next =
      std::dynamic_pointer_cast<MessageFilterBase>(s->param_source.value());
    s = next;
  }
}

int main(int argc, char** argv) {
  srrg2_core::srrgInit(argc, argv, "generic_laser2d_bench");
  srrg2_core::messages_registerTypes();
  srrg2_core::point_cloud_registerTypes();
  srrg2_laser_slam_2d::srrg2_laser_slam_2d_registerTypes();
  srrg2_core_ros::messages_ros_registerTypes();
  Profiler::enable_logging = true;

  ParseCommandLine cmd(argv);
  ArgumentString arg_config(&cmd, "c", "confg", "configuration to load", "");
  ArgumentString arg_output(&cmd, "o", "output", "output in TUM format", "");
  ArgumentString arg_output_timing(&cmd, "t", "timing", "output timings", "");
  ArgumentString arg_slammer_name(&cmd, "ns", "name-slam", "slammer name in the config", "slam");
  ArgumentString arg_sync_name(&cmd, "nss", "name-sync", "sync source name in the config", "sync");
  cmd.parse();

  if (!arg_config.isSet() || !isAccessible(arg_config.value())) {
    throw std::runtime_error(exe_name + "|ERROR, no config");
  }

  ConfigurableManager manager;
  manager.read(arg_config.value());

  auto slammer_ptr = manager.getByName<MultiGraphSLAM2D>(arg_slammer_name.value());
  auto sync_ptr    = manager.getByName<MessageSynchronizedSource>(arg_sync_name.value());
  if (!slammer_ptr || !sync_ptr) {
    throw std::runtime_error(exe_name + "|ERROR, wrong config");
  }

  // ia timings
  double total_dataset_time = 0.f;
  double total_compute_time = 0.f;
  size_t processed_msgs     = 0;

  TumContainer trajectory;
  BaseSensorMessagePtr msg = nullptr;
  total_dataset_time       = srrg2_core::getTime();
  while ((msg = sync_ptr->getMessage())) {
    LOG << "message [ " << msg->timestamp.value() << " ]\n";

    retrievePlatformSource(slammer_ptr, sync_ptr);

    double t0 = srrg2_core::getTime();
    slammer_ptr->setRawData(msg);
    slammer_ptr->compute();
    total_compute_time += (srrg2_core::getTime() - t0);

    // ia put in the trajectory
    const auto pose_2d = slammer_ptr->robotInWorld();
    const auto pose_3d = geometry3d::get3dFrom2dPose(pose_2d);
    const auto v       = geometry3d::t2tqxyzw(pose_3d);
    trajectory.insert(std::make_pair(msg->timestamp.value(), v));

    ++processed_msgs;
  }
  total_dataset_time = srrg2_core::getTime() - total_dataset_time;

  // ia timings
  const double mean_frame_time = total_compute_time / (double) (processed_msgs);
  const double mean_fps        = 1.f / mean_frame_time;

  // ia trajectory dump
  if (arg_output.isSet()) {
    LOG << "dumping TUM trajectory in [ " << arg_output.value() << " ]\n";
    std::ofstream stream(arg_output.value(), std::ofstream::out);
    for (const auto& pair : trajectory) {
      stream << pair.first << " ";
      for (int i = 0; i < pair.second.rows(); ++i) {
        stream << pair.second[i] << " ";
      }
      stream << std::endl;
    }
    stream.close();
  }

  // ia timing dump
  if (arg_output_timing.isSet()) {
    LOG << "dumping timings in [ " << arg_output_timing.value() << " ]\n";
    std::ofstream stream(arg_output_timing.value(), std::ofstream::out);
    stream << "total_frames= " << processed_msgs << " total_compute_time= " << total_compute_time
           << " mean_frame_time= " << mean_frame_time << " mean_frame_hz= " << mean_fps
           << std::endl;
    stream.close();
  }

  std::cerr
    << "\n ================================================================================\n";
  LOG << FG_GREEN("benchmark complete!") << std::endl;
  LOG << "total_dataset_time [ " << total_dataset_time << " ] s\n";
  LOG << "total_compute_time [ " << total_compute_time << " ] s\n";
  LOG << "mean_frame_time [ " << mean_frame_time << " ] s -- mean_fps [ " << mean_fps << " ] Hz\n";
}
