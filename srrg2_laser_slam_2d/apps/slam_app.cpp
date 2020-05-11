#include "srrg2_laser_slam_2d/instances.h"
#include <srrg2_slam_interfaces/instances.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_messages/instances.h>
#include <srrg_messages_ros/instances.h>
#include <srrg_messages_ros/message_handlers/message_rosbag_source.h>
#include <srrg_pcl/instances.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/parse_command_line.h>

using namespace srrg2_laser_slam_2d;
using namespace srrg2_qgl_viewport;
using namespace srrg2_slam_interfaces;
using namespace srrg2_core;
using namespace srrg2_core_ros;

void process(ViewerCanvasPtr canvas_,
             MessageSynchronizedSourcePtr synchronizer_,
             MultiGraphSLAM2DPtr slammer_);

void generateConfig(ConfigurableManager& manager_, const std::string& filepath_);

int main(int argc, char** argv) {
  messages_registerTypes();
  messages_ros_registerTypes();
  srrg2_laser_slam_2d_registerTypes();
  point_cloud_registerTypes();

  srrgInit(argc, argv, "laser_slam2d_app");
  ParseCommandLine command_line_parser(argv);
  ArgumentString config_file(
    &command_line_parser, "c", "configuration", "config file to read/write", "laser_slam2d.conf");
  ArgumentFlag generate_config(&command_line_parser,
                               "j",
                               "generate-config",
                               "generate a default configuration for laser slam 2d");
  ArgumentString input_bag(&command_line_parser, "i", "input", "input rosbag", "");
  command_line_parser.parse();
  ConfigurableManager manager;
  if (generate_config.isSet()) {
    generateConfig(manager, config_file.value());
    return -1;
  }
  manager.read(config_file.value());
  MultiGraphSLAM2DPtr slammer = manager.getByName<MultiGraphSLAM2D>("slam");
  if (!slammer) {
    std::cerr << "ERROR: could not instantiate tracker from configuration file: "
              << config_file.value() << std::endl;
    return -1;
  }

  // ds set up dataset playback from configuration
  MessageSynchronizedSourcePtr synchronizer = manager.getByName<MessageSynchronizedSource>("sync");
  if (!synchronizer) {
    std::cerr << "ERROR: could not load dataset from configuration file: " << config_file.value()
              << std::endl;
    return -1;
  }

  if (input_bag.value() != "") {
    MessageFileSourceBase* src =
      dynamic_cast<MessageFileSourceBase*>(synchronizer->getRootSource());
    src->open(input_bag.value());
  }
  // ds launch viewer and processing thread (points are not changed at this point)
  QApplication qapp(argc, argv);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("laser_slam2d_app");
  std::thread processing_thread(process, canvas, synchronizer, slammer);

  // ds display viewer until termination signal is received
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}

void process(ViewerCanvasPtr canvas_,
             MessageSynchronizedSourcePtr synchronizer_,
             MultiGraphSLAM2DPtr slammer_) {
  BaseSensorMessagePtr msg = nullptr;
  while (msg = synchronizer_->getMessage()) {
    slammer_->putMessage(msg);
    slammer_->draw(canvas_);
  }
}

void generateConfig(ConfigurableManager& manager_, const std::string& filepath_) {
  // tg slam algorithm
  MultiGraphSLAM2DPtr slammer = manager_.create<MultiGraphSLAM2D>("slam");
  // tg tracker params
  MultiTracker2DPtr tracker = manager_.create<MultiTracker2D>("tracker");

  std::shared_ptr<TrackerSliceProcessorLaser2D> tracker_slice =
    manager_.create<TrackerSliceProcessorLaser2D>();

  tracker->param_slice_processors.pushBack(tracker_slice);

  std::shared_ptr<TrackerSliceProcessorPriorOdom2D> odom_slice =
    manager_.create<TrackerSliceProcessorPriorOdom2D>();

  tracker->param_slice_processors.pushBack(odom_slice);
  // tg aligner params
  MultiAligner2DPtr aligner = manager_.create<MultiAligner2D>("aligner");

  AlignerSliceProcessorLaser2DPtr aligner_slice = manager_.create<AlignerSliceProcessorLaser2D>();
  aligner->param_slice_processors.pushBack(aligner_slice);

  AlignerSliceOdom2DPriorPtr aligner_odom_slice = manager_.create<AlignerSliceOdom2DPrior>();

  aligner->param_slice_processors.pushBack(aligner_odom_slice);
  // tg add aligner to tracker
  tracker->param_aligner.setValue(aligner);
  // tg add tracker to slammer
  slammer->param_tracker.setValue(tracker);

  // set up local map splitting criteria for slammer
  std::shared_ptr<LocalMapSplittingCriterionDistance2D> splitting_criteria =
    manager_.create<LocalMapSplittingCriterionDistance2D>("splitting_criteria");
  slammer->param_splitting_criterion.setValue(splitting_criteria);

  // tg set up loop detector
  std::shared_ptr<MultiLoopDetectorBruteForce2D> loop_detector =
    manager_.create<MultiLoopDetectorBruteForce2D>("loop_detector");
  std::shared_ptr<LocalMapSelectorBreadthFirst2D> local_map_selector =
    manager_.create<LocalMapSelectorBreadthFirst2D>("selector");

  MultiAligner2DPtr aligner_loop_detector = manager_.create<MultiAligner2D>();
  AlignerSliceProcessorLaser2DPtr aligner_loop_detector_slice =
    manager_.create<AlignerSliceProcessorLaser2D>();

  aligner_loop_detector->param_slice_processors.pushBack(aligner_loop_detector_slice);
  loop_detector->param_relocalize_aligner.setValue(aligner_loop_detector);
  loop_detector->param_local_map_selector.setValue(local_map_selector);
  // tg set up relocalizer
  std::shared_ptr<MultiRelocalizer2D> relocalizer =
    manager_.create<MultiRelocalizer2D>("relocalizer");
  MultiAligner2DPtr aligner_relocalizer = manager_.create<MultiAligner2D>();

  AlignerSliceProcessorLaser2DPtr aligner_relocalizer_slice =
    manager_.create<AlignerSliceProcessorLaser2D>();

  aligner_relocalizer->param_slice_processors.pushBack(aligner_relocalizer_slice);
  relocalizer->param_aligner.setValue(aligner_relocalizer);
  // tg set up solver
  SolverPtr global_solver = manager_.create<Solver>("global_solver");

  RobustifierPolicyByTypePtr robustifiers = manager_.create<RobustifierPolicyByType>();
  robustifiers->param_factor_class_name.setValue("SE2PosePoseGeodesicErrorFactor");

  std::shared_ptr<RobustifierCauchy> r = manager_.create<RobustifierCauchy>();
  robustifiers->param_robustifier.setValue(r);

  global_solver->param_robustifier_policies.pushBack(robustifiers);
  slammer->param_global_solver.setValue(global_solver);

  // tg set up source
  std::shared_ptr<MessageROSBagSource> rosbag = manager_.create<MessageROSBagSource>("src");
  MessageSortedSourcePtr sorter               = manager_.create<MessageSortedSource>("sorter");
  MessageSynchronizedSourcePtr sync           = manager_.create<MessageSynchronizedSource>("sync");
  std::shared_ptr<MessageOdomSubsamplerSource> odom_sub =
    manager_.create<MessageOdomSubsamplerSource>("odom_sub");
  sorter->param_source.setValue(rosbag);
  odom_sub->param_source.setValue(sorter);
  sync->param_source.setValue(odom_sub);

  manager_.write(filepath_);
  std::cerr << "Configuration writter in: " << FG_BYELLOW(filepath_) << std::endl;
}
