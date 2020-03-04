#include <iomanip>
#include <iostream>
#include <signal.h>

#include "srrg_laser_slam_2d/instances.h"
#include <srrg_config/configurable_manager.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/instances.h>
#include <srrg_messages_ros/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_slam_interfaces/instances.h>
#include <srrg_slam_interfaces/multi_aligner.h>
#include <srrg_slam_interfaces/multi_tracker.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

using namespace srrg2_core;
using namespace srrg2_laser_tracker_2d;

const std::string exe_name = "SRRG_LASER_SLAM_2D.visual_test_tracker_2d";
#define LOG std::cerr << exe_name + "|"
void process(std::shared_ptr<ConfigurableManager> src_, const ViewerCanvasPtr& canvas_);
void draw(const ViewerCanvasPtr& canvas_);
void generateConfig(std::shared_ptr<ConfigurableManager> manager);

int main(int argc, char** argv) {
  srrgInit(argc, argv, "srrg_node");
  srrg2_core::point_cloud_registerTypes();
  srrg2_core::messages_registerTypes();
  srrg2_core_ros::messages_ros_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString arg_gen_config(&cmd, "j", "gen-config", "config file path", "laser_config.conf");
  ArgumentString arg_config_filename(&cmd, "c", "config", "config file path", "laser_config.conf");
  ArgumentString arg_message(&cmd, "m", "message", "messages' file", "laser_messages.boss");
  cmd.parse();

  std::shared_ptr<ConfigurableManager> manager(new ConfigurableManager);
  if (arg_gen_config.isSet()) {
    generateConfig(manager);
    manager->write(arg_gen_config.value());
    return 0;
  }
  manager->read(arg_config_filename.value());

  QApplication qapp(argc, argv);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer_core(argc, argv, &qapp);
  const ViewerCanvasPtr& canvas = viewer_core.getCanvas(exe_name);

  srrg2_qgl_viewport::ViewerCoreSharedQGL::stop();
  std::thread processing_t(process, manager, canvas);
  viewer_core.startViewerServer();
  processing_t.join();
  return 0;
}

MultiTrackerLaser2DSlicePtr tracker_laser_slice = nullptr;
MultiTracker2DPtr tracker                       = nullptr;
PlatformPtr platform                            = nullptr;

void generateConfig(std::shared_ptr<ConfigurableManager> manager) {
  std::vector<std::string> topics;
  topics.push_back("/scan");
  topics.push_back("/odom");
  topics.push_back("/tf");

  MessageFileSourcePtr src = manager->create<MessageFileSource>("file_source");
  src->param_filename.setValue("laser_messages.boss");

  MessageSourcePlatformPtr platform_source =
    manager->create<MessageSourcePlatform>("platform_source");
  platform_source->param_source.setValue(src);
  platform_source->param_tf_topics.setValue(std::vector<std::string>(1, "/tf"));

  MessageSynchronizedSourcePtr sync = manager->create<MessageSynchronizedSource>("sync");
  sync->param_source.setValue(platform_source);
  sync->param_topics.setValue(topics);

  // srrg setup aligner
  MultiAlignerLaser2DWithSensorSlicePtr aligner_laser_slice =
    manager->create<MultiAlignerLaser2DWithSensorSlice>("aligner_laser_slice");
  aligner_laser_slice->param_fixed_slice_name.setValue("points");
  aligner_laser_slice->param_moving_slice_name.setValue("points");
  aligner_laser_slice->param_base_frame_id.setValue("base_frame");
  aligner_laser_slice->param_frame_id.setValue("scan");

  MultiAligner2DSliceOdomPriorPtr aligner_odom_slice =
    manager->create<MultiAligner2DSliceOdomPrior>("aligner_odom_slice");
  aligner_odom_slice->param_fixed_slice_name.setValue("odom");
  aligner_odom_slice->param_moving_slice_name.setValue("odom");
  aligner_odom_slice->param_base_frame_id.setValue("odom");
  aligner_odom_slice->param_frame_id.setValue("base_frame");

  MultiAligner2DPtr aligner = manager->create<MultiAligner2D>("aligner");
  aligner->param_slice_processors.pushBack(aligner_laser_slice);
  aligner->param_slice_processors.pushBack(aligner_odom_slice);

  tracker_laser_slice = manager->create<MultiTrackerLaser2DSlice>("tracker_laser_slice");
  tracker_laser_slice->param_base_frame_id.setValue("base_frame");
  tracker_laser_slice->param_frame_id.setValue("scan");

  MultiTracker2DSliceOdomPriorPtr tracker_odom_slice =
    manager->create<MultiTracker2DSliceOdomPrior>("tracker_odom_slice");
  tracker_odom_slice->param_measurement_slice_name.setValue("odom");
  tracker_odom_slice->param_scene_slice_name.setValue("odom");
  tracker_odom_slice->param_base_frame_id.setValue("odom");
  tracker_odom_slice->param_frame_id.setValue("base_frame");

  tracker = manager->create<MultiTracker2D>("tracker");
  tracker->param_aligner.setValue(aligner);
  tracker->param_slice_processors.pushBack(tracker_laser_slice);
  tracker->param_slice_processors.pushBack(tracker_odom_slice);
}

void process(std::shared_ptr<ConfigurableManager> manager_, const ViewerCanvasPtr& canvas_) {
  while (!srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto sync            = manager_->getByName<MessageSynchronizedSource>("sync");
  tracker_laser_slice  = manager_->getByName<MultiTrackerLaser2DSlice>("tracker_laser_slice");
  tracker              = manager_->getByName<MultiTracker2D>("tracker");
  auto platform_source = manager_->getByName<MessageSourcePlatform>("platform_source");

  srrg2_core::BaseSensorMessagePtr msg;
  PointNormal2fVectorCloud global_scene;
  Point3fVectorCloud pc3d;

  // srrg create property for scene
  srrg2_core::PropertyContainerDynamic scene_container;
  Property_<PointNormal2fVectorCloud*>* prop_scene =
    new Property_<PointNormal2fVectorCloud*>("points", "", &scene_container);
  prop_scene->setValue(&global_scene);

  tracker->populateScene(scene_container);
  tracker->setScene(&scene_container);
  //  tracker->setEstimate(Isometry2f::Identity());

  std::cerr << "reading msgs" << std::endl;
  bool triggered = false;
  while (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    if ((msg = sync->getMessage())) {
      if (!platform) {
        std::cerr << "getting platform" << std::endl;
        platform = platform_source->platform("/tf");
        std::cerr << platform << std::endl;
        tracker->setPlatform(platform);
      }

      //      if (PointCloud2MessagePtr casted_msg =
      //      std::dynamic_pointer_cast<PointCloud2Message>(msg)) {
      //        casted_msg->getPointCloud(pc3d);
      //        global_scene.reserve(pc3d.size());
      //        for (const auto& p : pc3d) {
      //          PointNormal2f pn;
      //          pn.coordinates().x() = p.coordinates().x();
      //          pn.coordinates().y() = p.coordinates().y();
      //          global_scene.emplace_back(pn);
      //        }
      //
      //        NormalComputator1DSlidingWindowNormal normal_computator;
      //        normal_computator.computeNormals(global_scene);
      //        continue;
      //      }
      //      if (LaserMessagePtr casted_msg = std::dynamic_pointer_cast<LaserMessage>(msg)) {
      tracker->setMeasurement(msg);
      std::cerr << "setMeasurement" << std::endl;
      draw(canvas_);
      std::cin.get();
      tracker->adaptMeasurements();
      std::cerr << "adaptMeasurements" << std::endl;
      draw(canvas_);
      std::cin.get();
      tracker->align();
      std::cerr << "align" << std::endl;
      draw(canvas_);
      std::cin.get();
      tracker->merge();
      std::cerr << "merge" << std::endl;
      draw(canvas_);
      std::cin.get();
      triggered = true;
      //      }
      //      if (OdometryMessagePtr casted_msg = std::dynamic_pointer_cast<OdometryMessage>(msg)) {
      //        odoms.push_back(casted_msg->pose.value());
      //      }
    } else {
      break;
    }

    if (!triggered) {
      continue;
    }

    triggered = false;
  }

  while (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    draw(canvas_);
  }
}

void draw(const ViewerCanvasPtr& canvas_) {
  Isometry3f sensor_in_robot_3d = Isometry3f::Identity();
  if (!platform->getTransform(sensor_in_robot_3d, "diago_0/laser_frame", "diago_0/base_frame")) {
    return;
  }

  // srrg draw
  // srrg draw origin reference frame
  canvas_->putReferenceSystem(.15);
  // srrg put scene
  {
    canvas_->pushColor(); // push color
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlack());
    canvas_->pushPointSize(); // push point size
    canvas_->setPointSize(1);

    canvas_->putPoints(*tracker_laser_slice->scene());

    canvas_->popAttribute(); // pop point size
    canvas_->popAttribute(); // pop color

    canvas_->pushColor(); // push color
    canvas_->setColor(srrg2_core::ColorPalette::color3fGreen());
    canvas_->pushPointSize(); // push point size
    canvas_->setPointSize(3);

    canvas_->putPoints(*tracker_laser_slice->clippedScene());

    canvas_->popAttribute(); // pop point size
    canvas_->popAttribute(); // pop color
  }
  {
    canvas_->pushMatrix();
    canvas_->multMatrix((geometry3d::get3dFrom2dPose(tracker->estimate())).matrix());

    canvas_->putReferenceSystem(.1);
    canvas_->popMatrix();

    canvas_->pushMatrix();
    canvas_->multMatrix(
      (geometry3d::get3dFrom2dPose(tracker->estimate()) * sensor_in_robot_3d).matrix());
    canvas_->putReferenceSystem(.25);

    canvas_->pushColor(); // push color
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
    canvas_->pushPointSize(); // push point size
    canvas_->setPointSize(4);

    canvas_->putPoints(*tracker_laser_slice->measurements());

    canvas_->popAttribute(); // pop point size
    canvas_->popAttribute(); // pop color

    canvas_->popMatrix();
  }

  canvas_->flush();
}
