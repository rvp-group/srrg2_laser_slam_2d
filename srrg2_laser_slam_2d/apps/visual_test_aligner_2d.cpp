#include <iomanip>
#include <iostream>
#include <signal.h>

#include "srrg_laser_slam_2d/instances.h"
#include <srrg_data_structures/platform.h>
#include <srrg_messages/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_slam_interfaces/multi_aligner.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

using namespace srrg2_core;
using namespace srrg2_laser_tracker_2d;

const std::string exe_name = "SRRG_LASER_SLAM_2D.visual_test_aligner_2d";
#define LOG std::cerr << exe_name + "|"
void process(MessageFileSourcePtr src_, const ViewerCanvasPtr& canvas_);

int main(int argc, char** argv) {
  srrg2_core::point_cloud_registerTypes();
  srrg2_core::messages_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString arg_config_filename(&cmd, "c", "config", "config file path", "laser_config.conf");
  ArgumentString arg_message(&cmd,
                             "m",
                             "message",
                             "module you want to create, choose between {System, Tracker, Aligner}",
                             "laser_messages.boss");
  cmd.parse();

  MessageFileSourcePtr src(new MessageFileSource);
  src->open(arg_message.value());

  QApplication qapp(argc, argv);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer_core(argc, argv, &qapp);
  const ViewerCanvasPtr& canvas = viewer_core.getCanvas(exe_name);

  srrg2_qgl_viewport::ViewerCoreSharedQGL::stop();
  std::thread processing_t(process, src, canvas);
  viewer_core.startViewerServer();
  processing_t.join();

  src->close();
  return 0;
}

void process(MessageFileSourcePtr src_, const ViewerCanvasPtr& canvas_) {
  while (!srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  srrg2_core::BaseSensorMessagePtr msg;
  MeasurementAdaptorProjective2DPtr meas_adaptor(new MeasurementAdaptorProjective2D);
  std::vector<MeasurementAdaptorProjective2D::DestType> adapted_meas;
  StdVectorEigenIsometry3f odoms;
  Point3fVectorCloud pc3d;
  PointNormal2fVectorCloud global_scene;

  PlatformPtr platform(new Platform);

  while ((msg = src_->getMessage())) {
    platform->add(msg);
    if (PointCloud2MessagePtr casted_msg = std::dynamic_pointer_cast<PointCloud2Message>(msg)) {
      casted_msg->getPointCloud(pc3d);
      global_scene.reserve(pc3d.size());
      for (const auto& p : pc3d) {
        PointNormal2f pn;
        pn.coordinates().x() = p.coordinates().x();
        pn.coordinates().y() = p.coordinates().y();
        global_scene.emplace_back(pn);
      }

      NormalComputator1DSlidingWindowNormal normal_computator;
      normal_computator.computeNormals(global_scene);
      continue;
    }
    if (LaserMessagePtr casted_msg = std::dynamic_pointer_cast<LaserMessage>(msg)) {
      MeasurementAdaptorProjective2D::DestType cloud;
      meas_adaptor->setDest(&cloud);
      if (meas_adaptor->setMeasurement(msg)) {
        meas_adaptor->compute();
        adapted_meas.push_back(cloud);
      }
    }
    if (OdometryMessagePtr casted_msg = std::dynamic_pointer_cast<OdometryMessage>(msg)) {
      odoms.push_back(casted_msg->pose.value());
    }
  }

  platform->isWellFormed();
  std::cerr << platform << std::endl;
  Isometry3f sensor_in_robot_3d = Isometry3f::Identity();
  platform->getTransform(sensor_in_robot_3d, "/scan", "/base_frame");

  // srrg set multi aligner
  //  MultiAlignerLaser2DSlicePtr slice(new MultiAlignerLaser2DSlice);
  MultiAlignerLaser2DWithSensorSlicePtr slice(new MultiAlignerLaser2DWithSensorSlice);
  slice->param_fixed_slice_name.setValue("points");
  slice->param_moving_slice_name.setValue("points");
  slice->param_base_frame_id.setValue("base_frame");
  slice->param_frame_id.setValue("scan");
  slice->setPlatform(platform);
  srrg2_core::PropertyContainerDynamic fixed_prop_container;
  auto fixed = adapted_meas[1];
  Property_<PointNormal2fVectorCloud*>* prop_fixed =
    new Property_<PointNormal2fVectorCloud*>("points", "", &fixed_prop_container);
  prop_fixed->setValue(&fixed);

  srrg2_core::PropertyContainerDynamic moving_prop_container;
  auto moving = adapted_meas[0];
  Property_<PointNormal2fVectorCloud*>* prop_moving =
    new Property_<PointNormal2fVectorCloud*>("points", "", &moving_prop_container);
  prop_moving->setValue(&moving);

  Isometry2f sensor_in_robot = geometry3d::get2dFrom3dPose(sensor_in_robot_3d);
  moving.transformInPlace(sensor_in_robot);

  MultiAligner2DPtr aligner(new MultiAligner2D);
  aligner->param_slice_processors.pushBack(slice);
  aligner->setFixed(&fixed_prop_container);
  aligner->setMoving(&moving_prop_container);
  aligner->setEstimate(Isometry2f::Identity());
  aligner->compute();
  auto corrs    = slice->correspondences();
  Vector3f* asd = new Vector3f[corrs.size() * 2];
  size_t k      = 0;
  for (size_t i = 0; i < corrs.size(); ++i) {
    const auto& pc_f = (*slice->fixed())[corrs[i].fixed_idx].coordinates();
    asd[k].x()       = pc_f.x();
    asd[k].y()       = pc_f.y();
    asd[k].z()       = 0;
    ++k;
    const auto& pc_m = (*slice->moving())[corrs[i].moving_idx].coordinates();
    asd[k].x()       = pc_m.x();
    asd[k].y()       = pc_m.y();
    asd[k].z()       = 0;
    ++k;
  }

  std::cerr << "estimate: " << geometry2d::t2v(aligner->estimate()).transpose() << std::endl;
  std::cerr << "investim: " << geometry2d::t2v(aligner->estimate().inverse()).transpose()
            << std::endl;

  Isometry3f robot_prev_in_robot = srrg2_core::geometry3d::get3dFrom2dPose(aligner->estimate());

  std::cerr << "s_pose  : " << geometry3d::t2ta(sensor_in_robot_3d).transpose() << std::endl;
  std::cerr << "estsens : "
            << geometry3d::t2ta(robot_prev_in_robot.inverse() * sensor_in_robot_3d).transpose()
            << std::endl;
  std::cerr << aligner->iterationStats() << std::endl;

  while (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    {
      canvas_->putReferenceSystem(.25);

      {
        canvas_->pushColor(); // push color
        canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
        canvas_->pushPointSize(); // push point size
        canvas_->setPointSize(3);

        canvas_->putPoints(fixed);

        canvas_->popAttribute(); // pop point size
        canvas_->popAttribute(); // pop color

        canvas_->pushColor(); // push color
        canvas_->setColor(srrg2_core::ColorPalette::color3fDarkGreen());
        canvas_->pushPointSize(); // push point size
        canvas_->setPointSize(5);
        canvas_->putPoints(moving);

        canvas_->popAttribute(); // pop point size
        canvas_->popAttribute(); // pop color

        canvas_->pushColor(); // push color
        canvas_->setColor(srrg2_core::ColorPalette::color3fMagenta());
        canvas_->pushPointSize(); // push point size
        canvas_->setPointSize(2);

        canvas_->putSegment(corrs.size() * 2, asd);
        canvas_->popAttribute(); // pop point size
        canvas_->popAttribute(); // pop color
      }
    }
    {
      canvas_->pushMatrix();
      canvas_->multMatrix(robot_prev_in_robot.matrix());
      canvas_->putReferenceSystem(.5);
      canvas_->popMatrix();

      canvas_->pushMatrix();
      canvas_->multMatrix(sensor_in_robot_3d.inverse().matrix() * robot_prev_in_robot.matrix());
      canvas_->pushColor(); // push point size
      canvas_->setColor(srrg2_core::ColorPalette::color3fGreen());
      canvas_->pushPointSize(); // push point size
      canvas_->setPointSize(5);
      canvas_->putPoints(moving);

      canvas_->popAttribute(); // pop point size
      canvas_->popAttribute(); // pop color
    }

    canvas_->flush();
  }
}
