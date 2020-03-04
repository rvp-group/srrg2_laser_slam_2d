#include <iomanip>
#include <iostream>
#include <signal.h>

#include "srrg_laser_slam_2d/measurement_adaptor_projective_2d.h"
#include "srrg_laser_slam_2d/merger_projective_2d.h"
#include "srrg_laser_slam_2d/scene_clipper_projective_2d.h"
#include <srrg_data_structures/platform.h>
#include <srrg_messages/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_slam_interfaces/measurement_adaptor.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

using namespace srrg2_core;
using namespace srrg2_laser_tracker_2d;

const std::string exe_name = "SRRG_LASER_SLAM_2D.visual_test_merger_projective_2d";
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
  PointNormal2fVectorCloud local_scene0, local_scene1;

  Platform platform;

  while ((msg = src_->getMessage())) {
    platform.add(msg);
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

  platform.isWellFormed();
  std::cerr << platform << std::endl;
  Isometry3f s_pose = Isometry3f::Identity();
  platform.getTransform(s_pose, "/scan", "/base_frame");
  Isometry2f sensor_in_world = geometry3d::get2dFrom3dPose(odoms[0] * s_pose);

  SceneClipperProjective2DPtr clipper(new SceneClipperProjective2D);
  clipper->param_projector->param_angle_col_min.setValue(-M_PI_2);
  clipper->param_projector->param_angle_col_max.setValue(M_PI_2);
  clipper->setGlobalScene(&global_scene);
  clipper->setLocalScene(&local_scene0);
  clipper->setTransform(sensor_in_world);
  clipper->compute();

  // local scene is in world frame
  PointNormal2fVectorCloud merged_scene = local_scene0.transform<Isometry>(sensor_in_world);

  sensor_in_world = geometry3d::get2dFrom3dPose(odoms[1] * s_pose);

  clipper->setLocalScene(&local_scene1);
  clipper->setTransform(sensor_in_world);
  clipper->compute();

  MergerProjective2DPtr merger(new MergerProjective2D);
  merger->setScene(&merged_scene);
  merger->setMoving(&local_scene1);
  merger->setTransform(sensor_in_world);
  merger->compute();

  std::cerr << "local scene 0 size: " << local_scene0.size() << std::endl;
  std::cerr << "local scene 1 size: " << local_scene1.size() << std::endl;
  std::cerr << "merged scene  size: " << merged_scene.size() << std::endl;

  while (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    {
      canvas_->pushColor(); // push color
      canvas_->setColor(srrg2_core::ColorPalette::color3fBlack());
      canvas_->pushPointSize(); // push point size
      canvas_->setPointSize(1);

      canvas_->putPoints(pc3d);

      canvas_->popAttribute(); // pop point size
      canvas_->popAttribute(); // pop color
    }

    {
      canvas_->pushColor(); // push color
      canvas_->setColor(srrg2_core::ColorPalette::color3fRed());
      canvas_->pushPointSize(); // push point size
      canvas_->setPointSize(3);

      canvas_->putPoints(merged_scene);

      canvas_->popAttribute(); // pop point size
      canvas_->popAttribute(); // pop color
    }

    {
      canvas_->multMatrix(odoms[0].matrix());
      canvas_->pushMatrix();

      canvas_->putReferenceSystem(.5);

      {
        canvas_->multMatrix(s_pose.matrix());
        canvas_->pushMatrix();

        canvas_->putReferenceSystem(.25);

        {
          canvas_->pushColor(); // push color
          canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
          canvas_->pushPointSize(); // push point size
          canvas_->setPointSize(5);
          canvas_->putPoints(local_scene0);

          canvas_->popAttribute(); // pop point size
          canvas_->popAttribute(); // pop color
        }
        canvas_->popMatrix();
      }
      canvas_->popMatrix();
    }

    {
      canvas_->multMatrix(odoms[1].matrix());
      canvas_->pushMatrix();

      canvas_->putReferenceSystem(1);

      {
        canvas_->multMatrix(s_pose.matrix());
        canvas_->pushMatrix();

        canvas_->putReferenceSystem(.25);

        {
          canvas_->pushColor(); // push color
          canvas_->setColor(srrg2_core::ColorPalette::color3fGreen());
          canvas_->pushPointSize(); // push point size
          canvas_->setPointSize(5);
          canvas_->putPoints(local_scene1);

          canvas_->popAttribute(); // pop point size
          canvas_->popAttribute(); // pop color
        }

        canvas_->popMatrix();
      }

      canvas_->popMatrix();
    }

    canvas_->flush();
  }
}
