#include <iomanip>
#include <iostream>
#include <signal.h>

#include "srrg_laser_slam_2d/correspondence_finder_projective_2d.h"
#include "srrg_laser_slam_2d/measurement_adaptor_projective_2d.h"
#include <srrg_messages/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_slam_interfaces/measurement_adaptor.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

using namespace srrg2_core;
using namespace srrg2_laser_tracker_2d;

const std::string exe_name = "SRRG_LASER_SLAM_2D.visual_test_correspondence_finder";
#define LOG std::cerr << exe_name + "|"
// const std::string test_data_folder(SRRG2_SHASLAM_TEST_DATA_FOLDER);

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

  while ((msg = src_->getMessage())) {
    if (LaserMessagePtr casted_msg = std::dynamic_pointer_cast<LaserMessage>(msg)) {
      MeasurementAdaptorProjective2D::DestType cloud;
      meas_adaptor->setDest(&cloud);
      if (meas_adaptor->setMeasurement(msg)) {
        meas_adaptor->compute();
        adapted_meas.push_back(cloud);
      }
    }
  }

  Isometry2f robot_pose = geometry2d::v2t(Vector3f(.2f, .02f, -M_PI * .05f));

  CorrespondenceVector correspondences;
  CorrespondenceFinderProjective2DPtr cf(new CorrespondenceFinderProjective2D);
  cf->setFixed(&adapted_meas[0]);
  cf->setMoving(&adapted_meas[1]);
  cf->setEstimate(robot_pose);
  cf->setCorrespondences(&correspondences);
  cf->compute();

  const size_t corr_double_size = correspondences.size() * 2;
  Vector3f* corrs               = new Vector3f[corr_double_size];
  size_t k                      = 0;
  for (size_t i = 0; i < correspondences.size(); ++i) {
    const auto& pc_f = adapted_meas[0][correspondences[i].fixed_idx].coordinates();
    corrs[k].x()     = pc_f.x();
    corrs[k].y()     = pc_f.y();
    corrs[k].z()     = 0;
    ++k;
    const auto& pc_m = adapted_meas[1][correspondences[i].moving_idx].coordinates();
    corrs[k].x()     = pc_m.x();
    corrs[k].y()     = pc_m.y();
    corrs[k].z()     = 0;
    ++k;
  }

  std::cerr << "number of correspondences: " << correspondences.size() << std::endl;

  while (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    canvas_->pushColor(); // push color
    canvas_->setColor(srrg2_core::ColorPalette::color3fRed());
    canvas_->pushPointSize(); // push point size
    canvas_->setPointSize(3);
    canvas_->putPoints(adapted_meas[0]);

    canvas_->popAttribute(); // pop point size
    canvas_->popAttribute(); // pop color

    canvas_->pushColor(); // push color
    canvas_->setColor(srrg2_core::ColorPalette::color3fGreen());
    canvas_->pushPointSize(); // push point size
    canvas_->setPointSize(3);
    canvas_->putPoints(adapted_meas[1]);

    canvas_->popAttribute(); // pop point size
    canvas_->popAttribute(); // pop color

    canvas_->putSegment(corr_double_size, corrs);

    canvas_->flush();
  }
  delete[] corrs;

  std::cerr << "DONE" << std::endl;
}
