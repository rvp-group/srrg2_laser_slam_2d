// messages
#include "srrg_messages/instances.h"
#include "srrg_messages/message_handlers/message_file_source.h"
#include "srrg_messages/message_handlers/message_sorted_source.h"
#include "srrg_messages/message_handlers/message_synchronized_source.h"
#include "srrg_messages/messages/laser_message.h"

// helper stuff
#include "srrg_config/configurable_manager.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/shell_colors.h"
#include "srrg_system_utils/system_utils.h"

// for converting laser to cloud
#include "srrg_geometry/geometry2d.h"
#include <srrg_pcl/point_projector_types.h>

// the meat
#include "srrg_laser_slam_2d/aligner_2d.h"
#include "srrg_laser_tracker_2d/instances.h"
#include "srrg_laser_tracker_2d/merger_projective_2d.h"

#include <signal.h>

using namespace srrg2_core;
using namespace srrg2_laser_tracker_2d;
using namespace std;

const char* banner[] = {"This program aligns (sequentially) scans provided in a boss file",
                        "At the moment the aligner ignores the odometry, and any other information",
                        "it generates a pipeline source->sorter->aligner",
                        "usage: [options] filename",
                        0};

volatile bool run = true;
static void sigIntHandler(int __attribute__((unused))) {
  cerr << "Got user interrupt, Terimnating" << endl;
  run = false;
}

void generateConfig(const std::string& config_file,
                    const std::string& source_name,
                    const std::string& sorter_name,
                    const std::string& aligner_name,
                    const std::string& merger_name_) {
  ConfigurableManager manager;
  auto aligner     = manager.createConfigurable<Aligner2D>(aligner_name);
  auto merger      = manager.createConfigurable<MergerProjective2D>(merger_name_);
  auto file_source = manager.createConfigurable<MessageFileSource>(source_name);

  auto sorter = manager.createConfigurable<MessageSortedSource>(sorter_name);
  sorter->param_source.setValue(file_source);

  cerr << "generating config on file[" << config_file << "]" << endl;
  manager.write(config_file);
}

void initUnprojector(PointNormal2fUnprojectorPolar& unprojector, LaserMessagePtr msg) {
  unprojector.param_range_min.setValue(msg->range_min.value());
  unprojector.param_range_max.setValue(msg->range_max.value());
  cerr << "unprojector initialized!" << endl;
}

int main(int argc, char** argv) {
  using namespace geometry2d;
  messages_registerTypes();
  laser_tracker_2d_registerTypes();
  ParseCommandLine cmd_line(argv, banner);
  ArgumentFlag conf_gen(&cmd_line, "j", "generate-config", "generates a config file");
  ArgumentFlag verbose(&cmd_line, "v", "verbose", "talks a lot");
  ArgumentString aligner_name(&cmd_line, "na", "name-aligner", "aligner name", "aligner");
  ArgumentString merger_name(&cmd_line, "nm", "name-merger", "merger name", "merger");
  ArgumentString sorter_name(&cmd_line, "ns", "name-sorter", "sorter name", "sorter");
  ArgumentString source_name(&cmd_line, "ni", "name-source", "source name", "source");
  ArgumentString config_file(
    &cmd_line, "c", "config-file ", "config file to read/write", "merger.conf");
  ArgumentString output_file(&cmd_line, "o", "output ", "file where to write the trajectory", "");
  ArgumentInt skip(
    &cmd_line, "s", "skip ", "frames to skip between matches (0 disables processing)", 1);
  cmd_line.parse();
  if (conf_gen.isSet()) {
    generateConfig(config_file.value(),
                   source_name.value(),
                   sorter_name.value(),
                   aligner_name.value(),
                   merger_name.value());
    return 0;
  }

  // read configuration
  ConfigurableManager manager;
  manager.read(config_file.value());
  cerr << "reading config from [" << config_file.value() << "]" << endl;
  // casts
  Aligner2DPtr aligner          = manager.getConfigurable<Aligner2D>(aligner_name.value());
  Merger2DPtr merger            = manager.getConfigurable<Merger2D>(merger_name.value());
  MessageSortedSourcePtr sorter = manager.getConfigurable<MessageSortedSource>(sorter_name.value());
  MessageFileSourcePtr source   = manager.getConfigurable<MessageFileSource>(source_name.value());

  if (!(merger && aligner && sorter && source)) {
    cerr << "config not ready, exiting" << endl;
    return 0;
  }
  cerr << "config loaded, modules ready" << endl;

  if (cmd_line.lastParsedArgs().empty()) {
    cerr << "no input file specified, exiting" << endl;
    return 0;
  }
  const std::string& input_file = cmd_line.lastParsedArgs()[0];

  signal(SIGINT, sigIntHandler);

  // open input file
  source->open(input_file);

  // open output file and initialize cumulative trajectory
  ofstream os;
  if (output_file.isSet() && output_file.value().length())
    os.open(output_file.value().c_str());
  Isometry2f trajectory;
  trajectory.setIdentity();

  // this is used to convert the scans into point clouds
  PointNormal2fUnprojectorPolar unprojector;
  // these holds the previous and current scans
  PointNormal2fVectorCloud fixed, moving, full_scene;

  // total number of frames
  int count = 0;

  // to see the resources used
  SystemUsageCounter usage_counter;
  int num_frames_processed = 0;

  BaseSensorMessagePtr msg;
  double processing_time = 0;

  merger->setScene(&full_scene);

  while (run && (msg = source->getMessage())) {
    LaserMessagePtr laser = dynamic_pointer_cast<LaserMessage>(msg);
    if (laser) {
      Matrix_<float> range_matrix;
      range_matrix.resize(laser->ranges.size(), 1);
      for (size_t i = 0; i < laser->ranges.size(); ++i)
        range_matrix.at(i, 1) = laser->ranges.value().at(i);

      if (!count) {
        initUnprojector(unprojector, laser);
        std::back_insert_iterator<PointNormal2fVectorCloud> fixed_back_insertor(fixed);
        unprojector.compute<WithNormals>(fixed_back_insertor, range_matrix);
      }
      if (skip.value() && count && !(count % skip.value())) {
        usage_counter.tic();
        std::back_insert_iterator<PointNormal2fVectorCloud> moving_back_insertor(moving);
        unprojector.compute<WithNormals>(moving_back_insertor, range_matrix);
        aligner->setFixed(&fixed);
        aligner->setMoving(&moving);
        aligner->setEstimate(Eigen::Isometry2f::Identity());
        aligner->compute();

        // cerr << aligner->iterationStats() << endl;
        if (verbose.isSet()) {
          cerr << "frame: " << count << " f: " << fixed.size() << " m: " << moving.size()
               << " p: " << FG_CYAN(full_scene.size()) << " status:  " << aligner->status() << endl;
        }
        trajectory = v2t(t2v(trajectory * aligner->estimate()));

        // ia merge
        merger->setMoving(&moving);
        merger->setTransform(trajectory);
        merger->compute();

        fixed = moving;
        ++num_frames_processed;
        processing_time += usage_counter.toc();
      }
      ++count;

      // cin.get();
    }
  }

  if (os.good())
    for (auto p : full_scene) {
      os << p.coordinates().transpose() << endl;
    }

  usage_counter.update();

  manager.write(config_file.value());
  cerr << "Done" << endl;
  cerr << "Processed " << num_frames_processed << " frames in " << processing_time << "sec ("
       << num_frames_processed / processing_time << " frames/sec)" << endl;
  cerr << "Avg Memory: " << usage_counter.totalMemory() / 1024 << " Kb" << endl;
}
