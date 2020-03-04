#include <signal.h>

// messages
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_file_source.h>

//helper stuff
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>

#include <srrg_pcl/instances.h>
#include <srrg_geometry/geometry2d.h>

// correspondence finders to be tested
#include <srrg_laser_tracker_2d/correspondence_finder_nn_2d.h>
#include <srrg_laser_tracker_2d/correspondence_finder_kd_tree_2d.h>
#include <srrg_laser_tracker_2d/correspondence_finder_projective_2d.h>
#include <srrg_laser_tracker_2d/instances.h>

#define TEST_LOG std::cerr << "test_correspondence_finder_2d| "

using namespace std;
using namespace srrg2_core;
using namespace srrg2_laser_tracker_2d;

const char* banner[]={
  "This program shows the correspondences between sequencial",
  "scans provided in a boss file.",
  "You can switch between finders [nn/kd-tree/projective]",
  "",
  "usage: [options] filename",
  0
};

volatile bool run=true;
static void sigIntHandler(int __attribute__((unused))) {
  cerr << "Got user interrupt, Terminating" << endl;
  run=false;
}

void generateConfig(const std::string& config_file,
                    const std::string& source_name,
                    const std::string& sorter_name,
                    const std::string& correspondence_finder_name) {
  ConfigurableManager manager;

  auto  correspondence_finder_nn=manager.createConfigurable<CorrespondenceFinderNN2D>(correspondence_finder_name + "_nn");
  auto correspondence_finder_kd_tree = manager.createConfigurable<CorrespondenceFinderKDTree2D>(correspondence_finder_name + "_kd_tree");
  auto correspondence_finder_proj    = manager.createConfigurable<CorrespondenceFinderProjective2D>(correspondence_finder_name + "_projective");
  auto file_source = manager.createConfigurable<MessageFileSource>(source_name);
  auto sorter      = manager.createConfigurable<MessageSortedSource>(sorter_name);

  TEST_LOG << "Generating config on file[" << FG_GREEN(config_file) << "]" << endl;
  manager.write(config_file);
}

void initProjectors(PointNormal2fUnprojectorPolarPtr unprojector,
                    PointNormal2fProjectorPolarPtr projector,
                    LaserMessagePtr msg){
  const size_t num_beams = msg->ranges.value().size();
  const float sensor_resolution = (msg->angle_max.value() - msg->angle_min.value()) / (float)num_beams;
  Matrix2f sensor_matrix;
  sensor_matrix << 1.f / sensor_resolution, num_beams / 2.f, 0.f, 1.f;

  projector->param_canvas_cols.setValue(msg->ranges.value().size());
  projector->param_range_min.setValue(msg->range_min.value());
  projector->param_range_max.setValue(msg->range_max.value());
  projector->param_angle_col_min.setValue(msg->angle_min.value());
  projector->param_angle_col_max.setValue(msg->angle_max.value());
  projector->setCameraMatrix(sensor_matrix);
  projector->setCameraPose(Isometry2f::Identity());
  
  unprojector->param_range_min.setValue(msg->range_min.value());
  unprojector->param_range_max.setValue(msg->range_max.value());
  unprojector->setCameraMatrix(sensor_matrix);
  unprojector->setCameraPose(Isometry2f::Identity());
  TEST_LOG << "Projectors initialized with sensor matrix:\n" << sensor_matrix << endl;  
}


int main(int argc, char** argv){
  using namespace geometry2d;
  messages_registerTypes();
  point_cloud_registerTypes(); 
  laser_tracker_2d_registerTypes();
  
  ParseCommandLine cmd_line(argv,banner);
  ArgumentFlag   conf_gen    (&cmd_line, "j",  "generate-config",
                              "generates a config file");
  ArgumentFlag   verbose     (&cmd_line, "v",  "verbose",
                              "talks a lot");
  ArgumentString corfind_name(&cmd_line, "nc", "name-correspondence-finder",
                              "correspondence_finder name", "correspondence_finder");
  ArgumentString sorter_name (&cmd_line, "ns", "name-sorter",
                              "sorter name",  "sorter");
  ArgumentString source_name (&cmd_line, "ni", "name-source",
                              "source name",  "source");
  ArgumentString config_file (&cmd_line, "c",  "config-file",
                              "config file to read/write", "test_correspondence_finder2d.conf");
  ArgumentString output_file (&cmd_line, "o",  "output",
                              "file where to write the trajectory", "");
  ArgumentInt    skip        (&cmd_line, "s",  "skip",
                              "frames to skip between matches (0 disables processing)", 1);
  cmd_line.parse();
  if(conf_gen.isSet()){
    generateConfig(config_file.value(),
                   source_name.value(),
                   sorter_name.value(),
                   corfind_name.value());
    return 0;
  }

  // read configuration
  ConfigurableManager manager;
  manager.read(config_file.value());
  TEST_LOG << "reading config from [" << FG_GREEN(config_file.value()) << "]" << endl;
  // casts
  const std::string& corr_find_name = corfind_name.value();
  CorrespondenceFinderNN2DPtr         corr_finder_nn = manager.getConfigurable<CorrespondenceFinderNN2D>(corr_find_name+"_nn");
  CorrespondenceFinderKDTree2DPtr     corr_finder_kd_tree = manager.getConfigurable<CorrespondenceFinderKDTree2D>(corr_find_name+"_kd_tree");
  CorrespondenceFinderProjective2DPtr corr_finder_proj = manager.getConfigurable<CorrespondenceFinderProjective2D>(corr_find_name+"_projective");
  MessageSortedSourcePtr sorter=manager.getConfigurable<MessageSortedSource>(sorter_name.value());
  MessageFileSourcePtr source=manager.getConfigurable<MessageFileSource>(source_name.value());

  if (! (corr_finder_nn && corr_finder_kd_tree && corr_finder_proj && sorter && source) ) {
    TEST_LOG << "Config not ready, exiting" << endl;
    return 0;
  }
  TEST_LOG << "Config loaded, modules ready" << endl;

  if(cmd_line.lastParsedArgs().empty()) {
    TEST_LOG << "No input file specified, exiting" << endl;
    return 0;
  }
  const std::string& input_file=cmd_line.lastParsedArgs()[0];

  signal(SIGINT,sigIntHandler);

  // open input file
  source->open(input_file);

  // this is used to convert the scans into point clouds
  PointNormal2fUnprojectorPolarPtr unprojector = PointNormal2fUnprojectorPolarPtr(new PointNormal2fUnprojectorPolar());
  // these holds the previous and current scans
  PointNormal2fVectorCloud fixed, moving;

  using NormalComputator = NormalComputator1DSlidingWindow<PointNormal2fVectorCloud, 1>;
  using NormalComputatorPtr = std::shared_ptr<NormalComputator>;
  NormalComputatorPtr normal_computator_sliding = NormalComputatorPtr(new NormalComputator());

  // Correspondence Vectors
  CorrespondenceVector correspondences_nn, correspondences_kd_tree, correspondences_proj;
  corr_finder_nn->setCorrespondences(&correspondences_nn);
  corr_finder_kd_tree->setCorrespondences(&correspondences_kd_tree);
  corr_finder_proj->setCorrespondences(&correspondences_proj);
  PointNormal2fProjectorPolarPtr projector = PointNormal2fProjectorPolarPtr(new PointNormal2fProjectorPolar());
  
  //total number of frames
  int count=0;

  // to see the resources used
  SystemUsageCounter usage_counter;
  int num_frames_processed=0;

  BaseSensorMessagePtr msg;
  
  while (run && (msg=source->getMessage())){
    LaserMessagePtr laser=dynamic_pointer_cast<LaserMessage>(msg);
    if (laser) {
      Matrix_<float> range_matrix;
      range_matrix.resize(1, laser->ranges.size());
      for(size_t i = 0; i < laser->ranges.size(); ++i)
        range_matrix.at(0, i) = laser->ranges.value().at(i);

      if (! count) {
        initProjectors(unprojector,projector,laser);
        corr_finder_proj->param_projector.setValue(projector);
        fixed.clear();
        std::back_insert_iterator<PointNormal2fVectorCloud> fixed_back_insertor(fixed);
        int unprojected_valid = unprojector->compute<WithNormals>(fixed_back_insertor, range_matrix);
        TEST_LOG << "Valid Unprojected Points: " << unprojected_valid << endl;
        normal_computator_sliding->computeNormals(fixed);
      }
      if (skip.value() && count && !(count%skip.value())) {
        moving.clear();
        std::back_insert_iterator<PointNormal2fVectorCloud> moving_back_insertor(moving);
        int unprojected_valid = unprojector->compute<WithNormals>(moving_back_insertor, range_matrix);
        TEST_LOG << "Valid Unprojected Points: " << unprojected_valid << endl;
        normal_computator_sliding->computeNormals(moving);

        // Clear old correspondences
        correspondences_nn.clear();
        correspondences_kd_tree.clear();
        correspondences_proj.clear();
        TEST_LOG << "Correspondences Cleared"<< std::endl;
        
        //set Fixed and Moving to Correspondence Finders
        corr_finder_nn->setFixed(&fixed);
        corr_finder_kd_tree->setFixed(&fixed);
        corr_finder_proj->setFixed(&fixed);
        corr_finder_nn->setMoving(&moving);
        corr_finder_kd_tree->setMoving(&moving);
        corr_finder_proj->setMoving(&moving);
        TEST_LOG << "Scenes Set"<< std::endl;

        corr_finder_nn->compute();
        corr_finder_kd_tree->compute();
        corr_finder_proj->compute();

        TEST_LOG << "NN correspondences:      " << correspondences_nn.size() << std::endl;
        TEST_LOG << "KD-TREE correspondences: " << correspondences_kd_tree.size() << std::endl;
        TEST_LOG << "PROJ correspondences:    " << correspondences_proj.size() << std::endl;
        
        if (verbose.isSet()) {
            TEST_LOG << "frame: " << count
                     << " f: " << fixed.size()
                     << " m: " << moving.size() << endl;
        }
        fixed=moving;
        ++num_frames_processed;
        TEST_LOG << "...Press Enter to continue..." << endl;
        cin.get();
      }
      ++count;
    }
  }
  manager.write(config_file.value());

}
