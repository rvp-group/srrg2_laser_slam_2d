#include "fixtures.hpp"

using namespace srrg2_core;
using namespace srrg2_laser_slam_2d;
using namespace srrg2_slam_interfaces;

#ifdef CURRENT_SOURCE_DIR
const std::string current_source_directory = CURRENT_SOURCE_DIR;
#else
const std::string current_source_directory = "./";
#endif

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST_F(AIS, KillianCourtSLAM) {
  // ds load the first 100 message packs of the killian dataset
  std::cerr << "loading dataset\n";
  loadDataset("mit_killian_court.json", "mit_killian_court_gt.txt", 100);
  std::cerr << "dataset loaded\n";

  std::cerr << "check whether config file exists\n";
  if (!isAccessible(current_source_directory + "/../../configs/killian.conf")) {
    throw std::runtime_error("TEST|AIS::KillianCourtSLAM|ERROR, config does not exist");
  }
  std::cerr << "config file exists\n";

  // ds load a laser slam assembly from configuration
  ConfigurableManager manager;
  std::cerr << "manager reads config file\n";
  manager.read(current_source_directory + "/../../configs/killian.conf");
  std::cerr << "done!\n";
  MultiGraphSLAM2DPtr slammer = manager.getByName<MultiGraphSLAM2D>("slam");
  std::cerr << "got also the fucking guy here\n";
  ASSERT_NOTNULL(slammer);

  // ds process all messages and feed benchamin with computed estimates
  // ds TODO clearly this does not account for PGO/BA which happens retroactively
  std::cerr << "getting a message\n";
  BaseSensorMessagePtr message = nullptr;
  message                      = benchamin->getMessage();
  std::cerr << "got a message\n";
  if (!message) {
    throw std::runtime_error("CANCRO");
  } else {
    std::cerr << "SEQ [ " << message->seq.value() << " ]\n";
  }
  std::cerr << "start while loop\n";
  while ((message = benchamin->getMessage())) {
    std::cerr << "TEST|AIS::KillianCourtSLAM|message [ " << message->seq.value() << " ]\n";
    SystemUsageCounter::tic();
    std::cerr << "TEST|AIS::KillianCourtSLAM|setting message\n";
    slammer->setRawData(message);
    std::cerr << "TEST|AIS::KillianCourtSLAM|compute\n";
    slammer->compute();
    const double processing_duration_seconds = SystemUsageCounter::toc();
    std::cerr << "TEST|AIS::KillianCourtSLAM|setting pose estimate to benchamin\n";
    benchamin->setPoseEstimate(
      slammer->robotInWorld(), message->timestamp.value(), processing_duration_seconds);
  }

  // ds check graph
  ASSERT_NOTNULL(slammer->graph());
  ASSERT_EQ(slammer->graph()->variables().size(), 11);
  ASSERT_EQ(slammer->graph()->factors().size(), 10);

  // ds run ground truth error computation
  benchamin->compute();
}

TEST_F(AIS, KillianCourtSLAMLoopClosure) {
  // ds TODO need a tight circle with few messages (DEBUG speed)
  std::cerr << "NEED A MINI LOOP CLOSURE DATASET <3" << std::endl;
}
