#include "fixtures.hpp"

using namespace srrg2_core;
using namespace srrg2_laser_slam_2d;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST_F(Synthetic, MeasurementAdaptor) {
  // ds allocate an adaptor with default parameters
  RawDataPreprocessorProjective2D adaptor;
  ASSERT_NOTNULL(adaptor.param_normal_computator_sliding.value());
  ASSERT_NOTNULL(adaptor.param_unprojector.value());

  // ds configure adaptor for fixture
  adaptor.param_range_max.setValue(range_max);
  adaptor.param_range_min.setValue(range_min);
  adaptor.param_scan_topic.setValue(scan_topic);
  adaptor.param_voxelize_resolution.setValue(voxelize_resolution);
  adaptor.param_unprojector->param_angle_max.setValue(angle_max_radians);
  adaptor.param_unprojector->param_angle_min.setValue(angle_min_radians);
  adaptor.param_unprojector->param_canvas_cols.setValue(number_of_measurements);
  adaptor.param_unprojector->param_canvas_rows.setValue(1);
  adaptor.param_unprojector->param_num_ranges.setValue(number_of_measurements);
  adaptor.param_unprojector->param_range_max.setValue(range_max);
  adaptor.param_unprojector->param_range_min.setValue(range_min);

  // ds process measurement
  PointNormal2fVectorCloud points;
  adaptor.setMeas(&points);
  adaptor.setRawData(std::dynamic_pointer_cast<BaseSensorMessage>(laser_message_circle));
  adaptor.compute();

  // ds check computed points
  ASSERT_EQ(points.size(), 100);

  // ds TODO validate computed polar positions
}

TEST_F(AIS, MeasurementAdaptorKillianCourt) {
  // ds load first message from killian dataset
  loadDataset("mit_killian_court.json", "mit_killian_court_gt.txt", 1);

  // ds allocate an adaptor with default parameters
  RawDataPreprocessorProjective2D adaptor;
  ASSERT_NOTNULL(adaptor.param_normal_computator_sliding.value());
  ASSERT_NOTNULL(adaptor.param_unprojector.value());

  // ds configure adaptor for fixture
  adaptor.param_range_max.setValue(laser_message_0->range_max.value());
  adaptor.param_range_min.setValue(laser_message_0->range_min.value());
  adaptor.param_scan_topic.setValue(laser_message_0->topic.value());
  adaptor.param_voxelize_resolution.setValue(voxelize_resolution);
  adaptor.param_unprojector->param_angle_max.setValue(laser_message_0->angle_max.value());
  adaptor.param_unprojector->param_angle_min.setValue(laser_message_0->angle_min.value());
  adaptor.param_unprojector->param_canvas_cols.setValue(laser_message_0->ranges.size());
  adaptor.param_unprojector->param_canvas_rows.setValue(1);
  adaptor.param_unprojector->param_num_ranges.setValue(laser_message_0->ranges.size());
  adaptor.param_unprojector->param_range_max.setValue(laser_message_0->range_max.value());
  adaptor.param_unprojector->param_range_min.setValue(laser_message_0->range_min.value());

  // ds process measurement
  PointNormal2fVectorCloud points;
  adaptor.setMeas(&points);
  adaptor.setRawData(benchamin->getMessage());
  adaptor.compute();

  // ds check computed points
  ASSERT_EQ(points.size(), 130);

  // ds TODO validate computed polar positions
}

TEST_F(AIS, MeasurementAdaptorIntel) {
  // ds load first message from intel dataset
  loadDataset("intel_research_lab.json", "intel_research_lab_gt.txt", 1);

  // ds allocate an adaptor with default parameters
  RawDataPreprocessorProjective2D adaptor;
  ASSERT_NOTNULL(adaptor.param_normal_computator_sliding.value());
  ASSERT_NOTNULL(adaptor.param_unprojector.value());

  // ds configure adaptor for fixture
  adaptor.param_range_max.setValue(laser_message_0->range_max.value());
  adaptor.param_range_min.setValue(laser_message_0->range_min.value());
  adaptor.param_scan_topic.setValue(laser_message_0->topic.value());
  adaptor.param_voxelize_resolution.setValue(voxelize_resolution);
  adaptor.param_unprojector->param_angle_max.setValue(laser_message_0->angle_max.value());
  adaptor.param_unprojector->param_angle_min.setValue(laser_message_0->angle_min.value());
  adaptor.param_unprojector->param_canvas_cols.setValue(laser_message_0->ranges.size());
  adaptor.param_unprojector->param_canvas_rows.setValue(1);
  adaptor.param_unprojector->param_num_ranges.setValue(laser_message_0->ranges.size());
  adaptor.param_unprojector->param_range_max.setValue(laser_message_0->range_max.value());
  adaptor.param_unprojector->param_range_min.setValue(laser_message_0->range_min.value());

  // ds process measurement
  PointNormal2fVectorCloud points;
  adaptor.setMeas(&points);
  adaptor.setRawData(benchamin->getMessage());
  adaptor.compute();

  // ds check computed points
  ASSERT_EQ(points.size(), 148);

  // ds TODO validate computed polar positions
}
