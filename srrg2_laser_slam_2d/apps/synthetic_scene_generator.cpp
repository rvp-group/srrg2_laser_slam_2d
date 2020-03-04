#include <srrg_boss/serializer.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_pcl/instances.h>
#include <srrg_system_utils/parse_command_line.h>

void createCorner(srrg2_core::PointNormal2fVectorCloud& points_,
                  const float& lenght_0,
                  const float& lenght_1,
                  const size_t& num_points_);

void createCircle(srrg2_core::PointNormal2fVectorCloud& points_,
                  const srrg2_core::Vector2f& center_,
                  const float& radius_,
                  const size_t& num_points_);

int main(int argc, char** argv) {
  using namespace srrg2_core;

  srrg2_core::point_cloud_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString arg_config_filename(&cmd, "c", "config", "config file path", "laser_config.conf");
  ArgumentString arg_message(&cmd,
                             "m",
                             "message",
                             "module you want to create, choose between {System, Tracker, Aligner}",
                             "laser_messages.boss");
  cmd.parse();

  // srrg create scene
  std::cerr << "creating scene...";
  PointNormal2fVectorCloud fixed;
  Vector2f circle_center(0, 0);
  const float circle_radius = 3.5;
  const int num_points      = 1024;

  PointNormal2fVectorCloud circle, corner;
  createCircle(circle, circle_center, circle_radius, num_points * 2);
  createCorner(corner, 2, 3, num_points);

  // srrg move the corner somewhere in the world
  //      0,0 is the origin of the straight corner
  Isometry2f corner_in_world(geometry2d::v2t(Vector3f(2, 0, M_PI * 0.25)));
  corner.transformInPlace<Isometry>(corner_in_world);

  // srrg concat the two pc in the scene
  fixed.insert(fixed.end(), circle.begin(), circle.end());
  fixed.insert(fixed.end(), corner.begin(), corner.end());

  std::cerr << "DONE" << std::endl;

  // srrg create the laser message for the scene
  std::cerr << "creating projector...";
  const int num_bins                   = 1024;
  const float laser_angle_min          = -M_PI * 0.4;
  const float laser_angle_max          = M_PI * 0.4;
  const float laser_angle_increment    = (laser_angle_max - laser_angle_min) / num_bins;
  const float laser_angular_resolution = 1 / laser_angle_increment;
  Matrix2f spherical_camera_matrix;
  spherical_camera_matrix << laser_angular_resolution, num_bins / 2, 0, 1;

  ConfigurableManager conf;
  auto projector = conf.create<PointNormal2fProjectorPolar>("PointNormal2fProjectorPolar");

  projector->param_canvas_rows.setValue(1);
  projector->param_canvas_cols.setValue(num_bins);
  projector->param_angle_col_max.setValue(laser_angle_max);
  projector->param_angle_col_min.setValue(laser_angle_min);
  projector->param_range_min.setValue(0.01);
  Isometry2f robot_pose          = Isometry2f::Identity();
  const Isometry2f proj_in_robot = geometry2d::v2t(Vector3f(.2f, 0.2f, 0.1f));
  Isometry2f projector_pose      = robot_pose * proj_in_robot;
  projector->setCameraPose(projector_pose);
  projector->initCameraMatrix();
  std::cerr << "DONE" << std::endl;

  std::cerr << "projecting scene...";
  PointNormal2fProjectorPolar::TargetMatrixType projected_matrix;
  projected_matrix.resize(1, num_bins);

  PointNormal2fVectorCloud points_in_camera;
  projector->compute(projected_matrix, fixed.begin(), fixed.end());

  std::vector<float> range_measurements;
  std::vector<float> intensity_measurements;

  range_measurements.reserve(num_bins);
  intensity_measurements.reserve(num_bins);

  for (size_t c = 0; c < num_bins; ++c) {
    range_measurements.emplace_back(projected_matrix.at(0, c).depth);
    intensity_measurements.emplace_back(.0f);
  }
  std::cerr << "DONE" << std::endl;

  std::cerr << "creating first message...";
  std::string scan_topic     = "/scan";
  std::string world_frame_id = "world";
  std::string scan_frame_id  = "scan";
  std::string base_frame_id  = "base_frame";
  std::string odom_frame_id  = "odom";

  SerializationContext* context = new SerializationContext();
  std::shared_ptr<Serializer> ser(new Serializer(context));
  ser->serializationContext()->setOutputFilePath(arg_message.value());

  size_t seq       = 0;
  double timestamp = 0.0;
  double timestep  = 0.001;

  // create scene message
  PointCloud2MessagePtr pc_msg(
    new PointCloud2Message("/point_cloud", world_frame_id, seq++, timestamp));
  timestamp += timestep;
  Point3fVectorCloud pc3d;
  pc3d.reserve(fixed.size());
  for (const auto& f : fixed) {
    Point3f p;
    p.coordinates().setZero();
    p.coordinates().head<2>() = f.coordinates();
    pc3d.emplace_back(p);
  }
  pc_msg->setPointCloud(pc3d);
  ser->writeObject(*pc_msg);

  // tf
  TransformEventsMessagePtr tf_msg(new TransformEventsMessage("/tf", "", seq++, timestamp));
  timestamp += timestep;
  TransformEvent te(
    timestamp, scan_frame_id, geometry3d::get3dFrom2dPose(proj_in_robot), base_frame_id);
  tf_msg->events.resize(1);
  tf_msg->events.setValue(0, te);
  ser->writeObject(*tf_msg);

  // state of the system at time 0
  // at each step we need
  // need the robot's pose, the tf tree, the laser msg
  LaserMessagePtr laser_msg(new LaserMessage(scan_topic, scan_frame_id, seq++, timestamp));
  timestamp += timestep;
  laser_msg->angle_min.setValue(laser_angle_min);
  laser_msg->angle_max.setValue(laser_angle_max);
  laser_msg->angle_increment.setValue(laser_angle_increment);
  laser_msg->time_increment.setValue(timestep);
  laser_msg->scan_time.setValue(getTime());
  laser_msg->range_min.setValue(projector->param_range_min.value());
  laser_msg->range_max.setValue(projector->param_range_max.value());
  laser_msg->ranges.setValue(range_measurements);
  laser_msg->intensities.setValue(intensity_measurements);

  ser->writeObject(*laser_msg);

  // robot's pose
  OdometryMessagePtr odom_msg(new OdometryMessage("/odom", odom_frame_id, seq++, timestamp));
  timestamp += timestep;
  odom_msg->child_frame.setValue(base_frame_id);
  odom_msg->pose.setValue(geometry3d::get3dFrom2dPose(robot_pose));
  ser->writeObject(*odom_msg);

  std::cerr << "DONE" << std::endl;

  const float inv_rand_max = 1.0f / (float) RAND_MAX;
  const float max_variance = 0.1f;
  // srrg set fixed seed
  std::cerr << "inv_rand_max: " << inv_rand_max << std::endl;
  srand(0);
  // srrg move the robot and compute new projection
  for (size_t i = 0; i < 100; ++i) {
    std::cerr << "compute moving...";
    Vector3f robot_motion_v = Vector3f::Zero();
    robot_motion_v.x()      = (rand() * inv_rand_max) * max_variance - max_variance * .5f;
    robot_motion_v.y()      = (rand() * inv_rand_max) * max_variance - max_variance * .5f;
    robot_motion_v.z()      = (rand() * inv_rand_max) * max_variance - max_variance * .5f;
    std::cerr << "robot motion: " << robot_motion_v.transpose() << std::endl;
    Isometry2f robot_motion = geometry2d::v2t(robot_motion_v);
    robot_pose              = robot_pose * robot_motion;
    projector_pose          = robot_pose * proj_in_robot;
    projector->setCameraPose(projector_pose);
    projected_matrix.clear();
    projected_matrix.resize(1, num_bins);

    projector->compute(projected_matrix, fixed.begin(), fixed.end());

    range_measurements.clear();
    intensity_measurements.clear();
    range_measurements.reserve(num_bins);
    intensity_measurements.reserve(num_bins);

    for (size_t c = 0; c < num_bins; ++c) {
      range_measurements.emplace_back(projected_matrix.at(0, c).depth);
      intensity_measurements.emplace_back(.0f);
    }
    std::cerr << "DONE" << std::endl;

    // tf
    tf_msg.reset(new TransformEventsMessage("/tf", "", seq++, timestamp));
    timestamp += timestep;
    TransformEvent te2(
      timestamp, scan_frame_id, geometry3d::get3dFrom2dPose(proj_in_robot), base_frame_id);
    tf_msg->events.resize(1);
    tf_msg->events.setValue(0, te2);
    ser->writeObject(*tf_msg);

    std::cerr << "creating second message...";

    laser_msg.reset(new LaserMessage(scan_topic, scan_frame_id, seq++, timestamp));
    timestamp += timestep;
    laser_msg->angle_min.setValue(laser_angle_min);
    laser_msg->angle_max.setValue(laser_angle_max);
    laser_msg->angle_increment.setValue(laser_angle_increment);
    laser_msg->time_increment.setValue(timestep);
    laser_msg->scan_time.setValue(getTime());
    laser_msg->range_min.setValue(projector->param_range_min.value());
    laser_msg->range_max.setValue(projector->param_range_max.value());
    laser_msg->ranges.setValue(range_measurements);
    laser_msg->intensities.setValue(intensity_measurements);

    ser->writeObject(*laser_msg);

    // robot's pose
    odom_msg.reset(new OdometryMessage("/odom", odom_frame_id, seq++, timestamp));
    timestamp += timestep;
    odom_msg->child_frame.setValue(base_frame_id);
    odom_msg->pose.setValue(geometry3d::get3dFrom2dPose(robot_pose));
    ser->writeObject(*odom_msg);

    std::cerr << "DONE" << std::endl;
  }

  conf.write(arg_config_filename.value());

  delete context;
}

void createCorner(srrg2_core::PointNormal2fVectorCloud& points_,
                  const float& lenght_0,
                  const float& lenght_1,
                  const size_t& num_points_) {
  points_.resize(num_points_);
  float step         = (lenght_0 + lenght_1) / num_points_;
  size_t n_points_l0 = lenght_0 / step;
  size_t n_points_l1 = num_points_ - n_points_l0;
  size_t k           = 0;
  for (size_t i = 0; i < n_points_l0; ++i) {
    srrg2_core::Vector2f t(step * i, 0);
    srrg2_core::PointNormal2f point;
    point.coordinates() = t;
    point.normal().setZero();
    points_[k++] = point;
  }

  for (size_t i = 1; i < n_points_l1; ++i) {
    srrg2_core::Vector2f t(0, -step * i);
    srrg2_core::PointNormal2f point;
    point.coordinates() = t;
    point.normal().setZero();
    points_[k++] = point;
  }
  points_.resize(k);
}

void createCircle(srrg2_core::PointNormal2fVectorCloud& points_,
                  const srrg2_core::Vector2f& center_,
                  const float& radius_,
                  const size_t& num_points_) {
  points_.resize(num_points_);
  for (size_t i = 0; i < num_points_; ++i) {
    const float angle = i * (2 * M_PI / num_points_);

    srrg2_core::Vector2f t = srrg2_core::Vector2f(radius_ * cosf(angle) + center_.x(),
                                                  radius_ * sinf(angle) + center_.y());
    srrg2_core::PointNormal2f point;
    point.coordinates() = t;
    point.normal().setZero();
    points_[i] = point;
  }
}
