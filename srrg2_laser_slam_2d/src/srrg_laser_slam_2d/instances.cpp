#include "instances.h"
#include "srrg_data_structures/path_matrix_distance_search.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/utils/factor_graph_utils/instances.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"

#include "srrg_slam_interfaces/aligner_termination_criteria_standard_impl.cpp"
#include "srrg_slam_interfaces/instances.h"
#include "srrg_slam_interfaces/multi_aligner_slice_impl.cpp"
#include "srrg_slam_interfaces/multi_tracker_slice_impl.cpp"

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_solver;

  inline void MultiAlignerLaser2DWithSensorSlice::setupFactor() {
    BaseType::setupFactor();
    BaseType::setupFactorWithSensor("MultiAlignerLaser2DWithSensorSlice");
  }

  namespace dummy_instances {
    static MultiAlignerLaser2DSlice dummy_aligner_slice;
    static MultiTrackerLaser2DSlice dummy_tracker_slice;
    static MultiAlignerLaser2DWithSensorSlice dummy_tracker_slice_in_sensor;
  } // namespace dummy_instances

  void laser_tracker_2d_registerTypes() {
    srrg2_solver::registerTypes2D();
    srrg2_solver::solver_registerTypes();
    srrg2_solver::linear_solver_registerTypes();
    srrg2_solver::solver_utils_registerTypes();
    srrg2_slam_interfaces::slam_interfaces_registerTypes();

    BOSS_REGISTER_CLASS(CorrespondenceFinderProjective2D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderNN2D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderKDTree2D);
    BOSS_REGISTER_CLASS(MergerProjective2D);
    BOSS_REGISTER_CLASS(MeasurementAdaptorProjective2D);
    BOSS_REGISTER_CLASS(SceneClipperProjective2D);
    BOSS_REGISTER_CLASS(SceneClipperBall2D);
    BOSS_REGISTER_CLASS(MultiAlignerLaser2DSlice);
    BOSS_REGISTER_CLASS(MultiAlignerLaser2DWithSensorSlice);
    BOSS_REGISTER_CLASS(MultiTrackerLaser2DSlice);
  }

} // namespace srrg2_laser_tracker_2d
