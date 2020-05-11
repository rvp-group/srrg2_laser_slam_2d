#include "instances.h"
#include "registration/aligner_slice_processor_laser_2d_impl.cpp"
#include "srrg_data_structures/path_matrix_distance_search.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/utils/factor_graph_utils/instances.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include <srrg2_slam_interfaces/instances.h>
#include <srrg2_slam_interfaces/trackers/tracker_slice_processor_impl.cpp>

namespace srrg2_laser_slam_2d {
  using namespace srrg2_solver;

  namespace dummy_instances {
    static AlignerSliceProcessorLaser2D dummy_aligner_slice;
    static TrackerSliceProcessorLaser2D dummy_tracker_slice;
    static AlignerSliceProcessorLaser2DWithSensor dummy_tracker_slice_in_sensor;
  } // namespace dummy_instances

  void srrg2_laser_slam_2d_registerTypes() {
    srrg2_solver::variables_and_factors_2d_registerTypes();
    srrg2_solver::solver_registerTypes();
    srrg2_solver::linear_solver_registerTypes();
    srrg2_solver::solver_utils_registerTypes();
    srrg2_slam_interfaces::srrg2_slam_interfaces_registerTypes();

    BOSS_REGISTER_CLASS(CorrespondenceFinderProjective2f);
    BOSS_REGISTER_CLASS(CorrespondenceFinderNN2D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderKDTree2D);
    BOSS_REGISTER_CLASS(MergerProjective2D);
    BOSS_REGISTER_CLASS(RawDataPreprocessorProjective2D);
    BOSS_REGISTER_CLASS(SceneClipperProjective2D);
    BOSS_REGISTER_CLASS(SceneClipperBall2D);
    BOSS_REGISTER_CLASS(AlignerSliceProcessorLaser2D);
    BOSS_REGISTER_CLASS(AlignerSliceProcessorLaser2DWithSensor);
    BOSS_REGISTER_CLASS(TrackerSliceProcessorLaser2D);
  }

} // namespace srrg2_laser_slam_2d
