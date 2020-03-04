#include "correspondence_finder_projective_2d.h"
#include <iostream>
namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;
  using namespace std;

  void CorrespondenceFinderProjective2D::printInfo() const {
    const std::string INFO = "CorrespondenceFinderProjective2D::printInfo| ";
    std::cerr << INFO << "Projector Info" << std::endl;
    std::cerr << INFO << "point_distance: " << param_point_distance.value() << std::endl;
    std::cerr << INFO << "normal_cos:     " << param_normal_cos.value() << std::endl;
    std::cerr << INFO << "projector:      " << param_projector.value() << std::endl;
    if (param_projector.value()) {
      std::cerr << INFO << "\t projector sensor matrix:\n"
                << param_projector->cameraMatrix() << std::endl;
    }
  }

  void CorrespondenceFinderProjective2D::compute() {
    // bdc if not loaded from the config, instantiate a new projector w/ default sensor matrix
    PointNormal2fProjectorPolarPtr projector = this->param_projector.value();
    if (!projector) {
      throw std::runtime_error("CorrespondenceFinderProjective2D::compute| Missing Projector");
    }

    if (!_fixed) {
      throw std::runtime_error("CorrespondenceFinderProjective2D::compute| Missing fixed!");
    }

    if (!_moving) {
      throw std::runtime_error("CorrespondenceFinderProjective2D::compute| Missing moving!");
    }

    //    printInfo();

    int num_beams = projector->param_canvas_cols.value();

    if (this->_fixed_changed_flag || this->_projector_changed_flag) {
      _fixed_matrix.resize(1, num_beams);
      _moving_matrix.resize(1, num_beams);
      projector->setCameraPose(Isometry2f::Identity());
      projector->compute(_fixed_matrix, _fixed->begin(), _fixed->end());
      _fixed_changed_flag     = false;
      _projector_changed_flag = false;
    }

    //std::cerr << "camera_pose: " << geometry2d::t2v(_estimate.inverse()).transpose() << std::endl;
    // the camera sits on the fixed
    projector->setCameraPose(_estimate.inverse());
    projector->compute(_moving_matrix, _moving->begin(), _moving->end());
    _correspondences->resize(num_beams);

    int k          = 0;
    auto moving_it = _moving_matrix.begin();
    auto fixed_it  = _fixed_matrix.begin();

    while (moving_it != _moving_matrix.end() && fixed_it != _fixed_matrix.end()) {
      const auto& mcell = *moving_it;
      const auto& fcell = *fixed_it;
      ++moving_it;
      ++fixed_it;

      if (mcell.source_idx < 0 || fcell.source_idx < 0) {
        continue;
      }

      if (fabs(fcell.depth - mcell.depth) > param_point_distance.value()) {
        continue;
      }

      if (mcell.transformed.normal().dot(fcell.transformed.normal()) < param_normal_cos.value()) {
        continue;
      }
      //std::cerr << fabs(fcell.depth - mcell.depth) <<  " ";
      _correspondences->at(k++) = (Correspondence(fcell.source_idx, mcell.source_idx));
    }
    //std::cerr << std::endl;
    _correspondences->resize(k);
  }

} // namespace srrg2_laser_tracker_2d
