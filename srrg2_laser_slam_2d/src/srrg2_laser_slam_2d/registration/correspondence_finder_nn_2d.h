#pragma once
#include "correspondence_finder_normal_2f.h"
#include "srrg_config/property_configurable.h"
#include <srrg_data_structures/path_matrix_distance_search.h>

namespace srrg2_laser_slam_2d {

  class CorrespondenceFinderNN2D : public CorrespondenceFinderNormal2f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = CorrespondenceFinderNormal2f;
    using ThisType = CorrespondenceFinderNN2D;

    using FixedPointType  = FixedType::value_type;
    using FixedScalarType = FixedPointType::Scalar;

    using MovingPointType  = MovingType::value_type;
    using MovingScalarType = MovingPointType::Scalar;

    PARAM(srrg2_core::PropertyFloat,
          max_distance_m,
          "max distance for correspondences [meters]",
          1.f,
          0);
    PARAM(srrg2_core::PropertyFloat,
          resolution,
          "resolution of the distance map [m/pixel]",
          5e-2f,
          &_config_changed);
    PARAM(srrg2_core::PropertyFloat, normal_cos, "min cosinus between normals", 0.8, 0);
    virtual void compute() override;

    void reset();

    CorrespondenceFinderNN2D();

  protected:
    inline srrg2_core::Vector2f _pointInMapCoords(const srrg2_core::Vector2f& point_) {
      srrg2_core::Vector2f p = (point_ - _lowest_point) * _invResolution() + _paddingVector() * .5f;
      return p;
    }

    void _adjustSize();

    FixedPointType::VectorType _bounding_box; // [m]
    FixedPointType::VectorType _lowest_point; // [m]

    bool _config_changed   = true;
    bool _bind_config_done = false;
    FixedType _points_in_distance_map;
    srrg2_core::PathMatrix _path_map;

  private:
    srrg2_core::PathMatrixDistanceSearch _search;
    void _recomputeInternals();

    inline const float& _invResolution() {
      return _inv_resolution;
    }

    inline const FixedPointType::VectorType& _paddingVector() {
      return _padding_vector;
    }

    FixedPointType::VectorType _padding_vector;
    float _inv_resolution = 1.0f;
  };

  using CorrespondenceFinderNN2DPtr = std::shared_ptr<CorrespondenceFinderNN2D>;

} // namespace srrg2_laser_slam_2d
