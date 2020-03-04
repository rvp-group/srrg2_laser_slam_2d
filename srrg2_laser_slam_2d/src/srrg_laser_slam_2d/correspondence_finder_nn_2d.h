#pragma once
#include "correspondence_finder_2d.h"
#include "srrg_config/property_configurable.h"
#include <srrg_data_structures/path_matrix_distance_search.h>

namespace srrg2_laser_tracker_2d {
  using namespace srrg2_core;

  class CorrespondenceFinderNN2D : public CorrespondenceFinder2D {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = CorrespondenceFinder2D;
    using ThisType = CorrespondenceFinderNN2D;

    using FixedPointType  = FixedType::value_type;
    using FixedScalarType = FixedPointType::Scalar;

    using MovingPointType  = MovingType::value_type;
    using MovingScalarType = MovingPointType::Scalar;

    PARAM(PropertyFloat, max_distance_m, "max distance for correspondences [meters]", 1.f, 0);
    PARAM(PropertyFloat,
          resolution,
          "resolution of the distance map [m/pixel]",
          5e-2f,
          &_config_changed);
    PARAM(PropertyFloat, normal_cos, "min cosinus between normals", 0.8, 0);
    virtual void compute() override;

    void reset();

    CorrespondenceFinderNN2D();

  protected:
    inline Vector2f _pointInMapCoords(const Vector2f& point_) {
      Vector2f p = (point_ - _lowest_point) * _invResolution() + _paddingVector() * .5f;
      return p;
    }

    void _adjustSize();

    FixedPointType::VectorType _bounding_box; // [m]
    FixedPointType::VectorType _lowest_point; // [m]

    bool _config_changed   = true;
    bool _bind_config_done = false;
    FixedType _points_in_distance_map;
    PathMatrix _path_map;

  private:
    PathMatrixDistanceSearch _search;
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

} // namespace srrg2_laser_tracker_2d
