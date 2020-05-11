#pragma once
#include "correspondence_finder_normal_2f.h"
#include "srrg_config/property_configurable.h"
#include <srrg_data_structures/kd_tree.hpp>

namespace srrg2_laser_slam_2d {
  using namespace srrg2_core;

  class CorrespondenceFinderKDTree2D : public CorrespondenceFinderNormal2f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = CorrespondenceFinderNormal2f;
    using ThisType = CorrespondenceFinderKDTree2D;

    using FixedPointType  = FixedType::value_type;
    using FixedScalarType = FixedPointType::Scalar;

    using MovingPointType  = MovingType::value_type;
    using MovingScalarType = MovingPointType::Scalar;

    using KDTree2D =
      srrg2_core::KDTree<FixedScalarType, FixedPointType::VectorType::RowsAtCompileTime>;
    PARAM(PropertyFloat, max_distance_m, "max distance for correspondences [meters]", 1e-2f, 0);
    PARAM(PropertyFloat,
          max_leaf_range,
          "maximum range for a leaf of the KDTree [meters]",
          1e-2f,
          0);
    PARAM(PropertyUnsignedInt,
          min_leaf_points,
          "minimum number of points in a leaf of the KDTree",
          20,
          0);
    PARAM(PropertyFloat, normal_cos, "min cosinus between normals", 0.8, 0);

    CorrespondenceFinderKDTree2D() {
      _kd_tree = std::move(std::unique_ptr<KDTree2D>(new KDTree2D(
        KDTree2D::VectorTDVector(0), param_max_leaf_range.value(), param_min_leaf_points.value())));
    }

    virtual void compute() override;
    void reset();

  protected:
    std::unique_ptr<KDTree2D> _kd_tree;
  };

  using CorrespondenceFinderKDTree2DPtr = std::shared_ptr<CorrespondenceFinderKDTree2D>;

} // namespace srrg2_laser_slam_2d
