#pragma once

#include "core/GeometryPrimitive.hpp"

namespace hexaarch::physics {

[[nodiscard]] double boundingRadius(const core::GeometryPrimitive& primitive);
[[nodiscard]] double primitiveClearance(
    const core::GeometryPrimitive& lhs,
    const Eigen::Isometry3d& lhs_pose,
    const core::GeometryPrimitive& rhs,
    const Eigen::Isometry3d& rhs_pose);

}  // namespace hexaarch::physics
