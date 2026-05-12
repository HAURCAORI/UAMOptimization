#pragma once

#include <string_view>
#include <vector>

#include "eigen3/Eigen/Geometry"

#include "core/GeometryPrimitive.hpp"

namespace hexaarch::core {

class SpatialElement;

struct AssembledElement {
    const SpatialElement* element = nullptr;
    Eigen::Isometry3d world_pose = Eigen::Isometry3d::Identity();
    GeometryPrimitives local_primitives;
};

struct AssemblyState {
    std::vector<AssembledElement> elements;

    [[nodiscard]] const AssembledElement* find(std::string_view element_id) const;
};

}  // namespace hexaarch::core
