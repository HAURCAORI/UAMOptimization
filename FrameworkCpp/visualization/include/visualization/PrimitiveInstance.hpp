#pragma once

#include <array>
#include <string>

#include "eigen3/Eigen/Geometry"

#include "core/GeometryPrimitive.hpp"

namespace hexaarch::visualization {

struct PrimitiveInstance {
    core::GeometryPrimitive::Kind primitive_type = core::GeometryPrimitive::Kind::sphere;
    Eigen::Isometry3d world_transform = Eigen::Isometry3d::Identity();
    Eigen::Vector3d dimensions = Eigen::Vector3d::Zero();
    double padding = 0.0;
    std::array<float, 4> color{0.8f, 0.8f, 0.8f, 1.0f};
    std::string source_element_id;
    std::string source_element_type;
    bool visible = true;
    bool wireframe = false;
};

}  // namespace hexaarch::visualization
