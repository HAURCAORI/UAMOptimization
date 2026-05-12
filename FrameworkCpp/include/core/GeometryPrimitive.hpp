#pragma once

#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace hexaarch::core {

struct GeometryPrimitive {
    enum class Kind {
        sphere,
        box,
        cylinder,
        disk,
        segment
    };

    Kind kind = Kind::sphere;
    Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
    Eigen::Vector3d dimensions = Eigen::Vector3d::Zero();
    double padding = 0.0;

    [[nodiscard]] static GeometryPrimitive makeSphere(double radius, double padding = 0.0);
    [[nodiscard]] static GeometryPrimitive makeBox(const Eigen::Vector3d& extents, double padding = 0.0);
    [[nodiscard]] static GeometryPrimitive makeCylinder(double radius, double length, double padding = 0.0);
    [[nodiscard]] static GeometryPrimitive makeDisk(double radius, double padding = 0.0);
    [[nodiscard]] static GeometryPrimitive makeSegment(double length, double padding = 0.0);
};

inline GeometryPrimitive GeometryPrimitive::makeSphere(const double radius, const double padding) {
    GeometryPrimitive primitive;
    primitive.kind = Kind::sphere;
    primitive.dimensions = {radius, 0.0, 0.0};
    primitive.padding = padding;
    return primitive;
}

inline GeometryPrimitive GeometryPrimitive::makeBox(const Eigen::Vector3d& extents, const double padding) {
    GeometryPrimitive primitive;
    primitive.kind = Kind::box;
    primitive.dimensions = extents;
    primitive.padding = padding;
    return primitive;
}

inline GeometryPrimitive GeometryPrimitive::makeCylinder(const double radius, const double length, const double padding) {
    GeometryPrimitive primitive;
    primitive.kind = Kind::cylinder;
    primitive.dimensions = {radius, length, 0.0};
    primitive.padding = padding;
    return primitive;
}

inline GeometryPrimitive GeometryPrimitive::makeDisk(const double radius, const double padding) {
    GeometryPrimitive primitive;
    primitive.kind = Kind::disk;
    primitive.dimensions = {radius, 0.0, 0.0};
    primitive.padding = padding;
    return primitive;
}

inline GeometryPrimitive GeometryPrimitive::makeSegment(const double length, const double padding) {
    GeometryPrimitive primitive;
    primitive.kind = Kind::segment;
    primitive.dimensions = {length, 0.0, 0.0};
    primitive.padding = padding;
    return primitive;
}

using GeometryPrimitives = std::vector<GeometryPrimitive>;

}  // namespace hexaarch::core
