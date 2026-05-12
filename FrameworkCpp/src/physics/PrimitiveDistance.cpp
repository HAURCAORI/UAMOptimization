#include "physics/PrimitiveDistance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hexaarch::physics {
namespace {

Eigen::Isometry3d worldPrimitivePose(const core::GeometryPrimitive& primitive, const Eigen::Isometry3d& pose) {
    return pose * primitive.local_pose;
}

double pointSegmentDistance(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& segment_start,
    const Eigen::Vector3d& segment_end) {
    const Eigen::Vector3d direction = segment_end - segment_start;
    const double length_squared = direction.squaredNorm();
    if (length_squared <= 1e-12) {
        return (point - segment_start).norm();
    }

    const double projection = std::clamp(
        (point - segment_start).dot(direction) / length_squared,
        0.0,
        1.0);
    const Eigen::Vector3d closest = segment_start + projection * direction;
    return (point - closest).norm();
}

double pointBoxDistance(
    const Eigen::Vector3d& point_world,
    const core::GeometryPrimitive& box,
    const Eigen::Isometry3d& box_pose) {
    const Eigen::Vector3d point_local = box_pose.inverse() * point_world;
    const Eigen::Vector3d delta = (point_local.cwiseAbs() - box.dimensions).cwiseMax(0.0);
    return delta.norm();
}

double segmentSegmentDistance(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2) {
    constexpr double epsilon = 1e-12;
    const Eigen::Vector3d d1 = q1 - p1;
    const Eigen::Vector3d d2 = q2 - p2;
    const Eigen::Vector3d r = p1 - p2;
    const double a = d1.dot(d1);
    const double e = d2.dot(d2);
    const double f = d2.dot(r);

    double s = 0.0;
    double t = 0.0;

    if (a <= epsilon && e <= epsilon) {
        return (p1 - p2).norm();
    }
    if (a <= epsilon) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = d1.dot(r);
        if (e <= epsilon) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = d1.dot(d2);
            const double denom = a * e - b * b;
            if (std::abs(denom) > epsilon) {
                s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
            }
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }

    const Eigen::Vector3d closest1 = p1 + d1 * s;
    const Eigen::Vector3d closest2 = p2 + d2 * t;
    return (closest1 - closest2).norm();
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> segmentEndpoints(
    const core::GeometryPrimitive& primitive,
    const Eigen::Isometry3d& pose) {
    const Eigen::Isometry3d primitive_pose = worldPrimitivePose(primitive, pose);
    const double half_length = 0.5 * primitive.dimensions.x();
    const Eigen::Vector3d start = primitive_pose * Eigen::Vector3d(-half_length, 0.0, 0.0);
    const Eigen::Vector3d end = primitive_pose * Eigen::Vector3d(half_length, 0.0, 0.0);
    return {start, end};
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> cylinderAxisEndpoints(
    const core::GeometryPrimitive& primitive,
    const Eigen::Isometry3d& pose) {
    const Eigen::Isometry3d primitive_pose = worldPrimitivePose(primitive, pose);
    const double half_length = 0.5 * primitive.dimensions.y();
    const Eigen::Vector3d start = primitive_pose * Eigen::Vector3d(0.0, -half_length, 0.0);
    const Eigen::Vector3d end = primitive_pose * Eigen::Vector3d(0.0, half_length, 0.0);
    return {start, end};
}

double primitiveRadius(const core::GeometryPrimitive& primitive) {
    using Kind = core::GeometryPrimitive::Kind;
    switch (primitive.kind) {
    case Kind::sphere:
    case Kind::disk:
    case Kind::cylinder:
        return primitive.dimensions.x();
    default:
        return 0.0;
    }
}

double boxBoxClearance(
    const core::GeometryPrimitive& lhs,
    const Eigen::Isometry3d& lhs_pose,
    const core::GeometryPrimitive& rhs,
    const Eigen::Isometry3d& rhs_pose) {
    std::vector<Eigen::Vector3d> axes;
    axes.reserve(15);

    const Eigen::Matrix3d lhs_rotation = lhs_pose.rotation();
    const Eigen::Matrix3d rhs_rotation = rhs_pose.rotation();
    for (int index = 0; index < 3; ++index) {
        axes.push_back(lhs_rotation.col(index).normalized());
        axes.push_back(rhs_rotation.col(index).normalized());
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const Eigen::Vector3d axis = lhs_rotation.col(i).cross(rhs_rotation.col(j));
            if (axis.squaredNorm() > 1e-12) {
                axes.push_back(axis.normalized());
            }
        }
    }

    const Eigen::Vector3d lhs_center = lhs_pose.translation();
    const Eigen::Vector3d rhs_center = rhs_pose.translation();
    const Eigen::Vector3d delta = rhs_center - lhs_center;

    double max_separation = -std::numeric_limits<double>::infinity();
    double min_penetration = std::numeric_limits<double>::infinity();
    for (const auto& axis : axes) {
        const double lhs_projection =
            std::abs(lhs_rotation.col(0).dot(axis)) * lhs.dimensions.x() +
            std::abs(lhs_rotation.col(1).dot(axis)) * lhs.dimensions.y() +
            std::abs(lhs_rotation.col(2).dot(axis)) * lhs.dimensions.z();
        const double rhs_projection =
            std::abs(rhs_rotation.col(0).dot(axis)) * rhs.dimensions.x() +
            std::abs(rhs_rotation.col(1).dot(axis)) * rhs.dimensions.y() +
            std::abs(rhs_rotation.col(2).dot(axis)) * rhs.dimensions.z();
        const double separation = std::abs(delta.dot(axis)) - lhs_projection - rhs_projection;
        max_separation = std::max(max_separation, separation);
        min_penetration = std::min(min_penetration, -separation);
    }

    const double padding = lhs.padding + rhs.padding;
    if (max_separation > 0.0) {
        return max_separation - padding;
    }
    return -min_penetration - padding;
}

double fallbackClearance(
    const core::GeometryPrimitive& lhs,
    const Eigen::Isometry3d& lhs_pose,
    const core::GeometryPrimitive& rhs,
    const Eigen::Isometry3d& rhs_pose) {
    const Eigen::Vector3d lhs_center = worldPrimitivePose(lhs, lhs_pose).translation();
    const Eigen::Vector3d rhs_center = worldPrimitivePose(rhs, rhs_pose).translation();
    return (lhs_center - rhs_center).norm() - boundingRadius(lhs) - boundingRadius(rhs);
}

}  // namespace

double boundingRadius(const core::GeometryPrimitive& primitive) {
    using Kind = core::GeometryPrimitive::Kind;

    switch (primitive.kind) {
    case Kind::sphere:
    case Kind::disk:
        return primitive.dimensions.x() + primitive.padding;
    case Kind::cylinder:
        return std::sqrt(primitive.dimensions.x() * primitive.dimensions.x() + 0.25 * primitive.dimensions.y() * primitive.dimensions.y()) + primitive.padding;
    case Kind::box:
        return primitive.dimensions.norm() + primitive.padding;
    case Kind::segment:
        return 0.5 * primitive.dimensions.x() + primitive.padding;
    default:
        return primitive.padding;
    }
}

double primitiveClearance(
    const core::GeometryPrimitive& lhs,
    const Eigen::Isometry3d& lhs_pose,
    const core::GeometryPrimitive& rhs,
    const Eigen::Isometry3d& rhs_pose) {
    using Kind = core::GeometryPrimitive::Kind;

    const Eigen::Isometry3d lhs_world = worldPrimitivePose(lhs, lhs_pose);
    const Eigen::Isometry3d rhs_world = worldPrimitivePose(rhs, rhs_pose);
    const Eigen::Vector3d lhs_center = lhs_world.translation();
    const Eigen::Vector3d rhs_center = rhs_world.translation();

    if (lhs.kind == Kind::sphere && rhs.kind == Kind::sphere) {
        return (lhs_center - rhs_center).norm() - primitiveRadius(lhs) - primitiveRadius(rhs) - lhs.padding - rhs.padding;
    }

    if (lhs.kind == Kind::sphere && rhs.kind == Kind::box) {
        return pointBoxDistance(lhs_center, rhs, rhs_world) - primitiveRadius(lhs) - lhs.padding - rhs.padding;
    }
    if (lhs.kind == Kind::box && rhs.kind == Kind::sphere) {
        return pointBoxDistance(rhs_center, lhs, lhs_world) - primitiveRadius(rhs) - lhs.padding - rhs.padding;
    }

    if (lhs.kind == Kind::segment && (rhs.kind == Kind::disk || rhs.kind == Kind::sphere || rhs.kind == Kind::cylinder)) {
        const auto [start, end] = segmentEndpoints(lhs, lhs_pose);
        return pointSegmentDistance(rhs_center, start, end) - primitiveRadius(rhs) - lhs.padding - rhs.padding;
    }
    if (rhs.kind == Kind::segment && (lhs.kind == Kind::disk || lhs.kind == Kind::sphere || lhs.kind == Kind::cylinder)) {
        const auto [start, end] = segmentEndpoints(rhs, rhs_pose);
        return pointSegmentDistance(lhs_center, start, end) - primitiveRadius(lhs) - lhs.padding - rhs.padding;
    }

    if (lhs.kind == Kind::segment && rhs.kind == Kind::segment) {
        const auto [lhs_start, lhs_end] = segmentEndpoints(lhs, lhs_pose);
        const auto [rhs_start, rhs_end] = segmentEndpoints(rhs, rhs_pose);
        return segmentSegmentDistance(lhs_start, lhs_end, rhs_start, rhs_end) - lhs.padding - rhs.padding;
    }

    if (lhs.kind == Kind::box && rhs.kind == Kind::box) {
        return boxBoxClearance(lhs, lhs_world, rhs, rhs_world);
    }

    if ((lhs.kind == Kind::cylinder || lhs.kind == Kind::disk) && (rhs.kind == Kind::cylinder || rhs.kind == Kind::disk)) {
        if (lhs.kind == Kind::disk && rhs.kind == Kind::disk) {
            return (lhs_center - rhs_center).norm() - primitiveRadius(lhs) - primitiveRadius(rhs) - lhs.padding - rhs.padding;
        }

        if (lhs.kind == Kind::cylinder && rhs.kind == Kind::disk) {
            const auto [start, end] = cylinderAxisEndpoints(lhs, lhs_pose);
            return pointSegmentDistance(rhs_center, start, end) - primitiveRadius(lhs) - primitiveRadius(rhs) - lhs.padding - rhs.padding;
        }
        if (lhs.kind == Kind::disk && rhs.kind == Kind::cylinder) {
            const auto [start, end] = cylinderAxisEndpoints(rhs, rhs_pose);
            return pointSegmentDistance(lhs_center, start, end) - primitiveRadius(lhs) - primitiveRadius(rhs) - lhs.padding - rhs.padding;
        }

        const auto [lhs_start, lhs_end] = cylinderAxisEndpoints(lhs, lhs_pose);
        const auto [rhs_start, rhs_end] = cylinderAxisEndpoints(rhs, rhs_pose);
        return segmentSegmentDistance(lhs_start, lhs_end, rhs_start, rhs_end) - primitiveRadius(lhs) - primitiveRadius(rhs) - lhs.padding - rhs.padding;
    }

    return fallbackClearance(lhs, lhs_pose, rhs, rhs_pose);
}

}  // namespace hexaarch::physics
