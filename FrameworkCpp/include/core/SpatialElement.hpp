#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "core/ConstraintRegistry.hpp"
#include "core/GeometryPrimitive.hpp"
#include "core/ParameterRegistry.hpp"

namespace hexaarch::core {

struct AnchorFrame {
    std::string name;
    Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
};

class SpatialElement {
public:
    virtual ~SpatialElement() = default;

    [[nodiscard]] virtual std::string id() const = 0;
    [[nodiscard]] virtual std::string type() const = 0;
    [[nodiscard]] virtual std::unique_ptr<SpatialElement> clone() const = 0;

    virtual void registerParameters(ParameterRegistry& registry) = 0;
    virtual void rebindParameters(ParameterRegistry& registry) = 0;
    virtual void registerConstraints(ConstraintRegistry& registry) const = 0;

    [[nodiscard]] virtual double mass() const = 0;
    [[nodiscard]] virtual Eigen::Vector3d localCOM() const = 0;
    [[nodiscard]] virtual Eigen::Matrix3d localInertiaAtLocalCOM() const = 0;

    [[nodiscard]] virtual GeometryPrimitives localPrimitives() const = 0;
    [[nodiscard]] virtual Eigen::Isometry3d localPose() const = 0;
    [[nodiscard]] virtual std::vector<AnchorFrame> anchors() const = 0;
    [[nodiscard]] virtual std::optional<Eigen::Isometry3d> anchorPose(std::string_view anchor_name) const = 0;

    virtual void updateFromParameters() = 0;
};

using SpatialElementPtr = std::unique_ptr<SpatialElement>;

}  // namespace hexaarch::core
