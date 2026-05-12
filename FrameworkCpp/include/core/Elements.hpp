#pragma once

#include <optional>
#include <string_view>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "core/ElementCapabilities.hpp"
#include "core/GeometryPrimitive.hpp"
#include "core/SpatialElement.hpp"

namespace hexaarch::core {

class BasicSpatialElement : public SpatialElement {
public:
    BasicSpatialElement(std::string id, std::string type);

    [[nodiscard]] std::string id() const override;
    [[nodiscard]] std::string type() const override;
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override = 0;
    void registerParameters(ParameterRegistry& registry) override = 0;
    void rebindParameters(ParameterRegistry& registry) override = 0;
    void registerConstraints(ConstraintRegistry& registry) const override = 0;
    [[nodiscard]] double mass() const override;
    [[nodiscard]] Eigen::Vector3d localCOM() const override;
    [[nodiscard]] Eigen::Matrix3d localInertiaAtLocalCOM() const override;
    [[nodiscard]] GeometryPrimitives localPrimitives() const override;
    [[nodiscard]] Eigen::Isometry3d localPose() const override;
    [[nodiscard]] std::vector<AnchorFrame> anchors() const override;
    [[nodiscard]] std::optional<Eigen::Isometry3d> anchorPose(std::string_view anchor_name) const override;

protected:
    void setAnchor(std::string name, const Eigen::Isometry3d& local_pose);

    std::string id_;
    std::string type_;
    double mass_ = 0.0;
    Eigen::Vector3d local_com_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d local_inertia_ = Eigen::Matrix3d::Zero();
    GeometryPrimitives primitives_;
    Eigen::Isometry3d local_pose_ = Eigen::Isometry3d::Identity();
    std::vector<AnchorFrame> anchors_;
};

class PayloadElement final : public BasicSpatialElement, public IPayloadMassContributor {
public:
    PayloadElement(std::string id, DesignParameter* payload_mass);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;

private:
    DesignParameter* payload_mass_;
};

class BodyElement final : public BasicSpatialElement {
public:
    BodyElement(std::string id, DesignParameter* Lx, DesignParameter* Lyi, DesignParameter* Lyo);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;

private:
    DesignParameter* Lx_;
    DesignParameter* Lyi_;
    DesignParameter* Lyo_;
};

class BatteryElement final : public BasicSpatialElement {
public:
    BatteryElement(std::string id, DesignParameter* thrust_max, DesignParameter* propeller_diameter);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;

private:
    DesignParameter* thrust_max_;
    DesignParameter* propeller_diameter_;
};

class ArmElement final : public BasicSpatialElement, public IStructuralMember {
public:
    ArmElement(std::string id, int index, DesignParameter* Lx, DesignParameter* Lyi, DesignParameter* Lyo, DesignParameter* Tmax);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] int index() const;
    [[nodiscard]] double structuralSpanContribution() const override;
    [[nodiscard]] bool contributesToFrameMass() const override;

private:
    int index_;
    DesignParameter* Lx_;
    DesignParameter* Lyi_;
    DesignParameter* Lyo_;
    DesignParameter* Tmax_;
};

class MotorElement final : public BasicSpatialElement, public IMotorMassContributor {
public:
    MotorElement(std::string id, int index, DesignParameter* thrust_max);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] int index() const;

private:
    int index_;
    DesignParameter* thrust_max_;
};

class RotorElement final : public BasicSpatialElement, public IPropulsionRotor {
public:
    RotorElement(std::string id, int index, double yaw_moment_sign, DesignParameter* propeller_diameter);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] int index() const;
    [[nodiscard]] int rotorIndex() const override;
    [[nodiscard]] double yawMomentSign() const override;

private:
    int index_;
    double yaw_moment_sign_;
    DesignParameter* propeller_diameter_;
};

}  // namespace hexaarch::core
