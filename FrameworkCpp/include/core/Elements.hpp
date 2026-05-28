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

class PayloadElement final : public BasicSpatialElement, public IPayloadMassContributor, public IEnvelopeProvider {
public:
    PayloadElement(std::string id, DesignParameter* payload_mass);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;

private:
    DesignParameter* payload_mass_;
};

// Outer fuselage hull geometry and attachment anchor root.
// Renamed from BodyElement; assembly ID remains "body" so all existing attachments are unchanged.
class BodyHullElement final : public BasicSpatialElement, public IEnvelopeProvider {
public:
    BodyHullElement(std::string id, DesignParameter* Lx, DesignParameter* Lyi, DesignParameter* Lyo);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;

private:
    DesignParameter* Lx_;
    DesignParameter* Lyi_;
    DesignParameter* Lyo_;
};

// Central structural hub/chassis — splits the structural frame concept from the outer hull.
// Zero-mass at Stage 1 (frame mass remains distributed to arms via ArmElement::frameMass()).
// Geometry: flat cylinder representing the arm-junction hub plate. No design parameters.
class BodyFrameElement final : public BasicSpatialElement {
public:
    explicit BodyFrameElement(std::string id);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
};

class BatteryElement final : public BasicSpatialElement, public IEnergyStorage, public IEnvelopeProvider {
public:
    // m_bat: battery pack mass design variable (contributes to total mass and energy capacity).
    // thrust_max / propeller_diameter: used for geometry scaling only, not for mass.
    BatteryElement(std::string id, DesignParameter* m_bat,
                   DesignParameter* thrust_max, DesignParameter* propeller_diameter);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] double batteryMass() const override;
    [[nodiscard]] LocalAABB localEnvelope() const override;

private:
    DesignParameter* m_bat_;
    DesignParameter* thrust_max_;
    DesignParameter* propeller_diameter_;
};

class CabinEnvelopeElement final : public BasicSpatialElement, public IEnvelopeProvider {
public:
    explicit CabinEnvelopeElement(std::string id);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;
};

// Zero-mass virtual element that defines the minimum occupant space inside the cabin.
// Smaller than CabinEnvelopeElement; used for cabin-fit and rotor keep-out constraints.
class OccupantEnvelopeElement final : public BasicSpatialElement, public IEnvelopeProvider {
public:
    explicit OccupantEnvelopeElement(std::string id);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;
};

// Zero-mass virtual element representing the swept disk volume of one rotor.
// One per motor; placed at the motor axis position. Used for rotor keep-out packaging checks.
class KeepOutZoneElement final : public BasicSpatialElement, public IEnvelopeProvider {
public:
    explicit KeepOutZoneElement(std::string id, DesignParameter* propeller_diameter);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;

private:
    DesignParameter* propeller_diameter_;
    double r_keepout_ = 0.0;  // cached: r_prop + kKeepOutRadialMargin
};

class ArmElement final : public BasicSpatialElement, public IStructuralBeam, public ILoadReceiver {
public:
    ArmElement(std::string id, int index, DesignParameter* Lx, DesignParameter* Lyi, DesignParameter* Lyo,
               DesignParameter* r_o, DesignParameter* t_wall);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] int index() const;
    [[nodiscard]] double structuralSpanContribution() const override;
    [[nodiscard]] bool contributesToFrameMass() const override;
    [[nodiscard]] double outerRadius() const override;
    [[nodiscard]] double innerRadius() const override;
    [[nodiscard]] double crossSectionArea() const override;
    [[nodiscard]] double secondMomentOfArea() const override;
    [[nodiscard]] double polarMomentOfArea() const override;
    void clearLoads() override;
    void addLoad(const AppliedLoad& load) override;
    [[nodiscard]] const std::vector<AppliedLoad>& loads() const override;

private:
    int index_;
    DesignParameter* Lx_;
    DesignParameter* Lyi_;
    DesignParameter* Lyo_;
    DesignParameter* r_o_;
    DesignParameter* t_wall_;
    double r_o_val_ = 0.08;
    double r_i_val_ = 0.075;
    double cross_section_area_ = 0.0;
    double second_moment_of_area_ = 0.0;
    std::vector<AppliedLoad> loads_;
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
    void registerThrustMax(DesignParameter* thrust_max);

private:
    int index_;
    double yaw_moment_sign_;
    DesignParameter* propeller_diameter_;
    DesignParameter* thrust_max_ = nullptr;
};

}  // namespace hexaarch::core
