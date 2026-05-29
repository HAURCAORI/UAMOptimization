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
    // bat_x/y/z: 3-DOF placement from body center; element encodes them in local_pose_.
    // Geometry is a flat slab sized from m_bat alone: volume = m_bat / pack_density,
    // with fixed plan area 1.30 × 1.50 m and height varying with capacity.
    BatteryElement(std::string id,
                   DesignParameter* m_bat,
                   DesignParameter* bat_x,
                   DesignParameter* bat_y,
                   DesignParameter* bat_z);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] double batteryMass() const override;
    [[nodiscard]] LocalAABB localEnvelope() const override;

private:
    DesignParameter* m_bat_;
    DesignParameter* bat_x_;
    DesignParameter* bat_y_;
    DesignParameter* bat_z_;
    double half_z_ = 0.0;  // cached physical half-height from pack volume, updated each call
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

class PassengerElement final : public BasicSpatialElement,
                               public IPayloadMassContributor,
                               public IEnvelopeProvider {
public:
    // Represents the 4-passenger group (2×2 seating).
    // pax_x/y/z: 3-DOF placement of the group center relative to body center.
    // Primitives are 4 seat boxes arranged symmetrically in element-local frame.
    PassengerElement(std::string id,
                     DesignParameter* mass,
                     DesignParameter* pax_x,
                     DesignParameter* pax_y,
                     DesignParameter* pax_z);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;
private:
    DesignParameter* mass_;
    DesignParameter* pax_x_;
    DesignParameter* pax_y_;
    DesignParameter* pax_z_;
};

class CargoElement final : public BasicSpatialElement,
                            public IPayloadMassContributor,
                            public IEnvelopeProvider {
public:
    // cargo_x/y/z: 3-DOF placement relative to body center.
    CargoElement(std::string id,
                 DesignParameter* mass,
                 DesignParameter* cargo_x,
                 DesignParameter* cargo_y,
                 DesignParameter* cargo_z);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;
private:
    DesignParameter* mass_;
    DesignParameter* cargo_x_;
    DesignParameter* cargo_y_;
    DesignParameter* cargo_z_;
};

class InstrumentPanelElement final : public BasicSpatialElement,
                                      public IPayloadMassContributor,
                                      public IEnvelopeProvider {
public:
    InstrumentPanelElement(std::string id, DesignParameter* mass);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;
private:
    DesignParameter* mass_;
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
