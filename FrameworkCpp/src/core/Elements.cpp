#include "core/Elements.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "core/HexacopterArchitecture.hpp"

namespace hexaarch::core {
namespace {

constexpr double kBaselineMotorMass = 74.07;
constexpr double kBaselineMotorTmax = 7327.0;
constexpr double kMotorMassExponent = 3.0 / 3.5;
constexpr double kBaselinePayloadRadius = 0.60;

// BodyElement geometry
constexpr double kBodyHalfSizeBase   = 0.35;
constexpr double kBodyHalfSizeScale  = 0.10;
constexpr double kBodyHalfSizeOffset = 0.10;
constexpr double kBodyHalfHeight     = 0.20;
constexpr double kBodyBoxPadding     = 0.05;

// BatteryElement geometry
constexpr double kBatteryLengthBase  = 0.35;
constexpr double kBatteryLengthScale = 0.08;
constexpr double kBatteryWidthBase   = 0.25;
constexpr double kBatteryWidthScale  = 0.10;
constexpr double kBatteryHalfHeight  = 0.12;
constexpr double kBatteryBoxPadding  = 0.02;

// MotorElement geometry
constexpr double kMotorRadiusBase      = 0.08;
constexpr double kMotorRadiusScale     = 0.04;
constexpr double kMotorHeight          = 0.12;
constexpr double kMotorCylinderPadding = 0.01;

double frameMass(const double Lx, const double Lyi, const double Lyo) {
    constexpr double baseline_mass = 2240.73;
    constexpr double baseline_payload = 1500.0;
    const double arm_span = 2.0 * Lx + 2.0 * Lyi + 2.0 * Lyo;
    constexpr double arm_span_ref = 2.0 * 2.65 + 2.0 * 2.65 + 2.0 * 5.50;
    const double frame_mass_ref = baseline_mass - baseline_payload - 6.0 * kBaselineMotorMass;
    return frame_mass_ref * arm_span / arm_span_ref;
}

double motorMass(const double thrust_max) {
    return kBaselineMotorMass * std::pow(thrust_max / kBaselineMotorTmax, kMotorMassExponent);
}

double armLength(const int index, const double Lx, const double Lyi, const double Lyo) {
    return (index == 2 || index == 3)
        ? Lyo
        : std::sqrt(Lx * Lx + Lyi * Lyi);
}

const DesignParameter& requireParameter(const ParameterRegistry& registry, const std::string& stable_id) {
    const auto* parameter = registry.find(stable_id);
    if (parameter == nullptr) {
        throw std::invalid_argument("Missing shared parameter: " + stable_id);
    }
    return *parameter;
}

DesignParameter* requireMutableParameter(ParameterRegistry& registry, const std::string& stable_id) {
    auto* parameter = registry.find(stable_id);
    if (parameter == nullptr) {
        throw std::invalid_argument("Missing shared parameter: " + stable_id);
    }
    return parameter;
}

}  // namespace

BasicSpatialElement::BasicSpatialElement(std::string id, std::string type)
    : id_(std::move(id)),
      type_(std::move(type)) {}

std::string BasicSpatialElement::id() const {
    return id_;
}

std::string BasicSpatialElement::type() const {
    return type_;
}

double BasicSpatialElement::mass() const {
    return mass_;
}

Eigen::Vector3d BasicSpatialElement::localCOM() const {
    return local_com_;
}

Eigen::Matrix3d BasicSpatialElement::localInertiaAtLocalCOM() const {
    return local_inertia_;
}

GeometryPrimitives BasicSpatialElement::localPrimitives() const {
    return primitives_;
}

Eigen::Isometry3d BasicSpatialElement::localPose() const {
    return local_pose_;
}

std::vector<AnchorFrame> BasicSpatialElement::anchors() const {
    return anchors_;
}

std::optional<Eigen::Isometry3d> BasicSpatialElement::anchorPose(const std::string_view anchor_name) const {
    const auto it = std::find_if(anchors_.begin(), anchors_.end(), [&](const AnchorFrame& anchor) {
        return anchor.name == anchor_name;
    });
    if (it == anchors_.end()) {
        return std::nullopt;
    }
    return it->local_pose;
}

void BasicSpatialElement::setAnchor(std::string name, const Eigen::Isometry3d& local_pose) {
    const auto it = std::find_if(anchors_.begin(), anchors_.end(), [&](const AnchorFrame& anchor) {
        return anchor.name == name;
    });
    if (it == anchors_.end()) {
        anchors_.push_back({std::move(name), local_pose});
        return;
    }
    it->local_pose = local_pose;
}

PayloadElement::PayloadElement(std::string id, DesignParameter* payload_mass)
    : BasicSpatialElement(std::move(id), "PayloadElement"),
      payload_mass_(payload_mass) {}

std::unique_ptr<SpatialElement> PayloadElement::clone() const {
    return std::make_unique<PayloadElement>(*this);
}

void PayloadElement::registerParameters(ParameterRegistry&) {
    payload_mass_->addConsumer(id_);
}

void PayloadElement::rebindParameters(ParameterRegistry& registry) {
    payload_mass_ = requireMutableParameter(registry, payload_mass_->stable_id());
}

void PayloadElement::registerConstraints(ConstraintRegistry& registry) const {
    const std::string stable_id = payload_mass_->stable_id();
    registry.add({
        "payload_mass_nonnegative",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        1000.0,
        [stable_id](const ConstraintEvaluationContext& context) {
            const auto& parameter = requireParameter(context.architecture.parameters(), stable_id);
            Constraint constraint{"payload_mass_nonnegative", "payload", ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(parameter.value);
        }
    });
}

void PayloadElement::updateFromParameters() {
    mass_ = payload_mass_->value;
    local_com_.setZero();
    local_inertia_.setZero();
    primitives_ = {GeometryPrimitive::makeSphere(kBaselinePayloadRadius)};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

BodyElement::BodyElement(std::string id, DesignParameter* Lx, DesignParameter* Lyi, DesignParameter* Lyo)
    : BasicSpatialElement(std::move(id), "BodyElement"),
      Lx_(Lx),
      Lyi_(Lyi),
      Lyo_(Lyo) {}

std::unique_ptr<SpatialElement> BodyElement::clone() const {
    return std::make_unique<BodyElement>(*this);
}

void BodyElement::registerParameters(ParameterRegistry&) {
    Lx_->addConsumer(id_);
    Lyi_->addConsumer(id_);
    Lyo_->addConsumer(id_);
}

void BodyElement::rebindParameters(ParameterRegistry& registry) {
    Lx_ = requireMutableParameter(registry, Lx_->stable_id());
    Lyi_ = requireMutableParameter(registry, Lyi_->stable_id());
    Lyo_ = requireMutableParameter(registry, Lyo_->stable_id());
}

void BodyElement::registerConstraints(ConstraintRegistry& registry) const {
    const std::string Lx_id = Lx_->stable_id();
    const std::string Lyi_id = Lyi_->stable_id();
    const std::string Lyo_id = Lyo_->stable_id();
    registry.add({
        "body_span_order",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        1000.0,
        [Lx_id, Lyi_id, Lyo_id](const ConstraintEvaluationContext& context) {
            const auto& Lx = requireParameter(context.architecture.parameters(), Lx_id);
            const auto& Lyi = requireParameter(context.architecture.parameters(), Lyi_id);
            const auto& Lyo = requireParameter(context.architecture.parameters(), Lyo_id);
            const double value = std::min({Lx.value, Lyi.value, Lyo.value - Lyi.value});
            Constraint constraint{"body_span_order", "body", ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(value);
        }
    });
}

void BodyElement::updateFromParameters() {
    const double half_x = std::max(kBodyHalfSizeBase, kBodyHalfSizeScale * Lx_->value + kBodyHalfSizeOffset);
    const double half_y = std::max(kBodyHalfSizeBase, kBodyHalfSizeScale * Lyi_->value + kBodyHalfSizeOffset);
    constexpr double half_z = kBodyHalfHeight;
    mass_ = 0.0;
    local_com_.setZero();
    local_inertia_.setZero();
    primitives_ = {GeometryPrimitive::makeBox({half_x, half_y, half_z}, kBodyBoxPadding)};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());

    Eigen::Isometry3d top = Eigen::Isometry3d::Identity();
    top.translation() = Eigen::Vector3d(0.0, 0.0, half_z);
    setAnchor("top", top);

    Eigen::Isometry3d bottom = Eigen::Isometry3d::Identity();
    bottom.translation() = Eigen::Vector3d(0.0, 0.0, -half_z);
    setAnchor("bottom", bottom);
}

BatteryElement::BatteryElement(std::string id, DesignParameter* thrust_max, DesignParameter* propeller_diameter)
    : BasicSpatialElement(std::move(id), "BatteryElement"),
      thrust_max_(thrust_max),
      propeller_diameter_(propeller_diameter) {}

std::unique_ptr<SpatialElement> BatteryElement::clone() const {
    return std::make_unique<BatteryElement>(*this);
}

void BatteryElement::registerParameters(ParameterRegistry&) {
    thrust_max_->addConsumer(id_);
    propeller_diameter_->addConsumer(id_);
}

void BatteryElement::rebindParameters(ParameterRegistry& registry) {
    thrust_max_ = requireMutableParameter(registry, thrust_max_->stable_id());
    propeller_diameter_ = requireMutableParameter(registry, propeller_diameter_->stable_id());
}

void BatteryElement::registerConstraints(ConstraintRegistry& registry) const {
    const std::string thrust_id = thrust_max_->stable_id();
    const std::string prop_id = propeller_diameter_->stable_id();
    registry.add({
        "battery_sizing_inputs",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        500.0,
        [thrust_id, prop_id](const ConstraintEvaluationContext& context) {
            const auto& thrust = requireParameter(context.architecture.parameters(), thrust_id);
            const auto& prop = requireParameter(context.architecture.parameters(), prop_id);
            const double value = std::min(thrust.value, prop.value);
            Constraint constraint{"battery_sizing_inputs", "battery", ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(value);
        }
    });
}

void BatteryElement::updateFromParameters() {
    const double normalized_thrust = thrust_max_->value / kBaselineMotorTmax;
    const double length = kBatteryLengthBase + kBatteryLengthScale * normalized_thrust;
    const double width = kBatteryWidthBase + kBatteryWidthScale * propeller_diameter_->value;
    mass_ = 0.0;
    local_com_.setZero();
    local_inertia_.setZero();
    primitives_ = {GeometryPrimitive::makeBox({0.5 * length, 0.5 * width, kBatteryHalfHeight}, kBatteryBoxPadding)};
    anchors_.clear();
    setAnchor("mount", Eigen::Isometry3d::Identity());
    setAnchor("center", Eigen::Isometry3d::Identity());
}

ArmElement::ArmElement(
    std::string id,
    const int index,
    DesignParameter* Lx,
    DesignParameter* Lyi,
    DesignParameter* Lyo,
    DesignParameter* Tmax)
    : BasicSpatialElement(std::move(id), "ArmElement"),
      index_(index),
      Lx_(Lx),
      Lyi_(Lyi),
      Lyo_(Lyo),
      Tmax_(Tmax) {}

std::unique_ptr<SpatialElement> ArmElement::clone() const {
    return std::make_unique<ArmElement>(*this);
}

void ArmElement::registerParameters(ParameterRegistry&) {
    Lx_->addConsumer(id_);
    Lyi_->addConsumer(id_);
    Lyo_->addConsumer(id_);
    Tmax_->addConsumer(id_);
}

void ArmElement::rebindParameters(ParameterRegistry& registry) {
    Lx_ = requireMutableParameter(registry, Lx_->stable_id());
    Lyi_ = requireMutableParameter(registry, Lyi_->stable_id());
    Lyo_ = requireMutableParameter(registry, Lyo_->stable_id());
    Tmax_ = requireMutableParameter(registry, Tmax_->stable_id());
}

void ArmElement::registerConstraints(ConstraintRegistry& registry) const {
    const std::string Lx_id = Lx_->stable_id();
    const std::string Lyi_id = Lyi_->stable_id();
    const std::string Lyo_id = Lyo_->stable_id();
    registry.add({
        "arm_length_positive",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        750.0,
        [index = index_, Lx_id, Lyi_id, Lyo_id](const ConstraintEvaluationContext& context) {
            const auto& Lx = requireParameter(context.architecture.parameters(), Lx_id);
            const auto& Lyi = requireParameter(context.architecture.parameters(), Lyi_id);
            const auto& Lyo = requireParameter(context.architecture.parameters(), Lyo_id);
            const double value = armLength(index, Lx.value, Lyi.value, Lyo.value);
            Constraint constraint{"arm_length_positive", "arm", ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(value);
        }
    });
}

void ArmElement::updateFromParameters() {
    const double length = armLength(index_, Lx_->value, Lyi_->value, Lyo_->value);
    mass_ = frameMass(Lx_->value, Lyi_->value, Lyo_->value) / 6.0;
    local_com_.setZero();
    local_inertia_ = Eigen::Matrix3d::Zero();
    local_inertia_(0, 0) = 1e-6;
    local_inertia_(1, 1) = mass_ * length * length / 12.0;
    local_inertia_(2, 2) = local_inertia_(1, 1);
    primitives_ = {GeometryPrimitive::makeSegment(length, 0.05)};
    anchors_.clear();
    setAnchor("root", Eigen::Isometry3d::Identity());
    Eigen::Isometry3d tip = Eigen::Isometry3d::Identity();
    tip.translation() = Eigen::Vector3d(length, 0.0, 0.0);
    setAnchor("tip", tip);
}

int ArmElement::index() const {
    return index_;
}

double ArmElement::structuralSpanContribution() const {
    return !primitives_.empty() ? primitives_.front().dimensions.x() : 0.0;
}

bool ArmElement::contributesToFrameMass() const {
    return true;
}

MotorElement::MotorElement(std::string id, const int index, DesignParameter* thrust_max)
    : BasicSpatialElement(std::move(id), "MotorElement"),
      index_(index),
      thrust_max_(thrust_max) {}

std::unique_ptr<SpatialElement> MotorElement::clone() const {
    return std::make_unique<MotorElement>(*this);
}

void MotorElement::registerParameters(ParameterRegistry&) {
    thrust_max_->addConsumer(id_);
}

void MotorElement::rebindParameters(ParameterRegistry& registry) {
    thrust_max_ = requireMutableParameter(registry, thrust_max_->stable_id());
}

void MotorElement::registerConstraints(ConstraintRegistry& registry) const {
    const std::string thrust_id = thrust_max_->stable_id();
    registry.add({
        "motor_thrust_positive",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        1000.0,
        [thrust_id](const ConstraintEvaluationContext& context) {
            const auto& thrust = requireParameter(context.architecture.parameters(), thrust_id);
            Constraint constraint{"motor_thrust_positive", "motor", ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(thrust.value);
        }
    });
}

void MotorElement::updateFromParameters() {
    const double radius = kMotorRadiusBase + kMotorRadiusScale * (thrust_max_->value / kBaselineMotorTmax);
    mass_ = motorMass(thrust_max_->value);
    local_com_.setZero();
    local_inertia_.setZero();
    GeometryPrimitive motor_cyl = GeometryPrimitive::makeCylinder(radius, kMotorHeight, kMotorCylinderPadding);
    motor_cyl.local_pose.linear() =
        Eigen::AngleAxisd(3.14159265358979323846 / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    primitives_ = {motor_cyl};
    anchors_.clear();
    setAnchor("mount", Eigen::Isometry3d::Identity());
    setAnchor("axis", Eigen::Isometry3d::Identity());
}

int MotorElement::index() const {
    return index_;
}

RotorElement::RotorElement(
    std::string id,
    const int index,
    const double yaw_moment_sign,
    DesignParameter* propeller_diameter)
    : BasicSpatialElement(std::move(id), "RotorElement"),
      index_(index),
      yaw_moment_sign_(yaw_moment_sign),
      propeller_diameter_(propeller_diameter) {}

std::unique_ptr<SpatialElement> RotorElement::clone() const {
    return std::make_unique<RotorElement>(*this);
}

void RotorElement::registerParameters(ParameterRegistry&) {
    propeller_diameter_->addConsumer(id_);
}

void RotorElement::rebindParameters(ParameterRegistry& registry) {
    propeller_diameter_ = requireMutableParameter(registry, propeller_diameter_->stable_id());
}

void RotorElement::registerConstraints(ConstraintRegistry& registry) const {
    const std::string prop_id = propeller_diameter_->stable_id();
    registry.add({
        "rotor_diameter_positive",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        750.0,
        [prop_id](const ConstraintEvaluationContext& context) {
            const auto& prop = requireParameter(context.architecture.parameters(), prop_id);
            Constraint constraint{"rotor_diameter_positive", "rotor", ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(prop.value);
        }
    });
}

void RotorElement::updateFromParameters() {
    mass_ = 0.0;
    local_com_.setZero();
    local_inertia_.setZero();
    primitives_ = {GeometryPrimitive::makeDisk(0.5 * propeller_diameter_->value, 0.01)};
    anchors_.clear();
    setAnchor("axis", Eigen::Isometry3d::Identity());
    setAnchor("center", Eigen::Isometry3d::Identity());
}

int RotorElement::index() const {
    return index_;
}

int RotorElement::rotorIndex() const {
    return index_;
}

double RotorElement::yawMomentSign() const {
    return yaw_moment_sign_;
}

}  // namespace hexaarch::core
