#include "core/Elements.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "eigen3/Eigen/Dense"

#include "core/HexacopterArchitecture.hpp"

namespace hexaarch::core {
namespace {

constexpr double kBaselineMotorMass = 74.07;
constexpr double kBaselineMotorTmax = 7327.0;
constexpr double kMotorMassExponent = 3.0 / 3.5;

// BodyHullElement geometry — outer fuselage hull.
// Sized from cabin envelope + wall margin + a small arm-length term so body grows as arms lengthen.
// This makes body volume a function of the optimization variables (Lx, Lyi) rather than a fixed box,
// satisfying the requirement that body volume is not decoupled from the optimized geometry.
// The arm scale is deliberately small (0.05 m/m) so body stays well within the rotor inner-edge
// distance (≈ arm_length − disk_radius) even at maximum design-space extents.
constexpr double kBodyHullWallMargin = 0.05;   // outer skin thickness over cabin [m]
constexpr double kBodyHullArmScale   = 0.05;   // body expands 5 cm per m of arm length [m/m]
constexpr double kBodyHalfHeight     = 1.25;   // must be >= kCabinHalfZ (1.10)
constexpr double kBodyBoxPadding     = 0.05;

// BodyFrameElement geometry — central hub plate connecting arm roots.
constexpr double kBodyFrameHubRadius = 0.30;
constexpr double kBodyFrameHubHeight = 0.08;

// BatteryElement geometry — physics-based slab model.
// Volume = m_bat / kBatteryPackDensity; height is derived from that volume given a fixed plan.
//   kBatteryPackDensity: 1500 kg/m³ ≈ NMC811 cell at ~600 Wh/L with 60-70% pack packing factor.
//   Plan area (slab_half_x × slab_half_y): sized to cover most of the cabin ceiling cross-section.
// This makes the battery grow visibly as m_bat increases and shrink as it decreases, giving the
// optimizer a physically correct volume signal rather than an arbitrary empirical scaling.
constexpr double kBatteryPackDensity = 1500.0;  // [kg/m³] — Li-ion pack including BMS / cooling
constexpr double kBatterySlabHalfX   = 0.65;    // fore/aft half-extent [m] → 1.30 m total
constexpr double kBatterySlabHalfY   = 0.75;    // lateral half-extent [m]  → 1.50 m total
constexpr double kBatteryBoxPadding  = 0.02;    // visual/envelope clearance margin [m]

// CabinEnvelopeElement — passenger cabin inner hull [m].
// 2.20 m total height gives realistic standing headroom relative to the mannequin reference asset.
// Battery zone (z<0), passenger seats (z≈0), and cargo floor (z>0) are vertically stratified
// inside this envelope; none share the same z band at the default placement.
constexpr double kCabinHalfX = 0.80;   // 1.60 m fore/aft — fits 2 seat rows + instrument panel
constexpr double kCabinHalfY = 0.90;   // 1.80 m lateral — fits 2 seats abreast with shoulder room
constexpr double kCabinHalfZ = 1.10;   // 2.20 m total — standing adult headroom

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

constexpr double kPayloadBoxHalfX = 0.60;
constexpr double kPayloadBoxHalfY = 0.60;
constexpr double kPayloadBoxHalfZ = 0.50;

void PayloadElement::updateFromParameters() {
    mass_ = payload_mass_->value;
    local_com_.setZero();
    local_inertia_.setZero();
    primitives_ = {GeometryPrimitive::makeBox({kPayloadBoxHalfX, kPayloadBoxHalfY, kPayloadBoxHalfZ}, 0.0)};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

LocalAABB PayloadElement::localEnvelope() const {
    return {
        Eigen::Vector3d{-kPayloadBoxHalfX, -kPayloadBoxHalfY, -kPayloadBoxHalfZ},
        Eigen::Vector3d{+kPayloadBoxHalfX, +kPayloadBoxHalfY, +kPayloadBoxHalfZ}
    };
}

LocalAABB BodyHullElement::localEnvelope() const {
    const double half_x = kCabinHalfX + kBodyHullWallMargin + kBodyHullArmScale * Lx_->value;
    const double half_y = kCabinHalfY + kBodyHullWallMargin + kBodyHullArmScale * Lyi_->value;
    return {
        Eigen::Vector3d{-half_x, -half_y, -kBodyHalfHeight},
        Eigen::Vector3d{+half_x, +half_y, +kBodyHalfHeight}
    };
}

BodyHullElement::BodyHullElement(std::string id, DesignParameter* Lx, DesignParameter* Lyi, DesignParameter* Lyo)
    : BasicSpatialElement(std::move(id), "BodyHullElement"),
      Lx_(Lx),
      Lyi_(Lyi),
      Lyo_(Lyo) {}

std::unique_ptr<SpatialElement> BodyHullElement::clone() const {
    return std::make_unique<BodyHullElement>(*this);
}

void BodyHullElement::registerParameters(ParameterRegistry&) {
    Lx_->addConsumer(id_);
    Lyi_->addConsumer(id_);
    Lyo_->addConsumer(id_);
}

void BodyHullElement::rebindParameters(ParameterRegistry& registry) {
    Lx_ = requireMutableParameter(registry, Lx_->stable_id());
    Lyi_ = requireMutableParameter(registry, Lyi_->stable_id());
    Lyo_ = requireMutableParameter(registry, Lyo_->stable_id());
}

void BodyHullElement::registerConstraints(ConstraintRegistry& registry) const {
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

void BodyHullElement::updateFromParameters() {
    // Width/depth tracks cabin + wall margin + arm-length contribution (optimization target).
    // At baseline Lx=Lyi=2.65: half_x≈0.98 m, half_y≈1.08 m — comfortably smaller than rotor
    // inner-edge at ~1.96 m from body center. Grows modestly as arms lengthen.
    const double half_x = kCabinHalfX + kBodyHullWallMargin + kBodyHullArmScale * Lx_->value;
    const double half_y = kCabinHalfY + kBodyHullWallMargin + kBodyHullArmScale * Lyi_->value;
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

BodyFrameElement::BodyFrameElement(std::string id)
    : BasicSpatialElement(std::move(id), "BodyFrameElement") {}

std::unique_ptr<SpatialElement> BodyFrameElement::clone() const {
    return std::make_unique<BodyFrameElement>(*this);
}

void BodyFrameElement::registerParameters(ParameterRegistry&) {}
void BodyFrameElement::rebindParameters(ParameterRegistry&) {}
void BodyFrameElement::registerConstraints(ConstraintRegistry&) const {}

void BodyFrameElement::updateFromParameters() {
    mass_ = 0.0;
    local_com_.setZero();
    local_inertia_.setZero();
    GeometryPrimitive hub = GeometryPrimitive::makeCylinder(kBodyFrameHubRadius, kBodyFrameHubHeight, 0.0);
    hub.local_pose.linear() =
        Eigen::AngleAxisd(3.14159265358979323846 / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    primitives_ = {hub};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

BatteryElement::BatteryElement(
    std::string id,
    DesignParameter* m_bat,
    DesignParameter* bat_x,
    DesignParameter* bat_y,
    DesignParameter* bat_z)
    : BasicSpatialElement(std::move(id), "BatteryElement"),
      m_bat_(m_bat),
      bat_x_(bat_x),
      bat_y_(bat_y),
      bat_z_(bat_z) {}

std::unique_ptr<SpatialElement> BatteryElement::clone() const {
    return std::make_unique<BatteryElement>(*this);
}

void BatteryElement::registerParameters(ParameterRegistry&) {
    m_bat_->addConsumer(id_);
    bat_x_->addConsumer(id_);
    bat_y_->addConsumer(id_);
    bat_z_->addConsumer(id_);
}

void BatteryElement::rebindParameters(ParameterRegistry& registry) {
    m_bat_  = requireMutableParameter(registry, m_bat_->stable_id());
    bat_x_  = requireMutableParameter(registry, bat_x_->stable_id());
    bat_y_  = requireMutableParameter(registry, bat_y_->stable_id());
    bat_z_  = requireMutableParameter(registry, bat_z_->stable_id());
}

void BatteryElement::registerConstraints(ConstraintRegistry& registry) const {
    const std::string mbat_id = m_bat_->stable_id();
    registry.add({
        "battery_mass_positive",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        500.0,
        [mbat_id](const ConstraintEvaluationContext& context) {
            const auto& mbat = requireParameter(context.architecture.parameters(), mbat_id);
            Constraint constraint{"battery_mass_positive", "battery", ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(mbat.value);
        }
    });
}

void BatteryElement::updateFromParameters() {
    mass_ = m_bat_->value;
    local_com_.setZero();
    local_inertia_.setZero();

    // Placement owned by local_pose_; attachment is a rigid mount (no lambda).
    local_pose_ = Eigen::Isometry3d::Identity();
    local_pose_.translation() = Eigen::Vector3d(bat_x_->value, bat_y_->value, bat_z_->value);

    // Physical height from pack volume. Packaging constraints use this same envelope, so large
    // battery masses cannot hide in a fixed-thickness visual slab.
    const double volume = std::max(m_bat_->value, 1.0) / kBatteryPackDensity;
    half_z_ = volume / (4.0 * kBatterySlabHalfX * kBatterySlabHalfY);

    // Primitive in element-local frame; local_pose_ carries the world offset.
    primitives_ = {GeometryPrimitive::makeBox(
        {kBatterySlabHalfX, kBatterySlabHalfY, half_z_}, kBatteryBoxPadding)};
    anchors_.clear();
    setAnchor("mount", Eigen::Isometry3d::Identity());
    setAnchor("center", Eigen::Isometry3d::Identity());
}

double BatteryElement::batteryMass() const {
    return m_bat_->value;
}

LocalAABB BatteryElement::localEnvelope() const {
    // AABB in element-local frame; ArchitecturePackagingEvaluator adds world_pose translation.
    const double hx = kBatterySlabHalfX + kBatteryBoxPadding;
    const double hy = kBatterySlabHalfY + kBatteryBoxPadding;
    const double hz = half_z_           + kBatteryBoxPadding;
    return {Eigen::Vector3d{-hx, -hy, -hz}, Eigen::Vector3d{+hx, +hy, +hz}};
}

CabinEnvelopeElement::CabinEnvelopeElement(std::string id)
    : BasicSpatialElement(std::move(id), "CabinEnvelopeElement") {}

std::unique_ptr<SpatialElement> CabinEnvelopeElement::clone() const {
    return std::make_unique<CabinEnvelopeElement>(*this);
}

void CabinEnvelopeElement::registerParameters(ParameterRegistry&) {}
void CabinEnvelopeElement::rebindParameters(ParameterRegistry&) {}
void CabinEnvelopeElement::registerConstraints(ConstraintRegistry&) const {}

void CabinEnvelopeElement::updateFromParameters() {
    mass_ = 0.0;
    local_com_.setZero();
    local_inertia_.setZero();
    primitives_ = {GeometryPrimitive::makeBox({kCabinHalfX, kCabinHalfY, kCabinHalfZ}, 0.0)};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

LocalAABB CabinEnvelopeElement::localEnvelope() const {
    return {
        Eigen::Vector3d{-kCabinHalfX, -kCabinHalfY, -kCabinHalfZ},
        Eigen::Vector3d{+kCabinHalfX, +kCabinHalfY, +kCabinHalfZ}
    };
}

// OccupantEnvelopeElement — minimum required 4-pax occupancy space.
// Values match PassengerElement group AABB (kPaxGroupHalfX/Y/Z defined later in this TU).
constexpr double kOccupantHalfX = 0.53;   // = kSeatRowPitch(0.28) + kSeatHalfX(0.25)
constexpr double kOccupantHalfY = 0.47;   // = kSeatSpanY(0.25) + kSeatHalfY(0.22)
constexpr double kOccupantHalfZ = 0.60;   // = kSeatHalfZ

OccupantEnvelopeElement::OccupantEnvelopeElement(std::string id)
    : BasicSpatialElement(std::move(id), "OccupantEnvelopeElement") {}

std::unique_ptr<SpatialElement> OccupantEnvelopeElement::clone() const {
    return std::make_unique<OccupantEnvelopeElement>(*this);
}

void OccupantEnvelopeElement::registerParameters(ParameterRegistry&) {}
void OccupantEnvelopeElement::rebindParameters(ParameterRegistry&) {}
void OccupantEnvelopeElement::registerConstraints(ConstraintRegistry&) const {}

void OccupantEnvelopeElement::updateFromParameters() {
    mass_ = 0.0;
    local_com_.setZero();
    local_inertia_.setZero();
    primitives_ = {GeometryPrimitive::makeBox({kOccupantHalfX, kOccupantHalfY, kOccupantHalfZ}, 0.0)};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

LocalAABB OccupantEnvelopeElement::localEnvelope() const {
    return {
        Eigen::Vector3d{-kOccupantHalfX, -kOccupantHalfY, -kOccupantHalfZ},
        Eigen::Vector3d{+kOccupantHalfX, +kOccupantHalfY, +kOccupantHalfZ}
    };
}

// PassengerElement — 4 passengers in symmetric 2×2 seating arrangement.
// Seat dimensions calibrated against the 1.80 m UE4 mannequin reference:
//   seated height (floor to top of head) ≈ 1.20 m → kSeatHalfZ = 0.60 m
//   seat pitch (fore/aft center-to-center) = 0.56 m → kSeatRowPitch = 0.28 m
//   seat width (lateral center-to-center) = 0.50 m → kSeatSpanY = 0.25 m
// Group center is at local origin; local_pose_ places the group in body frame.
constexpr double kSeatHalfX    = 0.25;   // fore/aft half-depth per seat (0.50 m seat+leg)
constexpr double kSeatHalfY    = 0.22;   // lateral half-width per seat (0.44 m shoulder)
constexpr double kSeatHalfZ    = 0.60;   // vertical half-height (1.20 m floor-to-head seated)
constexpr double kSeatRowPitch = 0.28;   // row center offset from group center
constexpr double kSeatSpanY    = 0.25;   // lateral center offset from group center
// AABB of the full 4-seat group in element-local frame:
constexpr double kPaxGroupHalfX = kSeatRowPitch + kSeatHalfX;   // 0.53 m
constexpr double kPaxGroupHalfY = kSeatSpanY    + kSeatHalfY;   // 0.47 m
constexpr double kPaxGroupHalfZ = kSeatHalfZ;                    // 0.60 m

PassengerElement::PassengerElement(std::string id,
                                   DesignParameter* mass,
                                   DesignParameter* pax_x,
                                   DesignParameter* pax_y,
                                   DesignParameter* pax_z)
    : BasicSpatialElement(std::move(id), "PassengerElement"),
      mass_(mass),
      pax_x_(pax_x),
      pax_y_(pax_y),
      pax_z_(pax_z) {}

std::unique_ptr<SpatialElement> PassengerElement::clone() const {
    return std::make_unique<PassengerElement>(*this);
}

void PassengerElement::registerParameters(ParameterRegistry&) {
    mass_->addConsumer(id_);
    pax_x_->addConsumer(id_);
    pax_y_->addConsumer(id_);
    pax_z_->addConsumer(id_);
}

void PassengerElement::rebindParameters(ParameterRegistry& registry) {
    mass_  = requireMutableParameter(registry, mass_->stable_id());
    pax_x_ = requireMutableParameter(registry, pax_x_->stable_id());
    pax_y_ = requireMutableParameter(registry, pax_y_->stable_id());
    pax_z_ = requireMutableParameter(registry, pax_z_->stable_id());
}

void PassengerElement::registerConstraints(ConstraintRegistry&) const {}

void PassengerElement::updateFromParameters() {
    BasicSpatialElement::mass_ = mass_->value;
    // Symmetric 2×2 arrangement → COM at group origin.
    local_com_.setZero();
    local_inertia_.setZero();

    // Group center placement via local_pose_; attachment is a rigid mount.
    local_pose_ = Eigen::Isometry3d::Identity();
    local_pose_.translation() = Eigen::Vector3d(pax_x_->value, pax_y_->value, pax_z_->value);

    // 4 seat boxes placed symmetrically in element-local frame (centered at origin).
    const std::array<Eigen::Vector3d, 4> positions{{
        { kSeatRowPitch, -kSeatSpanY, 0.0},  // front-left
        { kSeatRowPitch,  kSeatSpanY, 0.0},  // front-right
        {-kSeatRowPitch, -kSeatSpanY, 0.0},  // rear-left
        {-kSeatRowPitch,  kSeatSpanY, 0.0}   // rear-right
    }};
    primitives_.clear();
    primitives_.reserve(4);
    for (const auto& pos : positions) {
        GeometryPrimitive seat = GeometryPrimitive::makeBox({kSeatHalfX, kSeatHalfY, kSeatHalfZ}, 0.0);
        seat.local_pose.translation() = pos;
        primitives_.push_back(seat);
    }

    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

LocalAABB PassengerElement::localEnvelope() const {
    // AABB of the full 4-seat group in element-local frame.
    return {
        Eigen::Vector3d{-kPaxGroupHalfX, -kPaxGroupHalfY, -kPaxGroupHalfZ},
        Eigen::Vector3d{+kPaxGroupHalfX, +kPaxGroupHalfY, +kPaxGroupHalfZ}
    };
}

// CargoElement — baggage/cargo slab. Default placement below the passenger zone (cargo_z > 0).
constexpr double kCargoHalfX = 0.30;   // 0.60 m fore/aft
constexpr double kCargoHalfY = 0.42;   // 0.84 m lateral (close to cabin width)
constexpr double kCargoHalfZ = 0.18;   // 0.36 m tall (flat slab under passenger floor)

CargoElement::CargoElement(std::string id,
                            DesignParameter* mass,
                            DesignParameter* cargo_x,
                            DesignParameter* cargo_y,
                            DesignParameter* cargo_z)
    : BasicSpatialElement(std::move(id), "CargoElement"),
      mass_(mass),
      cargo_x_(cargo_x),
      cargo_y_(cargo_y),
      cargo_z_(cargo_z) {}

std::unique_ptr<SpatialElement> CargoElement::clone() const {
    return std::make_unique<CargoElement>(*this);
}

void CargoElement::registerParameters(ParameterRegistry&) {
    mass_->addConsumer(id_);
    cargo_x_->addConsumer(id_);
    cargo_y_->addConsumer(id_);
    cargo_z_->addConsumer(id_);
}

void CargoElement::rebindParameters(ParameterRegistry& registry) {
    mass_    = requireMutableParameter(registry, mass_->stable_id());
    cargo_x_ = requireMutableParameter(registry, cargo_x_->stable_id());
    cargo_y_ = requireMutableParameter(registry, cargo_y_->stable_id());
    cargo_z_ = requireMutableParameter(registry, cargo_z_->stable_id());
}

void CargoElement::registerConstraints(ConstraintRegistry&) const {}

void CargoElement::updateFromParameters() {
    BasicSpatialElement::mass_ = mass_->value;
    local_com_.setZero();
    local_inertia_.setZero();

    local_pose_ = Eigen::Isometry3d::Identity();
    local_pose_.translation() = Eigen::Vector3d(cargo_x_->value, cargo_y_->value, cargo_z_->value);

    primitives_ = {GeometryPrimitive::makeBox({kCargoHalfX, kCargoHalfY, kCargoHalfZ}, 0.0)};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

LocalAABB CargoElement::localEnvelope() const {
    return {
        Eigen::Vector3d{-kCargoHalfX, -kCargoHalfY, -kCargoHalfZ},
        Eigen::Vector3d{+kCargoHalfX, +kCargoHalfY, +kCargoHalfZ}
    };
}

// InstrumentPanelElement — avionics/instrument panel fixed at cabin front (+0.68 m from body center).
// Half-x = 0.08 m (very thin panel); half-y spans nearly the full cabin width.
constexpr double kInstHalfX = 0.08;
constexpr double kInstHalfY = 0.62;   // 1.24 m panel width (nearly full cabin lateral extent)
constexpr double kInstHalfZ = 0.35;   // 0.70 m panel height

InstrumentPanelElement::InstrumentPanelElement(std::string id, DesignParameter* mass)
    : BasicSpatialElement(std::move(id), "InstrumentPanelElement"),
      mass_(mass) {}

std::unique_ptr<SpatialElement> InstrumentPanelElement::clone() const {
    return std::make_unique<InstrumentPanelElement>(*this);
}

void InstrumentPanelElement::registerParameters(ParameterRegistry&) {
    mass_->addConsumer(id_);
}

void InstrumentPanelElement::rebindParameters(ParameterRegistry& registry) {
    mass_ = requireMutableParameter(registry, mass_->stable_id());
}

void InstrumentPanelElement::registerConstraints(ConstraintRegistry&) const {}

void InstrumentPanelElement::updateFromParameters() {
    BasicSpatialElement::mass_ = mass_->value;
    local_com_.setZero();
    local_inertia_.setZero();
    // Fixed position at cabin front: +0.68 m forward from body center (no DOF).
    // Ownership in local_pose_ keeps the attachment as a simple rigid mount.
    local_pose_ = Eigen::Isometry3d::Identity();
    local_pose_.translation() = Eigen::Vector3d(0.68, 0.0, 0.0);
    primitives_ = {GeometryPrimitive::makeBox({kInstHalfX, kInstHalfY, kInstHalfZ}, 0.0)};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

LocalAABB InstrumentPanelElement::localEnvelope() const {
    return {
        Eigen::Vector3d{-kInstHalfX, -kInstHalfY, -kInstHalfZ},
        Eigen::Vector3d{+kInstHalfX, +kInstHalfY, +kInstHalfZ}
    };
}

ArmElement::ArmElement(
    std::string id,
    const int index,
    DesignParameter* Lx,
    DesignParameter* Lyi,
    DesignParameter* Lyo,
    DesignParameter* r_o,
    DesignParameter* t_wall)
    : BasicSpatialElement(std::move(id), "ArmElement"),
      index_(index),
      Lx_(Lx),
      Lyi_(Lyi),
      Lyo_(Lyo),
      r_o_(r_o),
      t_wall_(t_wall) {}

std::unique_ptr<SpatialElement> ArmElement::clone() const {
    return std::make_unique<ArmElement>(*this);
}

void ArmElement::registerParameters(ParameterRegistry&) {
    Lx_->addConsumer(id_);
    Lyi_->addConsumer(id_);
    Lyo_->addConsumer(id_);
    r_o_->addConsumer(id_);
    t_wall_->addConsumer(id_);
}

void ArmElement::rebindParameters(ParameterRegistry& registry) {
    Lx_ = requireMutableParameter(registry, Lx_->stable_id());
    Lyi_ = requireMutableParameter(registry, Lyi_->stable_id());
    Lyo_ = requireMutableParameter(registry, Lyo_->stable_id());
    r_o_ = requireMutableParameter(registry, r_o_->stable_id());
    t_wall_ = requireMutableParameter(registry, t_wall_->stable_id());
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
    constexpr double kPi = 3.14159265358979323846;
    const double length = armLength(index_, Lx_->value, Lyi_->value, Lyo_->value);
    mass_ = frameMass(Lx_->value, Lyi_->value, Lyo_->value) / 6.0;
    local_com_.setZero();
    local_inertia_ = Eigen::Matrix3d::Zero();
    local_inertia_(0, 0) = 1e-6;
    local_inertia_(1, 1) = mass_ * length * length / 12.0;
    local_inertia_(2, 2) = local_inertia_(1, 1);

    r_o_val_ = r_o_->value;
    r_i_val_ = r_o_->value - t_wall_->value;
    const double ro2 = r_o_val_ * r_o_val_;
    const double ri2 = r_i_val_ * r_i_val_;
    cross_section_area_ = kPi * (ro2 - ri2);
    second_moment_of_area_ = kPi / 4.0 * (ro2 * ro2 - ri2 * ri2);

    primitives_ = {GeometryPrimitive::makeSegment(length, r_o_val_)};
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

double ArmElement::outerRadius() const {
    return r_o_val_;
}

double ArmElement::innerRadius() const {
    return r_i_val_;
}

double ArmElement::crossSectionArea() const {
    return cross_section_area_;
}

double ArmElement::secondMomentOfArea() const {
    return second_moment_of_area_;
}

double ArmElement::polarMomentOfArea() const {
    // For circular hollow section: J = π/32*(Do⁴−Di⁴) = 2I exactly.
    return 2.0 * second_moment_of_area_;
}

void ArmElement::clearLoads() {
    loads_.clear();
}

void ArmElement::addLoad(const AppliedLoad& load) {
    loads_.push_back(load);
}

const std::vector<AppliedLoad>& ArmElement::loads() const {
    return loads_;
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
    const double normalized = thrust_max_->value / kBaselineMotorTmax;
    const double radius = kMotorRadiusBase + kMotorRadiusScale * normalized;
    const double height = kMotorHeight * std::pow(normalized, 1.0 / 3.0);
    mass_ = motorMass(thrust_max_->value);
    local_com_.setZero();
    local_inertia_.setZero();
    GeometryPrimitive motor_cyl = GeometryPrimitive::makeCylinder(radius, height, kMotorCylinderPadding);
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

void RotorElement::registerThrustMax(DesignParameter* thrust_max) {
    thrust_max_ = thrust_max;
}

void RotorElement::updateFromParameters() {
    mass_ = 0.0;
    local_com_.setZero();
    local_inertia_.setZero();

    double disk_radius;
    if (thrust_max_ != nullptr) {
        // Mach-tip blade-loading formula: T = C_T * rho * pi * R^2 * V_tip^2
        // d_prop stays frozen at 0.40 m for cT yaw torque; visual disk uses T_max.
        constexpr double kCT      = 0.02;
        constexpr double kRho     = 1.225;
        constexpr double kPi      = 3.14159265358979323846;
        constexpr double kMtip    = 0.65;
        constexpr double kVsound  = 340.0;
        const double Vtip2 = (kMtip * kVsound) * (kMtip * kVsound);
        disk_radius = std::sqrt(thrust_max_->value / (kCT * kRho * kPi * Vtip2));
    } else {
        disk_radius = 0.5 * propeller_diameter_->value;
    }

    primitives_ = {GeometryPrimitive::makeDisk(disk_radius, 0.01)};
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
