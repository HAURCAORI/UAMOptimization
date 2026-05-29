#include "core/HexacopterArchitecture.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <string>
#include <unordered_map>
#include <utility>

#include "evaluation/EvaluationContext.hpp"
#include "evaluation/EvaluationResult.hpp"

namespace hexaarch::core {
constexpr double kBaselineLx = 2.65;
constexpr double kBaselineLyi = 2.65;
constexpr double kBaselineLyo = 5.50;
constexpr double kBaselineCT = 0.03;
constexpr double kBaselinePropDiameter = 0.40;
// Motor-2 fault (0-indexed; x=0, y=-Lyo) is the binding ACS case: requires T_max >= mg/2.
// At payload=800, m_bat=400, default arms: m_total≈2130 kg → mg/2≈10450 N.
// Default 12000 N starts the optimizer inside the feasible region (hover_margin≈+0.12).
// Lower bound 8000 N remains valid for compact, light designs (min arms + low m_bat).
constexpr double kBaselineTmax = 12000.0;
constexpr double kBaselineBatteryMass = 400.0;  // Phase 2: default battery mass [kg]

std::string canonicalPairKey(const std::string_view lhs_id, const std::string_view rhs_id) {
    if (lhs_id <= rhs_id) {
        return std::string(lhs_id) + "||" + std::string(rhs_id);
    }
    return std::string(rhs_id) + "||" + std::string(lhs_id);
}

HexacopterArchitecture::HexacopterArchitecture()
    : id_("default-architecture") {
    registerDefaultParameters();
    bindCanonicalParameters();
    rebuildElements();
    registerDefaultConstraints();
    registerElementConstraints();
    rebuildAttachments();
    assemble();
}

HexacopterArchitecture::HexacopterArchitecture(EmptyTag)
    : id_("custom-architecture") {
    registerDefaultParameters();
    bindCanonicalParameters();
}

HexacopterArchitecture HexacopterArchitecture::createEmpty() {
    return HexacopterArchitecture{EmptyTag{}};
}

HexacopterArchitecture::HexacopterArchitecture(const HexacopterArchitecture& other)
    : id_(other.id_),
      nominal_mass_(other.nominal_mass_),
      use_vehicle_model_(other.use_vehicle_model_),
      gravity_(other.gravity_),
      parameters_(other.parameters_),
      constraints_(other.constraints_),
      attachments_(other.attachments_) {
    bindCanonicalParameters();
    elements_.reserve(other.elements_.size());
    for (const auto& element : other.elements_) {
        auto clone = element->clone();
        clone->rebindParameters(parameters_);
        clone->updateFromParameters();
        elements_.push_back(std::move(clone));
    }
    assemble();
}

HexacopterArchitecture& HexacopterArchitecture::operator=(const HexacopterArchitecture& other) {
    if (this == &other) {
        return *this;
    }

    id_ = other.id_;
    nominal_mass_ = other.nominal_mass_;
    use_vehicle_model_ = other.use_vehicle_model_;
    gravity_ = other.gravity_;
    parameters_ = other.parameters_;
    constraints_ = other.constraints_;
    attachments_ = other.attachments_;
    bindCanonicalParameters();
    elements_.clear();
    elements_.reserve(other.elements_.size());
    for (const auto& element : other.elements_) {
        auto clone = element->clone();
        clone->rebindParameters(parameters_);
        clone->updateFromParameters();
        elements_.push_back(std::move(clone));
    }
    assemble();
    return *this;
}

const std::string& HexacopterArchitecture::id() const {
    return id_;
}

ParameterRegistry& HexacopterArchitecture::parameters() {
    return parameters_;
}

const ParameterRegistry& HexacopterArchitecture::parameters() const {
    return parameters_;
}

ConstraintRegistry& HexacopterArchitecture::constraints() {
    return constraints_;
}

const ConstraintRegistry& HexacopterArchitecture::constraints() const {
    return constraints_;
}

const std::vector<SpatialElementPtr>& HexacopterArchitecture::elements() const {
    return elements_;
}

const std::vector<Attachment>& HexacopterArchitecture::attachments() const {
    return attachments_;
}

const AssemblyState& HexacopterArchitecture::assemblyState() const {
    return assembly_state_;
}

const SpatialElement* HexacopterArchitecture::findElement(const std::string_view element_id) const {
    const auto it = std::find_if(elements_.begin(), elements_.end(), [&](const auto& element) {
        return element->id() == element_id;
    });
    return it == elements_.end() ? nullptr : it->get();
}

bool HexacopterArchitecture::isPackagingPairExempt(const std::string_view lhs_id, const std::string_view rhs_id) const {
    return packaging_exemptions_.contains(packagingPairKey(lhs_id, rhs_id));
}

void HexacopterArchitecture::addElement(SpatialElementPtr element) {
    if (element == nullptr) {
        throw std::invalid_argument("Cannot add null element.");
    }
    if (findElement(element->id()) != nullptr) {
        throw std::invalid_argument("Element already exists: " + element->id());
    }
    element->rebindParameters(parameters_);
    element->registerParameters(parameters_);
    element->updateFromParameters();
    constraints_.removeByOwner(element->id());
    element->registerConstraints(constraints_);
    elements_.push_back(std::move(element));
}

bool HexacopterArchitecture::removeElement(const std::string_view element_id) {
    const auto previous_size = elements_.size();
    elements_.erase(
        std::remove_if(elements_.begin(), elements_.end(), [&](const auto& element) {
            return element->id() == element_id;
        }),
        elements_.end());
    if (elements_.size() == previous_size) {
        return false;
    }
    constraints_.removeByOwner(element_id);
    attachments_.erase(
        std::remove_if(attachments_.begin(), attachments_.end(), [&](const Attachment& attachment) {
            return attachment.parent_id == element_id || attachment.child_id == element_id;
        }),
        attachments_.end());
    assemble();
    return true;
}

void HexacopterArchitecture::clearElements() {
    for (const auto& element : elements_) {
        constraints_.removeByOwner(element->id());
    }
    elements_.clear();
    attachments_.clear();
    packaging_exemptions_.clear();
    assembly_state_.elements.clear();
}

void HexacopterArchitecture::addAttachment(Attachment attachment) {
    attachments_.push_back(std::move(attachment));
}

bool HexacopterArchitecture::removeAttachment(const std::string_view child_id) {
    const auto previous_size = attachments_.size();
    attachments_.erase(
        std::remove_if(attachments_.begin(), attachments_.end(), [&](const Attachment& attachment) {
            return attachment.child_id == child_id;
        }),
        attachments_.end());
    if (attachments_.size() == previous_size) {
        return false;
    }
    assemble();
    return true;
}

void HexacopterArchitecture::clearAttachments() {
    attachments_.clear();
    packaging_exemptions_.clear();
    assembly_state_.elements.clear();
}

void HexacopterArchitecture::addConstraint(Constraint constraint) {
    constraints_.add(std::move(constraint));
}

bool HexacopterArchitecture::removeConstraint(const std::string_view stable_id) {
    return constraints_.remove(stable_id);
}

void HexacopterArchitecture::rebuildAssembly() {
    for (auto& element : elements_) {
        element->updateFromParameters();
    }
    assemble();
}

void HexacopterArchitecture::updateFromParameters() {
    for (auto& element : elements_) {
        element->updateFromParameters();
    }
    assemble();
}

double HexacopterArchitecture::Lx() const {
    return Lx_parameter_->value;
}

double HexacopterArchitecture::Lyi() const {
    return Lyi_parameter_->value;
}

double HexacopterArchitecture::Lyo() const {
    return Lyo_parameter_->value;
}

double HexacopterArchitecture::Tmax() const {
    return Tmax_parameter_->value;
}

double HexacopterArchitecture::cT() const {
    return cT_parameter_->value;
}

double HexacopterArchitecture::propellerDiameter() const {
    return dprop_parameter_->value;
}

double HexacopterArchitecture::payloadMass() const {
    return m_pax_parameter_->value + m_cargo_parameter_->value + m_instrument_parameter_->value;
}

double HexacopterArchitecture::batteryMass() const {
    return mbat_parameter_->value;
}

double HexacopterArchitecture::batteryX() const { return bat_x_parameter_->value; }
double HexacopterArchitecture::batteryY() const { return bat_y_parameter_->value; }
double HexacopterArchitecture::batteryZ() const { return bat_z_parameter_->value; }
double HexacopterArchitecture::passengerX() const { return pax_x_parameter_->value; }
double HexacopterArchitecture::passengerY() const { return pax_y_parameter_->value; }
double HexacopterArchitecture::passengerZ() const { return pax_z_parameter_->value; }
double HexacopterArchitecture::cargoX() const { return cargo_x_parameter_->value; }
double HexacopterArchitecture::cargoY() const { return cargo_y_parameter_->value; }
double HexacopterArchitecture::cargoZ() const { return cargo_z_parameter_->value; }

bool HexacopterArchitecture::useVehicleModel() const {
    return use_vehicle_model_;
}

double HexacopterArchitecture::nominalMass() const {
    return nominal_mass_;
}

double HexacopterArchitecture::gravity() const {
    return gravity_;
}

std::string HexacopterArchitecture::packagingPairKey(const std::string_view lhs_id, const std::string_view rhs_id) {
    return canonicalPairKey(lhs_id, rhs_id);
}

void HexacopterArchitecture::registerDefaultParameters() {
    parameters_.clear();

    // Lower bounds 2.0m prevent arm_1256 = sqrt(Lx²+Lyi²) from collapsing to ~1.4m while Lyo→9m.
    // At Lx=Lyi=2.0, arm_1256 >= 2.83m vs arm_34 max 9m → ratio ≤ 3.2 (vs 6.4 at old bound 1.0).
    parameters_.add({"Lx", id_, "m", "Fore/aft arm length", kBaselineLx, 2.0, 5.0, kBaselineLx, true, 1.0});
    parameters_.add({"Lyi", id_, "m", "Inner lateral arm length", kBaselineLyi, 2.0, 5.0, kBaselineLyi, true, 1.0});
    parameters_.add({"Lyo", id_, "m", "Outer lateral arm length", kBaselineLyo, 2.5, 9.0, kBaselineLyo, true, 1.0});
    parameters_.add({"T_max", id_, "N", "Maximum thrust per motor", kBaselineTmax, 8000.0, 20000.0, kBaselineTmax, true, 1.0});
    parameters_.add({"cT", id_, "-", "Moment to thrust ratio", kBaselineCT, 0.01, 0.08, kBaselineCT, false, 1.0});
    parameters_.add({"d_prop", id_, "m", "Propeller diameter", kBaselinePropDiameter, 0.20, 1.20, kBaselinePropDiameter, false, 1.0});
    parameters_.add({"arm_outer_radius", id_, "m", "Arm tube outer radius", 0.08, 0.02, 0.15, 0.08, true, 1.0});
    parameters_.add({"arm_wall_thickness", id_, "m", "Arm tube wall thickness", 0.005, 0.001, 0.020, 0.005, true, 1.0});
    // Phase 2: battery mass design variable. Contributes to total mass and energy capacity.
    parameters_.add({"m_bat", id_, "kg", "Battery pack mass", kBaselineBatteryMass, 100.0, 1000.0, kBaselineBatteryMass, true, 1.0});
    // Mass is fixed mission requirement — not optimized
    parameters_.add({"m_pax",        id_, "kg", "Passenger mass",        600.0, 0.0, 2000.0,  600.0, false, 1.0});
    parameters_.add({"m_cargo",      id_, "kg", "Cargo/baggage mass",    200.0, 0.0, 1000.0,  200.0, false, 1.0});
    parameters_.add({"m_instrument", id_, "kg", "Instrument panel mass",  50.0, 0.0,  500.0,   50.0, false, 1.0});
    // Placement DOFs (active — optimizer can adjust component positions)
    // 3-DOF placement in body-frame NED (z-down: −z = above hub = toward ceiling; +z = toward floor).
    // Z-stratified default layout — three distinct vertical zones (no x/y overlap needed):
    //
    //   Zone 1 — battery ceiling slab (bat_z = −0.90):
    //     At 400 kg default, half_z ≈ 0.137 m → battery z ∈ [−1.037, −0.763]
    //     Cabin ceiling at z=−1.10 → 0.063 m headroom above battery.
    //
    //   Zone 2 — passenger mid-cabin (pax_z = −0.10, half_z = 0.60):
    //     Passenger z ∈ [−0.70, +0.50]
    //     Battery bottom at −0.763 is above passenger top at −0.70 → gap ≈ 0.063 m ✓
    //
    //   Zone 3 — cargo floor slab (cargo_z = +0.75, half_z = 0.18):
    //     Cargo z ∈ [+0.57, +0.93]
    //     Passenger bottom at +0.50 is above cargo top at +0.57 → gap = 0.07 m ✓
    //
    // Battery is symmetric (bat_x/y = 0) and centered; passenger symmetric about both axes.
    parameters_.add({"pax_x",   id_, "m", "Passenger group x",  0.00, -0.25,  0.25,  0.00, true, 1.0});
    parameters_.add({"pax_y",   id_, "m", "Passenger group y",  0.00, -0.42,  0.42,  0.00, true, 1.0});
    parameters_.add({"pax_z",   id_, "m", "Passenger group z", -0.10, -0.50,  0.40, -0.10, true, 1.0});
    parameters_.add({"cargo_x", id_, "m", "Cargo x",            0.00, -0.50,  0.50,  0.00, true, 1.0});
    parameters_.add({"cargo_y", id_, "m", "Cargo y",            0.00, -0.45,  0.45,  0.00, true, 1.0});
    parameters_.add({"cargo_z", id_, "m", "Cargo z",            0.75,  0.30,  1.00,  0.75, true, 1.0});
    // Battery: centered and symmetric (bat_x = bat_y = 0). bat_z in ceiling zone.
    // Lower bound −1.00 keeps the slim slab (half_z ≤ 0.08 m) inside cabin (ceiling at z=−1.10):
    //   at bat_z = −1.00, slab top = −1.00 − 0.08 − 0.02(pad) = −1.10 = exactly cabin ceiling.
    // Optimizer may push bat_z positive (toward mid-cabin or floor) if energy constraints require.
    parameters_.add({"bat_x",   id_, "m", "Battery x",          0.00, -0.50,  0.50,  0.00, true, 1.0});
    parameters_.add({"bat_y",   id_, "m", "Battery y",          0.00, -0.55,  0.55,  0.00, true, 1.0});
    parameters_.add({"bat_z",   id_, "m", "Battery z",         -0.90, -1.00,  0.85, -0.90, true, 1.0});
}

void HexacopterArchitecture::registerDefaultConstraints() {
    constraints_.clear();

    constraints_.add({
        "parameter_bounds",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            double worst = 0.0;
            for (const auto* parameter : context.architecture.parameters().parameterPointers()) {
                if (parameter->value < parameter->lower_bound) {
                    worst = std::max(worst, parameter->lower_bound - parameter->value);
                }
                if (parameter->value > parameter->upper_bound) {
                    worst = std::max(worst, parameter->value - parameter->upper_bound);
                }
            }
            Constraint constraint{"parameter_bounds", context.architecture.id(), ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(worst);
        }
    });
    constraints_.add({
        "minimum_geometry_margin",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            const double value = std::min({
                context.architecture.Lx() - context.evaluation_context.minimum_arm_length,
                context.architecture.Lyi() - context.evaluation_context.minimum_arm_length,
                context.architecture.Lyo() - (context.architecture.Lyi() + context.evaluation_context.minimum_outer_arm_delta)
            });
            Constraint constraint{"minimum_geometry_margin", context.architecture.id(), ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(value);
        }
    });
    constraints_.add({
        "rotor_clearance",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"rotor_clearance", context.architecture.id(), ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(context.physical_model.packaging.minimum_rotor_clearance);
        }
    });
    constraints_.add({
        "packaging::payload_in_cabin",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"packaging::payload_in_cabin", context.architecture.id(), ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(context.physical_model.packaging.payload_containment_violation);
        }
    });
    constraints_.add({
        "packaging::battery_in_cabin",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"packaging::battery_in_cabin", context.architecture.id(), ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(context.physical_model.packaging.battery_containment_violation);
        }
    });
    constraints_.add({
        "packaging::battery_payload_nonoverlap",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"packaging::battery_payload_nonoverlap", context.architecture.id(), ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(context.physical_model.packaging.battery_payload_overlap);
        }
    });
    constraints_.add({
        "packaging::occupant_in_cabin",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"packaging::occupant_in_cabin", context.architecture.id(), ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(context.physical_model.packaging.occupant_containment_violation);
        }
    });
    constraints_.add({
        "packaging::payload_components_nonoverlap",
        id_,
        ConstraintSense::less_equal, 0.0, true, true, 1000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint c{"packaging::payload_components_nonoverlap",
                context.architecture.id(), ConstraintSense::less_equal, 0.0};
            return c.evaluate(context.physical_model.packaging.payload_internal_overlap);
        }
    });
    // CG envelope: both fore/aft and lateral offset must stay within prescribed bounds so that
    // hover trim authority is retained. Reads limits from EvaluationContext so they're tunable.
    constraints_.add({
        "cg_envelope",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        800.0,
        [](const ConstraintEvaluationContext& context) {
            const auto& cg = context.physical_model.mass_properties.center_of_mass;
            const auto& ec = context.evaluation_context;
            // Worst violation over both axes (positive = infeasible, 0 = on boundary).
            const double vx = std::abs(cg.x()) - ec.cg_envelope_half_x;
            const double vy = std::abs(cg.y()) - ec.cg_envelope_half_y;
            const double violation = std::max(vx, vy);
            Constraint constraint{"cg_envelope", context.architecture.id(), ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(violation);
        }
    });
    constraints_.add({
        "failed_hover_gamma",
        id_,
        ConstraintSense::greater_equal,
        1.5,
        true,
        true,
        2000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"failed_hover_gamma", context.architecture.id(), ConstraintSense::greater_equal, 1.5};
            return constraint.evaluate(context.stage1_metrics.gamma_worst);
        }
    });
    constraints_.add({
        "fault_allocation_ratio",
        id_,
        ConstraintSense::greater_equal,
        0.05,
        true,
        true,
        1500.0,
        [](const ConstraintEvaluationContext& context) {
            const double ratio = context.stage1_metrics.sigma_worst / std::max(context.stage1_metrics.sigma_reference, 1e-9);
            Constraint constraint{"fault_allocation_ratio", context.architecture.id(), ConstraintSense::greater_equal, 0.05};
            return constraint.evaluate(ratio);
        }
    });
    constraints_.add({
        "arm_yield_failure",
        id_,
        ConstraintSense::greater_equal,
        1.5,
        true,
        true,
        2000.0,
        [](const ConstraintEvaluationContext& context) {
            const double min_sf = context.physical_model.structural.min_safety_factor;
            const double threshold = context.evaluation_context.minimum_arm_safety_factor;
            Constraint constraint{"arm_yield_failure", context.architecture.id(), ConstraintSense::greater_equal, threshold};
            return constraint.evaluate(min_sf);
        }
    });

    // Phase 2: battery energy reserve — available energy must cover nominal + emergency mission.
    // reserve_fraction = (E_avail - E_req_total) / E_avail >= 0.
    constraints_.add({
        "battery_energy_reserve",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        2000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"battery_energy_reserve", context.architecture.id(),
                ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(context.stage1_metrics.bat_energy_reserve_fraction);
        }
    });

    // Phase 2: battery C-rate — peak current draw must not exceed pack continuous rating.
    // Violation value = C_rate / C_allow - 1; feasible when <= 0.
    constraints_.add({
        "battery_crate_limit",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1500.0,
        [](const ConstraintEvaluationContext& context) {
            const double ratio = context.stage1_metrics.bat_c_rate
                / std::max(context.evaluation_context.battery_crate_limit, 1e-9) - 1.0;
            Constraint constraint{"battery_crate_limit", context.architecture.id(),
                ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(ratio);
        }
    });

    // Phase 4: arm tip deflection — worst δ_tip over all members × load cases must not exceed limit.
    // Normalized violation = (δ - δ_allow) / δ_allow so the gradient is dimensionally consistent.
    constraints_.add({
        "arm_tip_deflection",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1500.0,
        [](const ConstraintEvaluationContext& context) {
            const double delta = context.stage1_metrics.struct_net_max_tip_deflection_m;
            const double limit = std::max(context.evaluation_context.arm_tip_deflection_limit_m, 1e-9);
            const double ratio = delta / limit - 1.0;
            Constraint constraint{"arm_tip_deflection", context.architecture.id(),
                ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(ratio);
        }
    });

    // Phase 4: arm tip rotation — worst θ_tip over all members × load cases must not exceed limit.
    constraints_.add({
        "arm_tip_rotation",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1500.0,
        [](const ConstraintEvaluationContext& context) {
            const double theta = context.stage1_metrics.struct_net_max_tip_rotation_rad;
            const double limit = std::max(context.evaluation_context.arm_tip_rotation_limit_rad, 1e-9);
            const double ratio = theta / limit - 1.0;
            Constraint constraint{"arm_tip_rotation", context.architecture.id(),
                ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(ratio);
        }
    });

    // ACS fault-hover feasibility: all single-fault hover trims must be achievable within T_max.
    // hover_margin = T_max / T_hover_worst - 1 >= 0 when all faults feasible.
    // Continuous violation (not binary) gives CMA-ES a gradient signal within the infeasible region:
    //   violation = max(0, -hover_margin) = max(0, T_hover_worst/T_max - 1) ∈ [0, 1].
    // Large penalty (20000) ensures the gradient dominates over objective terms near the boundary.
    constraints_.add({
        "all_faults_hover_feasible",
        "acs",
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        20000.0,
        [](const ConstraintEvaluationContext& context) {
            if (!context.evaluation_context.require_all_fault_acs_feasible) {
                return ConstraintEvaluation{1.0, 0.0, true};
            }
            // Cap at -1.0 so violation stays bounded when hover is completely infeasible.
            const double margin = std::isfinite(context.stage1_metrics.acs_hover_margin)
                ? context.stage1_metrics.acs_hover_margin
                : -1.0;
            Constraint constraint{"all_faults_hover_feasible", "acs",
                ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(margin);
        }
    });

    // ACS 4D directional margin: worst-fault minimum directional margin must exceed eps_acs_fault_margin.
    // m(d, u_req) = h_U(d) - d^T u_req for each of 11 sampled directions d and each of 6 fault cases.
    // acs_worst_fault_min_margin = min over faults of min over directions.
    // Normalized violation ∈ [0,1] so the penalty contribution is comparable to other constraints.
    // Penalty 5000 < 20000 (LP feasibility) so LP trim constraint takes precedence during search.
    constraints_.add({
        "fault_directional_margin",
        "acs",
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        5000.0,
        [](const ConstraintEvaluationContext& context) {
            const double margin = context.stage1_metrics.acs_worst_fault_min_margin;
            const double eps = context.evaluation_context.eps_acs_fault_margin;
            const double norm = std::max(eps, 1.0);
            const double violation = std::max(0.0, (eps - margin) / norm);
            return ConstraintEvaluation{margin, violation, margin >= eps};
        }
    });

}
// NOTE: acs::hover_slice_margin (sd >= ε) is NOT registered as a hard constraint.
// Reason: For motor-2 fault (at x=0, y=-Lyo), the equality constraints (thrust + yaw=0) enforce
//   T3+T5=mg/2 and T0+T1+T4=mg/2, which forces L = -Lyi*(T0-T1+T4) + Lyo*T3 + Lyi*T5 >= 0.
// The minimum achievable roll moment under motor-2 fault is exactly 0, regardless of T_max or
// arm geometry (as long as Lyi < Lyo). The origin (L=0,M=0) always lies on the polygon boundary.
// hover_slice_signed_distance = 0 for motor-2 fault is geometrically expected, not a defect.
// Use as an analysis metric only. Feasibility is enforced by acs::all_faults_hover_feasible.

void HexacopterArchitecture::registerElementConstraints() {
    for (const auto& element : elements_) {
        element->registerConstraints(constraints_);
    }
}

void HexacopterArchitecture::bindCanonicalParameters() {
    Lx_parameter_ = parameters_.find(id_ + "::Lx");
    Lyi_parameter_ = parameters_.find(id_ + "::Lyi");
    Lyo_parameter_ = parameters_.find(id_ + "::Lyo");
    Tmax_parameter_ = parameters_.find(id_ + "::T_max");
    cT_parameter_ = parameters_.find(id_ + "::cT");
    dprop_parameter_ = parameters_.find(id_ + "::d_prop");
    r_o_parameter_ = parameters_.find(id_ + "::arm_outer_radius");
    t_wall_parameter_ = parameters_.find(id_ + "::arm_wall_thickness");
    mbat_parameter_         = parameters_.find(id_ + "::m_bat");
    m_pax_parameter_        = parameters_.find(id_ + "::m_pax");
    m_cargo_parameter_      = parameters_.find(id_ + "::m_cargo");
    m_instrument_parameter_ = parameters_.find(id_ + "::m_instrument");
    pax_x_parameter_        = parameters_.find(id_ + "::pax_x");
    pax_y_parameter_        = parameters_.find(id_ + "::pax_y");
    pax_z_parameter_        = parameters_.find(id_ + "::pax_z");
    cargo_x_parameter_      = parameters_.find(id_ + "::cargo_x");
    cargo_y_parameter_      = parameters_.find(id_ + "::cargo_y");
    cargo_z_parameter_      = parameters_.find(id_ + "::cargo_z");
    bat_x_parameter_        = parameters_.find(id_ + "::bat_x");
    bat_y_parameter_        = parameters_.find(id_ + "::bat_y");
    bat_z_parameter_        = parameters_.find(id_ + "::bat_z");
}

void HexacopterArchitecture::rebuildElements() {
    elements_ = DefaultHexacopterBuilder::buildElements({
        Lx_parameter_,
        Lyi_parameter_,
        Lyo_parameter_,
        Tmax_parameter_,
        cT_parameter_,
        dprop_parameter_,
        m_pax_parameter_,
        m_cargo_parameter_,
        m_instrument_parameter_,
        r_o_parameter_,
        t_wall_parameter_,
        mbat_parameter_,
        pax_x_parameter_,
        pax_y_parameter_,
        pax_z_parameter_,
        cargo_x_parameter_,
        cargo_y_parameter_,
        cargo_z_parameter_,
        bat_x_parameter_,
        bat_y_parameter_,
        bat_z_parameter_
    });

    // CabinEnvelopeElement and OccupantEnvelopeElement inserted before passenger element.
    const auto passenger_it = std::find_if(elements_.begin(), elements_.end(),
        [](const SpatialElementPtr& e) { return e->id() == "passenger"; });
    elements_.insert(passenger_it, std::make_unique<CabinEnvelopeElement>("cabin_envelope"));
    elements_.insert(
        std::find_if(elements_.begin(), elements_.end(),
            [](const SpatialElementPtr& e) { return e->id() == "passenger"; }),
        std::make_unique<OccupantEnvelopeElement>("occupant_envelope"));

    for (auto& element : elements_) {
        element->registerParameters(parameters_);
        if (auto* rotor = dynamic_cast<RotorElement*>(element.get())) {
            rotor->registerThrustMax(Tmax_parameter_);
        }
        element->updateFromParameters();
    }
}

void HexacopterArchitecture::rebuildAttachments() {
    attachments_ = DefaultHexacopterBuilder::buildAttachments({
        Lx_parameter_,
        Lyi_parameter_,
        Lyo_parameter_,
        Tmax_parameter_,
        cT_parameter_,
        dprop_parameter_,
        m_pax_parameter_,
        m_cargo_parameter_,
        m_instrument_parameter_,
        r_o_parameter_,
        t_wall_parameter_,
        mbat_parameter_,
        pax_x_parameter_,
        pax_y_parameter_,
        pax_z_parameter_,
        cargo_x_parameter_,
        cargo_y_parameter_,
        cargo_z_parameter_,
        bat_x_parameter_,
        bat_y_parameter_,
        bat_z_parameter_
    });

    // Cabin envelope centred inside body hull (body "center" anchor = body origin).
    attachments_.push_back({"body", "cabin_envelope", "center", "center",
        AttachmentRelationship::localOffset({0.0, 0.0, 0.0}),
        true, "", AttachmentContactPolicy::bonded_overlap, nullptr});
    // Occupant envelope centred inside cabin (same origin — occupant space is a subset of cabin).
    attachments_.push_back({"cabin_envelope", "occupant_envelope", "center", "center",
        AttachmentRelationship::localOffset({0.0, 0.0, 0.0}),
        true, "", AttachmentContactPolicy::bonded_overlap, nullptr});
}

void HexacopterArchitecture::assemble() {
    assembly_state_.elements.clear();
    packaging_exemptions_.clear();

    std::unordered_map<std::string, const SpatialElement*> element_by_id;
    for (const auto& element : elements_) {
        element_by_id.emplace(element->id(), element.get());
    }

    std::unordered_map<std::string, std::vector<const Attachment*>> children;
    std::unordered_map<std::string, std::size_t> parent_count;
    std::unordered_map<std::string, const Attachment*> attachment_by_child;
    std::unordered_map<std::string, std::size_t> symmetry_tag_counts;
    for (const auto& attachment : attachments_) {
        if (attachment.enabled && !attachment.symmetry_tag.empty()) {
            symmetry_tag_counts[attachment.symmetry_tag] += 1U;
        }
    }
    for (const auto& [tag, count] : symmetry_tag_counts) {
        if (count % 2U != 0U) {
            std::cerr << "[HexacopterArchitecture] symmetry_tag '" << tag
                      << "' has odd count " << count << " (expected even)\n";
        }
    }

    for (const auto& attachment : attachments_) {
        if (!attachment.enabled) {
            continue;
        }
        if (!attachment.parent_id.empty() && !element_by_id.contains(attachment.parent_id)) {
            throw std::runtime_error("Attachment parent not found: " + attachment.parent_id);
        }
        if (!element_by_id.contains(attachment.child_id)) {
            throw std::runtime_error("Attachment child not found: " + attachment.child_id);
        }
        parent_count[attachment.child_id] += 1U;
        if (parent_count[attachment.child_id] > 1U) {
            throw std::runtime_error("Attachment child has multiple parents: " + attachment.child_id);
        }
        children[attachment.parent_id].push_back(&attachment);
        attachment_by_child[attachment.child_id] = &attachment;
    }

    for (const auto& [child_id, attachment] : attachment_by_child) {
        if (attachment == nullptr || attachment->contact_policy != AttachmentContactPolicy::bonded_overlap) {
            continue;
        }

        std::string descendant = child_id;
        std::string cursor = child_id;
        while (true) {
            const auto it = attachment_by_child.find(cursor);
            if (it == attachment_by_child.end() || it->second == nullptr) {
                break;
            }
            const Attachment& bonded = *it->second;
            if (bonded.contact_policy != AttachmentContactPolicy::bonded_overlap || bonded.parent_id.empty()) {
                break;
            }
            packaging_exemptions_.insert(packagingPairKey(descendant, bonded.parent_id));
            cursor = bonded.parent_id;
        }
    }

    for (const auto& [child_id, attachment] : attachment_by_child) {
        if (attachment == nullptr || attachment->contact_policy != AttachmentContactPolicy::allow_touch) {
            continue;
        }
        if (!attachment->parent_id.empty()) {
            packaging_exemptions_.insert(packagingPairKey(child_id, attachment->parent_id));
        }
    }

    std::unordered_set<std::string> visiting;
    std::unordered_set<std::string> visited;
    std::function<void(const std::string&, const Eigen::Isometry3d&)> visit =
        [&](const std::string& parent_id, const Eigen::Isometry3d& parent_pose) {
            if (!parent_id.empty()) {
                if (visiting.contains(parent_id)) {
                    throw std::runtime_error("Attachment cycle detected at element: " + parent_id);
                }
                if (visited.contains(parent_id)) {
                    return;
                }
                visiting.insert(parent_id);
            }

            const auto child_it = children.find(parent_id);
            if (child_it == children.end()) {
                if (!parent_id.empty()) {
                    visiting.erase(parent_id);
                    visited.insert(parent_id);
                }
                return;
            }

            for (const auto* attachment : child_it->second) {
                const auto element_it = element_by_id.find(attachment->child_id);
                if (element_it == element_by_id.end()) {
                    continue;
                }

                const Eigen::Isometry3d relative_pose = attachment->relative_transform
                    ? attachment->relative_transform(*this)
                    : attachment->relationship.resolve();
                Eigen::Isometry3d parent_anchor_pose = parent_pose;
                if (!attachment->parent_id.empty() && !attachment->parent_anchor.empty()) {
                    const auto parent_element_it = element_by_id.find(attachment->parent_id);
                    if (parent_element_it == element_by_id.end()) {
                        throw std::runtime_error("Attachment parent missing during assembly: " + attachment->parent_id);
                    }
                    const auto local_anchor = parent_element_it->second->anchorPose(attachment->parent_anchor);
                    if (!local_anchor.has_value()) {
                        throw std::runtime_error(
                            "Missing parent anchor '" + attachment->parent_anchor + "' on element " + attachment->parent_id);
                    }
                    parent_anchor_pose = parent_pose * local_anchor.value();
                }

                Eigen::Isometry3d child_anchor_pose = Eigen::Isometry3d::Identity();
                if (!attachment->child_anchor.empty()) {
                    const auto local_anchor = element_it->second->anchorPose(attachment->child_anchor);
                    if (!local_anchor.has_value()) {
                        throw std::runtime_error(
                            "Missing child anchor '" + attachment->child_anchor + "' on element " + attachment->child_id);
                    }
                    child_anchor_pose = local_anchor.value();
                }

                const Eigen::Isometry3d world_pose =
                    parent_anchor_pose * relative_pose * child_anchor_pose.inverse() * element_it->second->localPose();

                AssembledElement assembled;
                assembled.element = element_it->second;
                assembled.world_pose = world_pose;
                assembled.local_primitives = element_it->second->localPrimitives();
                assembly_state_.elements.push_back(std::move(assembled));

                visit(attachment->child_id, world_pose);
            }

            if (!parent_id.empty()) {
                visiting.erase(parent_id);
                visited.insert(parent_id);
            }
        };

    visit("", Eigen::Isometry3d::Identity());

    for (const auto& [element_id, element] : element_by_id) {
        if (!visited.contains(element_id)) {
            throw std::runtime_error("Orphaned element not reachable from root: " + element_id);
        }
    }
}

}  // namespace hexaarch::core
