#include "physics/ArchitecturePackagingEvaluator.hpp"

#include <limits>
#include <optional>

#include "core/ElementCapabilities.hpp"
#include "physics/PrimitiveDistance.hpp"

namespace hexaarch::physics {
namespace {

core::LocalAABB worldAABB(const core::LocalAABB& local, const Eigen::Isometry3d& world_pose) {
    // Valid for identity-rotation elements (pure translation in assembly).
    const Eigen::Vector3d t = world_pose.translation();
    return {local.min_corner + t, local.max_corner + t};
}

}  // namespace

void ArchitecturePackagingEvaluator::analyze(
    PhysicalModel& model,
    const core::HexacopterArchitecture& architecture) const {

    // Rotor-to-rotor disk clearance — the only physically meaningful packaging
    // constraint at current geometry fidelity. Arm/body/battery primitives are
    // placeholder-scale and not calibrated for containment checks (see Step 3).
    std::vector<const core::AssembledElement*> rotor_assembled;
    for (const auto& assembled : architecture.assemblyState().elements) {
        if (assembled.element != nullptr &&
            dynamic_cast<const core::IPropulsionRotor*>(assembled.element) != nullptr) {
            rotor_assembled.push_back(&assembled);
        }
    }

    double min_rotor_clearance = rotor_assembled.empty()
        ? 0.0
        : (std::numeric_limits<double>::max)();

    for (std::size_t i = 0; i < rotor_assembled.size(); ++i) {
        const auto& lhs = *rotor_assembled.at(i);
        for (std::size_t j = i + 1; j < rotor_assembled.size(); ++j) {
            const auto& rhs = *rotor_assembled.at(j);
            for (const auto& lhs_prim : lhs.local_primitives) {
                for (const auto& rhs_prim : rhs.local_primitives) {
                    const double cl = primitiveClearance(
                        lhs_prim, lhs.world_pose,
                        rhs_prim, rhs.world_pose);
                    min_rotor_clearance = std::min(min_rotor_clearance, cl);
                }
            }
        }
    }

    model.packaging.minimum_rotor_clearance = min_rotor_clearance;
    model.packaging.valid = min_rotor_clearance >= 0.0;
    model.packaging.overlap_penalty = min_rotor_clearance >= 0.0
        ? 0.0
        : -min_rotor_clearance / std::max(architecture.propellerDiameter(), 1e-9);

    // Find envelope elements by type
    const core::AssembledElement* cabin_el     = nullptr;
    const core::AssembledElement* passenger_el = nullptr;
    const core::AssembledElement* cargo_el     = nullptr;
    const core::AssembledElement* instrument_el = nullptr;
    const core::AssembledElement* battery_el   = nullptr;
    const core::AssembledElement* occupant_el  = nullptr;
    for (const auto& assembled : architecture.assemblyState().elements) {
        if (assembled.element == nullptr) continue;
        const auto& t = assembled.element->type();
        if (t == "CabinEnvelopeElement")    cabin_el     = &assembled;
        if (t == "PassengerElement")        passenger_el = &assembled;
        if (t == "CargoElement")            cargo_el     = &assembled;
        if (t == "InstrumentPanelElement")  instrument_el = &assembled;
        if (t == "BatteryElement")          battery_el   = &assembled;
        if (t == "OccupantEnvelopeElement") occupant_el  = &assembled;
    }

    auto getWorldAABB = [&](const core::AssembledElement* el) -> std::optional<core::LocalAABB> {
        if (el == nullptr) return std::nullopt;
        const auto* p = dynamic_cast<const core::IEnvelopeProvider*>(el->element);
        if (p == nullptr) return std::nullopt;
        return worldAABB(p->localEnvelope(), el->world_pose);
    };

    const auto cabin_world = getWorldAABB(cabin_el);
    const auto pax_world   = getWorldAABB(passenger_el);
    const auto cargo_world = getWorldAABB(cargo_el);
    const auto inst_world  = getWorldAABB(instrument_el);
    const auto bat_world   = getWorldAABB(battery_el);
    const auto occ_world   = getWorldAABB(occupant_el);

    // Containment: all payload components must fit inside cabin
    double payload_containment = 0.0;
    if (cabin_world.has_value()) {
        if (pax_world.has_value())  payload_containment = std::max(payload_containment, cabin_world->containmentViolation(*pax_world));
        if (cargo_world.has_value()) payload_containment = std::max(payload_containment, cabin_world->containmentViolation(*cargo_world));
        if (inst_world.has_value()) payload_containment = std::max(payload_containment, cabin_world->containmentViolation(*inst_world));
    }
    model.packaging.payload_containment_violation = payload_containment;

    // Battery containment
    if (cabin_world.has_value() && bat_world.has_value())
        model.packaging.battery_containment_violation = cabin_world->containmentViolation(*bat_world);

    // Occupant containment
    if (cabin_world.has_value() && occ_world.has_value())
        model.packaging.occupant_containment_violation = cabin_world->containmentViolation(*occ_world);

    // Battery vs payload overlap (max over all payload components)
    double bat_payload_overlap = 0.0;
    if (bat_world.has_value()) {
        if (pax_world.has_value())   bat_payload_overlap = std::max(bat_payload_overlap, bat_world->overlapMagnitude(*pax_world));
        if (cargo_world.has_value()) bat_payload_overlap = std::max(bat_payload_overlap, bat_world->overlapMagnitude(*cargo_world));
        if (inst_world.has_value())  bat_payload_overlap = std::max(bat_payload_overlap, bat_world->overlapMagnitude(*inst_world));
    }
    model.packaging.battery_payload_overlap = bat_payload_overlap;

    // Payload internal overlap (passenger vs cargo vs instrument)
    double internal_overlap = 0.0;
    if (pax_world.has_value() && cargo_world.has_value())
        internal_overlap = std::max(internal_overlap, pax_world->overlapMagnitude(*cargo_world));
    if (pax_world.has_value() && inst_world.has_value())
        internal_overlap = std::max(internal_overlap, pax_world->overlapMagnitude(*inst_world));
    if (cargo_world.has_value() && inst_world.has_value())
        internal_overlap = std::max(internal_overlap, cargo_world->overlapMagnitude(*inst_world));
    model.packaging.payload_internal_overlap = internal_overlap;

    // Zero out keepout fields (no longer checked)
    model.packaging.rotor_keepout_intrusion_m = 0.0;
    model.packaging.rotor_keepout_offending_zone_id.clear();
    model.packaging.rotor_keepout_offending_element_id.clear();
}

}  // namespace hexaarch::physics
