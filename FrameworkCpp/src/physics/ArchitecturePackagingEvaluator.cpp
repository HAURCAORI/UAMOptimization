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

    // --- Payload containment inside cabin envelope ---
    const core::AssembledElement* cabin_el   = nullptr;
    const core::AssembledElement* payload_el = nullptr;
    const core::AssembledElement* battery_el = nullptr;
    const core::AssembledElement* occupant_el = nullptr;
    for (const auto& assembled : architecture.assemblyState().elements) {
        if (assembled.element == nullptr) continue;
        const auto& t = assembled.element->type();
        if (t == "CabinEnvelopeElement")    cabin_el    = &assembled;
        if (t == "PayloadElement")          payload_el  = &assembled;
        if (t == "BatteryElement")          battery_el  = &assembled;
        if (t == "OccupantEnvelopeElement") occupant_el = &assembled;
    }

    auto cabinWorld = [&]() -> std::optional<core::LocalAABB> {
        if (cabin_el == nullptr) return std::nullopt;
        const auto* p = dynamic_cast<const core::IEnvelopeProvider*>(cabin_el->element);
        if (p == nullptr) return std::nullopt;
        return worldAABB(p->localEnvelope(), cabin_el->world_pose);
    };

    if (cabin_el != nullptr && payload_el != nullptr) {
        const auto* payload_prov = dynamic_cast<const core::IEnvelopeProvider*>(payload_el->element);
        if (const auto cw = cabinWorld(); cw.has_value() && payload_prov != nullptr) {
            const core::LocalAABB payload_world = worldAABB(payload_prov->localEnvelope(), payload_el->world_pose);
            model.packaging.payload_containment_violation = cw->containmentViolation(payload_world);
        }
    }

    if (cabin_el != nullptr && battery_el != nullptr) {
        const auto* bat_prov = dynamic_cast<const core::IEnvelopeProvider*>(battery_el->element);
        if (const auto cw = cabinWorld(); cw.has_value() && bat_prov != nullptr) {
            const core::LocalAABB bat_world = worldAABB(bat_prov->localEnvelope(), battery_el->world_pose);
            model.packaging.battery_containment_violation = cw->containmentViolation(bat_world);
        }
    }

    if (payload_el != nullptr && battery_el != nullptr) {
        const auto* payload_prov = dynamic_cast<const core::IEnvelopeProvider*>(payload_el->element);
        const auto* bat_prov     = dynamic_cast<const core::IEnvelopeProvider*>(battery_el->element);
        if (payload_prov != nullptr && bat_prov != nullptr) {
            const core::LocalAABB payload_world = worldAABB(payload_prov->localEnvelope(), payload_el->world_pose);
            const core::LocalAABB bat_world     = worldAABB(bat_prov->localEnvelope(),     battery_el->world_pose);
            model.packaging.battery_payload_overlap = payload_world.overlapMagnitude(bat_world);
        }
    }

    if (cabin_el != nullptr && occupant_el != nullptr) {
        const auto* occup_prov = dynamic_cast<const core::IEnvelopeProvider*>(occupant_el->element);
        if (const auto cw = cabinWorld(); cw.has_value() && occup_prov != nullptr) {
            const core::LocalAABB occup_world = worldAABB(occup_prov->localEnvelope(), occupant_el->world_pose);
            model.packaging.occupant_containment_violation = cw->containmentViolation(occup_world);
        }
    }

    // Rotor keep-out: occupant envelope, payload, and battery must not intrude into any rotor's
    // swept keep-out cylinder. worldAABB (translation-only) is valid because the cylinder is
    // rotationally symmetric in XY — its AABB depends only on world_pose translation.
    // Arm/motor/rotor are exempt via bonded_overlap attachment; only internal placeable elements
    // (occupant, payload, battery) are checked.
    double worst_keepout = 0.0;
    std::string worst_zone_id;
    std::string worst_element_id;

    auto checkKeepout = [&](const core::LocalAABB& ko_world,
                             const std::string& zone_id,
                             const core::AssembledElement* el_p) {
        if (el_p == nullptr || el_p->element == nullptr) return;
        const auto* prov = dynamic_cast<const core::IEnvelopeProvider*>(el_p->element);
        if (prov == nullptr) return;
        const core::LocalAABB el_world = worldAABB(prov->localEnvelope(), el_p->world_pose);
        const double intrusion = ko_world.overlapMagnitude(el_world);
        if (intrusion > worst_keepout) {
            worst_keepout   = intrusion;
            worst_zone_id   = zone_id;
            worst_element_id = el_p->element->id();
        }
    };

    for (const auto& assembled : architecture.assemblyState().elements) {
        if (assembled.element == nullptr || assembled.element->type() != "KeepOutZoneElement") continue;
        const auto* ko_prov = dynamic_cast<const core::IEnvelopeProvider*>(assembled.element);
        if (ko_prov == nullptr) continue;
        const core::LocalAABB ko_world = worldAABB(ko_prov->localEnvelope(), assembled.world_pose);
        const std::string zone_id = assembled.element->id();
        checkKeepout(ko_world, zone_id, battery_el);
        checkKeepout(ko_world, zone_id, payload_el);
        checkKeepout(ko_world, zone_id, occupant_el);
    }
    model.packaging.rotor_keepout_intrusion_m      = worst_keepout;
    model.packaging.rotor_keepout_offending_zone_id    = std::move(worst_zone_id);
    model.packaging.rotor_keepout_offending_element_id = std::move(worst_element_id);
}

}  // namespace hexaarch::physics
