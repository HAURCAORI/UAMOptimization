#pragma once

#include "physics/PhysicsTypes.hpp"

namespace hexaarch::evaluation {
struct EvaluationContext;
}

namespace hexaarch::mission {
struct MissionResult;
}

namespace hexaarch::physics {

// Computes battery energy budget from powertrain result and battery design parameters.
// Legacy mode (no mission_profile in context):
//   E_avail = eta_pack * DoD * m_bat * e_spec [Wh]
//   E_req   = P_nom * t_nom + P_fault * t_emg + P_aux * (t_nom + t_emg)  [Wh]
// Mission mode (mission_profile set): uses the precomputed MissionResult instead of the two
// fixed-time hover legs above. Auxiliary power is included via the MissionResult.
class BatteryEvaluator {
public:
    [[nodiscard]] BatteryResult evaluate(
        const PowertrainResult& powertrain,
        double battery_mass_kg,
        double total_vehicle_mass_kg,
        const evaluation::EvaluationContext& context) const;

    [[nodiscard]] BatteryResult evaluateWithMission(
        const PowertrainResult& powertrain,
        const mission::MissionResult& mission_result,
        double battery_mass_kg,
        double total_vehicle_mass_kg,
        const evaluation::EvaluationContext& context) const;
};

}  // namespace hexaarch::physics
