#pragma once

#include "physics/PhysicsTypes.hpp"

namespace hexaarch::evaluation {
struct EvaluationContext;
}

namespace hexaarch::physics {

// Computes battery energy budget from powertrain result and battery design parameters.
// E_avail = eta_pack * DoD * m_bat * e_spec [Wh]
// E_req   = P_nom * t_nom + P_fault * t_emg + P_aux * (t_nom + t_emg)  [Wh]
class BatteryEvaluator {
public:
    [[nodiscard]] BatteryResult evaluate(
        const PowertrainResult& powertrain,
        double battery_mass_kg,
        double total_vehicle_mass_kg,
        const evaluation::EvaluationContext& context) const;
};

}  // namespace hexaarch::physics
