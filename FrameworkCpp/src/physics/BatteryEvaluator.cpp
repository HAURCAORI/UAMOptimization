#include "physics/BatteryEvaluator.hpp"

#include <algorithm>
#include <cmath>

#include "evaluation/EvaluationContext.hpp"

namespace hexaarch::physics {

BatteryResult BatteryEvaluator::evaluate(
    const PowertrainResult& powertrain,
    const double battery_mass_kg,
    const double total_vehicle_mass_kg,
    const evaluation::EvaluationContext& context) const {

    BatteryResult result;

    // Available energy [Wh] = eta_pack * DoD * m_bat * e_spec
    result.available_energy_wh =
        context.battery_pack_efficiency
        * context.battery_dod_usable
        * battery_mass_kg
        * context.battery_specific_energy_wh_per_kg;

    // Mission energy requirements [Wh].
    // Auxiliary power is shared across both phases.
    const double t_nom_h = context.mission_time_nominal_min / 60.0;
    const double t_emg_h = context.mission_time_emergency_min / 60.0;
    const double P_nom_w = powertrain.total_power_nominal_w + context.power_auxiliary_w;
    const double P_fault_w = powertrain.total_power_faulted_w + context.power_auxiliary_w;

    result.required_energy_nominal_wh = P_nom_w * t_nom_h;
    result.required_energy_fault_wh = P_fault_w * t_emg_h;
    result.required_energy_total_wh =
        result.required_energy_nominal_wh + result.required_energy_fault_wh;

    // Reserve fraction: positive = feasible, negative = deficit.
    const double E_avail = std::max(result.available_energy_wh, 1e-9);
    result.energy_reserve_fraction =
        (result.available_energy_wh - result.required_energy_total_wh) / E_avail;

    // C-rate = P_peak / E_avail. Voltage cancels: C = (P/V) / (E/V) = P [W] / E [Wh] [h^-1].
    const double P_peak_w = std::max(
        powertrain.total_power_nominal_w, powertrain.total_power_faulted_w)
        + context.power_auxiliary_w;
    result.c_rate = P_peak_w / std::max(result.available_energy_wh, 1e-9);

    result.mass_fraction =
        battery_mass_kg / std::max(total_vehicle_mass_kg, 1e-9);

    return result;
}

}  // namespace hexaarch::physics
