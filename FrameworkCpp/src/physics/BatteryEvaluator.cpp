#include "physics/BatteryEvaluator.hpp"

#include <algorithm>
#include <cmath>

#include "evaluation/EvaluationContext.hpp"
#include "mission/MissionEvaluator.hpp"

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

BatteryResult BatteryEvaluator::evaluateWithMission(
    const PowertrainResult& powertrain,
    const mission::MissionResult& mission_result,
    const double battery_mass_kg,
    const double total_vehicle_mass_kg,
    const evaluation::EvaluationContext& context) const {

    BatteryResult result;

    result.available_energy_wh =
        context.battery_pack_efficiency
        * context.battery_dod_usable
        * battery_mass_kg
        * context.battery_specific_energy_wh_per_kg;

    // Mission energy already includes auxiliary draw via MissionEvaluator.
    result.required_energy_total_wh = mission_result.total_energy_with_aux_wh;

    // Split nominal vs. emergency for backwards-compatible reporting: hover/cruise legs feed the
    // nominal bucket, emergency_hover legs feed the fault bucket. reserve_hover is also nominal.
    double nominal = 0.0;
    double fault = 0.0;
    for (const auto& seg : mission_result.segments) {
        if (seg.kind == mission::SegmentKind::emergency_hover) {
            fault += seg.energy_wh;
        } else {
            nominal += seg.energy_wh;
        }
    }
    // Distribute auxiliary draw proportionally to time (already lumped in total_with_aux).
    const double aux_wh = mission_result.total_energy_with_aux_wh - mission_result.total_energy_wh;
    result.required_energy_nominal_wh = nominal + aux_wh;
    result.required_energy_fault_wh   = fault;

    const double E_avail = std::max(result.available_energy_wh, 1e-9);
    result.energy_reserve_fraction =
        (result.available_energy_wh - result.required_energy_total_wh) / E_avail;

    // Peak C-rate uses the worst per-segment electrical power (still adds aux draw).
    const double P_peak = mission_result.peak_power_w + context.power_auxiliary_w;
    result.c_rate = P_peak / E_avail;

    result.mass_fraction = battery_mass_kg / std::max(total_vehicle_mass_kg, 1e-9);

    return result;
}

}  // namespace hexaarch::physics
