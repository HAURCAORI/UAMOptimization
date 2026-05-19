#include "physics/PowertrainEvaluator.hpp"

#include <algorithm>
#include <cmath>

#include "evaluation/EvaluationContext.hpp"

namespace hexaarch::physics {

PowertrainResult PowertrainEvaluator::evaluate(
    const std::array<double, kNumRotors>& trim_thrust_nominal,
    const std::array<double, kNumRotors>& trim_thrust_faulted,
    const double thrust_max,
    const double max_arm_length,
    const evaluation::EvaluationContext& context) const {

    constexpr double kPi = 3.14159265358979323846;

    // Effective rotor disk area derived from arm geometry (not d_prop).
    // d_prop is frozen for cT/yaw-torque purposes only; power uses arm-tip radius.
    const double r_eff = std::max(max_arm_length * 0.5, 0.1);
    const double A_eff = kPi * r_eff * r_eff;

    // Combined efficiency denominator: figure-of-merit × motor × ESC.
    const double eta_total = std::max(
        context.figure_of_merit * context.motor_efficiency * context.esc_efficiency, 1e-6);
    const double denom = eta_total * std::sqrt(2.0 * context.air_density * A_eff);

    // Continuous power capacity = electrical power at T_max [W].
    const double p_cont = std::pow(std::max(thrust_max, 1e-9), 1.5) / std::max(denom, 1e-9);

    PowertrainResult result;
    result.total_power_nominal_w = 0.0;
    result.worst_thrust_utilization = 0.0;
    result.worst_power_utilization = 0.0;

    for (int i = 0; i < kNumRotors; ++i) {
        const double T_i = std::max(trim_thrust_nominal.at(static_cast<std::size_t>(i)), 0.0);
        const double P_i = std::pow(T_i, 1.5) / std::max(denom, 1e-9);
        result.motor_power_nominal.at(static_cast<std::size_t>(i)) = P_i;
        result.total_power_nominal_w += P_i;
        result.worst_thrust_utilization = std::max(
            result.worst_thrust_utilization, T_i / std::max(thrust_max, 1e-9));
        result.worst_power_utilization = std::max(
            result.worst_power_utilization, P_i / std::max(p_cont, 1e-9));
    }

    result.total_power_faulted_w = 0.0;
    for (int i = 0; i < kNumRotors; ++i) {
        const double T_i = std::max(trim_thrust_faulted.at(static_cast<std::size_t>(i)), 0.0);
        const double P_i = std::pow(T_i, 1.5) / std::max(denom, 1e-9);
        result.motor_power_faulted.at(static_cast<std::size_t>(i)) = P_i;
        result.total_power_faulted_w += P_i;
    }

    return result;
}

}  // namespace hexaarch::physics
