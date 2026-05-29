#include "mission/CruisePowerModel.hpp"

#include <algorithm>
#include <cmath>

#include "evaluation/EvaluationContext.hpp"

namespace hexaarch::mission {

double CruisePowerModel::inducedVelocityForwardFlight(
    const double thrust_n,
    const double disk_area_m2,
    const double airspeed_mps,
    const double climb_rate_mps,
    const double air_density_kg_m3) {

    const double T = std::max(thrust_n, 0.0);
    const double A = std::max(disk_area_m2, 1e-6);
    const double rho = std::max(air_density_kg_m3, 1e-3);
    const double Vh = std::max(airspeed_mps, 0.0);
    const double Vv = climb_rate_mps;

    // Hover induced velocity v_h = sqrt(T / (2 ρ A)).
    const double v_h = std::sqrt(T / (2.0 * rho * A));

    // Solve f(v_i) = v_i · sqrt(Vh² + (Vv + v_i)²) − v_h² = 0 by Newton iteration.
    // Initial guess: hover v_i when no airspeed; momentum-theory v_i = v_h²/Vh at high speed.
    double v_i = (Vh > v_h) ? (v_h * v_h / Vh) : v_h;
    for (int iter = 0; iter < 32; ++iter) {
        const double s = std::sqrt(Vh * Vh + (Vv + v_i) * (Vv + v_i));
        const double f = v_i * s - v_h * v_h;
        const double df_dv = s + v_i * (Vv + v_i) / std::max(s, 1e-9);
        const double step = f / std::max(df_dv, 1e-9);
        v_i -= step;
        if (v_i < 0.0) v_i = 0.5 * v_h;
        if (std::abs(step) < 1e-6) break;
    }
    return std::max(v_i, 0.0);
}

CruiseResult CruisePowerModel::evaluate(
    const CruiseQuery& query,
    const evaluation::EvaluationContext& context) const {

    constexpr double kGravity = 9.81;
    CruiseResult result;

    const double Vh = std::max(query.airspeed_mps, 0.0);
    const double Vv = query.climb_rate_mps;
    const double rho = std::max(query.air_density_kg_m3, 1e-3);
    const double m = std::max(query.vehicle_mass_kg, 0.0);
    const double W = m * kGravity;

    // Parasite drag from equivalent flat-plate area.
    const double drag_n = 0.5 * rho * Vh * Vh * std::max(query.parasite_cd_a_m2, 0.0);

    // Quasi-steady force balance: T cos(θ) = W + 0  (vertical), T sin(θ) = D (horizontal).
    // Climb adds m·g·v_z to the propulsive demand.
    result.thrust_required_n = std::sqrt(W * W + drag_n * drag_n);

    const double v_i = inducedVelocityForwardFlight(
        result.thrust_required_n,
        query.disk_area_m2,
        Vh,
        Vv,
        rho);

    // Mechanical powers at the rotor disks.
    result.induced_power_w  = result.thrust_required_n * v_i;
    result.parasite_power_w = drag_n * Vh;
    result.climb_power_w    = W * std::max(Vv, 0.0);  // descent recovery is not modelled
    result.mechanical_power_w =
        result.induced_power_w + result.parasite_power_w + result.climb_power_w;

    const double eta_total = std::max(
        context.figure_of_merit * context.motor_efficiency * context.esc_efficiency, 1e-6);
    result.electrical_power_w = result.mechanical_power_w / eta_total;

    return result;
}

}  // namespace hexaarch::mission
