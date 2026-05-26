#include "physics/StructuralNetworkAnalyzer.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "core/ElementCapabilities.hpp"
#include "core/HexacopterArchitecture.hpp"
#include "evaluation/EvaluationContext.hpp"

namespace hexaarch::physics {

void StructuralNetworkAnalyzer::analyze(
    PhysicalModel& model,
    const core::HexacopterArchitecture& arch,
    const evaluation::EvaluationContext& context,
    const std::array<double, kNumRotors>& nominal_thrust,
    const std::vector<std::array<double, kNumRotors>>& fault_thrusts) const {

    const double cT = arch.cT();
    const double g = model.propulsion.gravity;
    const double E = context.arm_material.elastic_modulus;
    const double yield_strength = context.arm_material.yield_strength;

    model.network_structural.clear();
    model.arm_structural.clear();

    // Build load cases: max_thrust, nominal_hover, fault_0 .. fault_N-1.
    std::vector<std::pair<std::string, std::array<double, kNumRotors>>> load_cases;
    load_cases.reserve(2U + fault_thrusts.size());

    std::array<double, kNumRotors> max_thrust_arr{};
    max_thrust_arr.fill(model.propulsion.thrust_max);
    load_cases.push_back({"max_thrust", max_thrust_arr});
    load_cases.push_back({"nominal_hover", nominal_thrust});
    for (std::size_t fi = 0; fi < fault_thrusts.size(); ++fi) {
        load_cases.push_back({"fault_" + std::to_string(fi), fault_thrusts[fi]});
    }

    double min_sf = (std::numeric_limits<double>::max)();
    double max_delta = 0.0;
    double max_theta = 0.0;
    double max_sigma_vm = 0.0;

    for (const auto& elem : arch.elements()) {
        auto* beam = dynamic_cast<core::IStructuralBeam*>(elem.get());
        if (!beam) {
            continue;
        }

        const std::string& arm_id = elem->id();

        // Parse arm index from ID (format "arm_N", 1-based → 0-based motor column).
        const auto underscore = arm_id.rfind('_');
        if (underscore == std::string::npos) {
            continue;
        }
        int arm_number = 0;
        try {
            arm_number = std::stoi(arm_id.substr(underscore + 1));
        } catch (...) {
            continue;
        }
        const int motor_idx = arm_number - 1;
        if (motor_idx < 0 || motor_idx >= kNumRotors) {
            continue;
        }

        // Find attached motor element for its mass.
        const std::string motor_id = "motor" + arm_id.substr(3);
        const auto* motor_elem = arch.findElement(motor_id);
        const double m_motor = motor_elem ? motor_elem->mass() : 0.0;

        const double L = beam->structuralSpanContribution();
        const double m_arm = elem->mass();
        const double r_o = beam->outerRadius();
        const double A = beam->crossSectionArea();
        const double I = beam->secondMomentOfArea();
        const double J = beam->polarMomentOfArea();

        if (L < 1e-6 || I < 1e-20 || A < 1e-20) {
            continue;
        }

        const double EI = E * I;
        const double q_arm = m_arm * g / L;  // distributed arm weight [N/m]

        for (const auto& [lc_id, thrust_arr] : load_cases) {
            const double T_motor = thrust_arr.at(static_cast<std::size_t>(motor_idx));

            // Net vertical tip force: thrust (up) minus motor weight (down).
            // In the cantilever model, positive F_tip_z = upward load at tip.
            const double F_tip_z = T_motor - m_motor * g;

            // Yaw reaction torque at tip. For a horizontal arm the yaw torque (about the
            // global z-axis) manifests as horizontal bending at the root, not torsion.
            const double Q_yaw = cT * T_motor;

            // --- Internal resultants (cantilever root section) ---

            // Axial: vertical loads are perpendicular to horizontal arm → N = 0.
            const double N = 0.0;

            // Root shear: sum of tip load and distributed arm weight.
            const double V = std::abs(F_tip_z + m_arm * g);

            // Bending moments at root.
            // Vertical plane (gravity + thrust): M_v = F_tip_z*L − q_arm*L²/2
            const double M_v = F_tip_z * L - m_arm * g * (L / 2.0);
            // Horizontal plane (yaw reaction acts as moment load at tip = bending moment at root).
            const double M_h = Q_yaw;
            const double M_b = std::sqrt(M_v * M_v + M_h * M_h);

            // Torsion: yaw torque is about vertical z-axis; horizontal arm axis has
            // zero z-component → projection onto arm axis = 0. T_torsion = 0.
            const double T_torsion = 0.0;

            // --- Stresses (hollow circular cross-section) ---
            const double sigma_ax = N / A;
            const double sigma_b = (I > 1e-20) ? M_b * r_o / I : 0.0;
            const double tau_t   = (J > 1e-20) ? T_torsion * r_o / J : 0.0;

            // Von Mises: σ_vm = sqrt((σ_ax + σ_b)² + 3τ²)
            const double sigma_vm = std::sqrt(
                (sigma_ax + sigma_b) * (sigma_ax + sigma_b) + 3.0 * tau_t * tau_t);

            const double sf = (sigma_vm > 1e-12)
                ? yield_strength / sigma_vm
                : (std::numeric_limits<double>::max)();

            // --- Euler-Bernoulli tip deflection (cantilever) ---
            // End load |F_tip_z|: δ = PL³/(3EI), θ = PL²/(2EI)
            // Distributed arm weight q_arm: δ += qL⁴/(8EI), θ += qL³/(6EI)
            const double P = std::abs(F_tip_z);
            const double delta_tip = P * L * L * L / (3.0 * EI)
                                   + q_arm * L * L * L * L / (8.0 * EI);
            const double theta_tip = P * L * L / (2.0 * EI)
                                   + q_arm * L * L * L / (6.0 * EI);

            min_sf       = std::min(min_sf, sf);
            max_delta    = std::max(max_delta, delta_tip);
            max_theta    = std::max(max_theta, theta_tip);
            max_sigma_vm = std::max(max_sigma_vm, sigma_vm);

            MemberLoadResult mlr;
            mlr.member_id   = arm_id;
            mlr.load_case_id = lc_id;
            mlr.L           = L;
            mlr.N           = N;
            mlr.V           = V;
            mlr.M_b         = M_b;
            mlr.T_torsion   = T_torsion;
            mlr.sigma_ax    = sigma_ax;
            mlr.sigma_b     = sigma_b;
            mlr.tau_t       = tau_t;
            mlr.sigma_vm    = sigma_vm;
            mlr.safety_factor = sf;
            mlr.delta_tip   = delta_tip;
            mlr.theta_tip   = theta_tip;
            model.network_structural.push_back(std::move(mlr));

            // Back-populate arm_structural from the max_thrust case for JSON export
            // compatibility with the existing physicalModelToJson() path.
            if (lc_id == "max_thrust") {
                ArmStructuralResult arm_result;
                arm_result.arm_id           = arm_id;
                arm_result.L_arm            = L;
                arm_result.M_vertical       = M_v;
                arm_result.M_horizontal     = M_h;
                arm_result.M_total          = M_b;
                arm_result.sigma_bending    = sigma_b;
                arm_result.safety_factor    = sf;
                arm_result.structural_failure = sf < context.minimum_arm_safety_factor;
                model.arm_structural.push_back(std::move(arm_result));
            }
        }
    }

    const double final_min_sf = std::isfinite(min_sf) && min_sf < (std::numeric_limits<double>::max)()
        ? min_sf : 0.0;

    // Write to both the legacy field (used by arm_yield_failure constraint) and the
    // new network-specific fields.
    model.structural.min_safety_factor         = final_min_sf;
    model.structural.network_min_safety_factor = final_min_sf;
    model.structural.network_max_tip_deflection = max_delta;
    model.structural.network_max_tip_rotation   = max_theta;
    model.structural.network_max_sigma_vm       = max_sigma_vm;
}

}  // namespace hexaarch::physics
