#include "physics/StructuralAnalyzer.hpp"

#include <cmath>
#include <limits>

#include "core/ElementCapabilities.hpp"
#include "core/HexacopterArchitecture.hpp"
#include "evaluation/EvaluationContext.hpp"

namespace hexaarch::physics {

void StructuralAnalyzer::analyze(
    PhysicalModel& model,
    const core::HexacopterArchitecture& arch,
    const evaluation::EvaluationContext& context) const {

    model.arm_structural.clear();

    const double Tmax = model.propulsion.thrust_max;
    const double cT = arch.cT();
    const double g = model.propulsion.gravity;
    const double yield_strength = context.arm_material.yield_strength;

    double min_sf = std::numeric_limits<double>::infinity();

    for (const auto& elem : arch.elements()) {
        auto* beam = dynamic_cast<core::IStructuralBeam*>(elem.get());
        auto* receiver = dynamic_cast<core::ILoadReceiver*>(elem.get());
        if (!beam || !receiver) {
            continue;
        }

        // Derive matching motor ID: "arm_N" -> "motor_N"
        const std::string& arm_id = elem->id();
        const std::string motor_id = "motor" + arm_id.substr(3);
        const auto* motor_elem = arch.findElement(motor_id);
        const double m_motor = motor_elem ? motor_elem->mass() : 0.0;

        const double L = beam->structuralSpanContribution();
        const double m_arm = elem->mass();
        const double r_o = beam->outerRadius();
        const double I = beam->secondMomentOfArea();

        // Worst-case load: all rotors at Tmax simultaneously.
        // In flight: thrust (up) and arm self-weight (down) act in OPPOSITE directions.
        // Net vertical bending at root = tip_force × L − arm_weight_contribution.
        const double F_tip_z = Tmax - m_motor * g;
        const double M_v = F_tip_z * L - m_arm * g * (L / 2.0);

        // Horizontal bending: yaw reaction torque about Z acts as bending moment at root.
        const double M_h = cT * Tmax;

        const double M_total = std::sqrt(M_v * M_v + M_h * M_h);
        const double sigma_b = (I > 1e-20) ? M_total * r_o / I : std::numeric_limits<double>::infinity();
        const double sf = (sigma_b > 1e-12) ? yield_strength / sigma_b : std::numeric_limits<double>::infinity();

        receiver->clearLoads();
        receiver->addLoad({
            Eigen::Vector3d(0.0, 0.0, F_tip_z),
            Eigen::Vector3d(0.0, 0.0, cT * Tmax),
            Eigen::Vector3d::Zero(),
            "motor_worst_case"
        });

        ArmStructuralResult result;
        result.arm_id = arm_id;
        result.L_arm = L;
        result.M_vertical = M_v;
        result.M_horizontal = M_h;
        result.M_total = M_total;
        result.sigma_bending = sigma_b;
        result.safety_factor = sf;
        result.structural_failure = sf < context.minimum_arm_safety_factor;

        min_sf = std::min(min_sf, sf);
        model.arm_structural.push_back(std::move(result));
    }

    model.structural.min_safety_factor =
        (min_sf == std::numeric_limits<double>::infinity()) ? 0.0 : min_sf;
}

}  // namespace hexaarch::physics
