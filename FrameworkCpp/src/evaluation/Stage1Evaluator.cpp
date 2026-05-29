#include "evaluation/Stage1Evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include "evaluation/ObjectiveAggregator.hpp"
#include "mission/MissionEvaluator.hpp"
#include "mission/MissionProfile.hpp"
#include "physics/ArchitecturePackagingEvaluator.hpp"
#include "physics/AttainableControlSetAnalyzer.hpp"
#include "physics/BatteryEvaluator.hpp"
#include "physics/PowertrainEvaluator.hpp"
#include "physics/StructuralNetworkAnalyzer.hpp"
#include "physics/VehicleScalingModel.hpp"

namespace hexaarch::evaluation {
namespace {

double scaledControlEffectiveness(
    const physics::AllocationMatrix& matrix,
    const std::array<double, 6>& upper_bounds,
    const physics::PhysicalModel& model) {
    const double length_scale = std::max(model.structural.max_arm_length, 1e-3);
    double yaw_ref = 0.0;
    for (int index = 0; index < 6; ++index) {
        yaw_ref += std::abs(matrix(3, index)) * upper_bounds.at(static_cast<std::size_t>(index));
    }
    yaw_ref = std::max(yaw_ref, 1e-9);

    Eigen::Matrix4d scale = Eigen::Matrix4d::Zero();
    const double mg = model.mass_properties.mass * model.propulsion.gravity;
    scale(0, 0) = 1.0 / std::max(mg, 1e-9);
    scale(1, 1) = 1.0 / std::max(mg * length_scale, 1e-9);
    scale(2, 2) = 1.0 / std::max(mg * length_scale, 1e-9);
    scale(3, 3) = 1.0 / yaw_ref;

    Eigen::Matrix<double, 6, 6> thrust_scale = Eigen::Matrix<double, 6, 6>::Zero();
    for (int index = 0; index < 6; ++index) {
        thrust_scale(index, index) = upper_bounds.at(static_cast<std::size_t>(index));
    }

    const Eigen::JacobiSVD<Eigen::Matrix<double, 4, 6>> svd(
        scale * matrix * thrust_scale,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto singular_values = svd.singularValues();
    return singular_values.size() > 0 ? singular_values(singular_values.size() - 1) : 0.0;
}

}  // namespace

EvaluationResult Stage1Evaluator::evaluate(
    const core::HexacopterArchitecture& architecture,
    const EvaluationContext& context) const {
    EvaluationResult result;

    physics::VehicleScalingModel model_builder;
    result.physical_model = model_builder.evaluate(architecture);

    static const physics::PhysicalModel s_reference_model = []() {
        return physics::VehicleScalingModel{}.evaluate(core::HexacopterArchitecture{});
    }();
    static const physics::AcsResult s_reference_acs = []() {
        const physics::PhysicalModel& ref = s_reference_model;
        return physics::AttainableControlSetAnalyzer{}.analyze(
            ref.allocation_matrix,
            ref.faulted_allocation,
            ref.propulsion.thrust_max,
            ref.mass_properties.mass,
            ref.propulsion.gravity);
    }();
    static const double s_reference_power_w = []() {
        const physics::PowertrainEvaluator pt;
        const auto powertrain = pt.evaluate(
            s_reference_acs.nominal.trim_thrust,
            s_reference_acs.nominal.trim_thrust,
            s_reference_model.propulsion.thrust_max,
            s_reference_model.structural.max_arm_length,
            EvaluationContext{});
        return std::max(powertrain.total_power_nominal_w, 1e-9);
    }();
    const physics::PhysicalModel& reference_model = s_reference_model;

    // --- Phase 1: ACS analysis ---
    const physics::AttainableControlSetAnalyzer acs_analyzer;
    result.acs = acs_analyzer.analyze(
        result.physical_model.allocation_matrix,
        result.physical_model.faulted_allocation,
        result.physical_model.propulsion.thrust_max,
        result.physical_model.mass_properties.mass,
        result.physical_model.propulsion.gravity);

    // Nominal hover results come directly from the ACS trim solution.
    const auto& nominal_trim = result.acs.nominal;
    result.stage1.hover_ok_nominal = nominal_trim.trim_feasible;
    result.stage1.hover_utilization_nominal = nominal_trim.trim_utilization;

    // Fault hover feasibility across all single-rotor failures.
    bool all_single_fault_hover_ok = true;
    for (int index = 0; index < 6; ++index) {
        if (!result.acs.faulted.at(static_cast<std::size_t>(index)).trim_feasible) {
            all_single_fault_hover_ok = false;
            break;
        }
    }

    // ACS summary scalars.
    result.stage1.acs_nominal_feasible = nominal_trim.trim_feasible;
    result.stage1.acs_all_faults_feasible = all_single_fault_hover_ok;
    result.stage1.acs_nominal_min_margin = nominal_trim.min_margin;
    result.stage1.acs_worst_fault_min_margin = result.acs.worst_fault_min_margin;
    result.stage1.acs_overall_min_margin = result.acs.overall_min_margin;

    // Volume-based metrics (matching MATLAB eval_acs.m)
    result.stage1.acs_PFWAR = result.acs.PFWAR;
    result.stage1.acs_FII = result.acs.FII;
    result.stage1.acs_WCFR = result.acs.WCFR;
    result.stage1.acs_hover_margin = result.acs.hover_margin;

    // Per-axis reserves and fault degradation ratio (spec §9.8)
    result.stage1.acs_yaw_reserve   = result.acs.nominal.yaw_reserve;
    result.stage1.acs_roll_reserve  = result.acs.nominal.roll_reserve;
    result.stage1.acs_pitch_reserve = result.acs.nominal.pitch_reserve;
    result.stage1.acs_faulted_to_nominal_ratio = result.acs.faulted_to_nominal_ratio;
    result.stage1.acs_hover_slice_worst_fault_margin = result.acs.hover_slice_worst_fault_margin;

    const double mg = result.physical_model.mass_properties.mass * result.physical_model.propulsion.gravity;
    // Soft penalty: positive only when ACS is violated (margin < 0), normalized by vehicle weight.
    result.stage1.acs_margin_penalty =
        std::max(0.0, -result.acs.overall_min_margin) / std::max(mg, 1e-9);

    // --- Phase 2: Powertrain and battery analysis ---
    // Worst-fault motor: fault case with the highest total power demand.
    // This represents the most stressing condition for the battery.
    {
        int worst_fault_motor = 0;
        double worst_fault_power = -std::numeric_limits<double>::infinity();
        for (int idx = 0; idx < 6; ++idx) {
            double fault_total = 0.0;
            for (const double t : result.acs.faulted.at(static_cast<std::size_t>(idx)).trim_thrust) {
                fault_total += std::max(t, 0.0);
            }
            if (fault_total > worst_fault_power) {
                worst_fault_power = fault_total;
                worst_fault_motor = idx;
            }
        }

        const physics::PowertrainEvaluator pt_evaluator;
        result.powertrain = pt_evaluator.evaluate(
            result.acs.nominal.trim_thrust,
            result.acs.faulted.at(static_cast<std::size_t>(worst_fault_motor)).trim_thrust,
            result.physical_model.propulsion.thrust_max,
            result.physical_model.structural.max_arm_length,
            context);

        const physics::BatteryEvaluator bat_evaluator;
        if (context.mission_profile && !context.mission_profile->empty()) {
            constexpr double kPi = 3.14159265358979323846;
            const double r_eff =
                std::max(result.physical_model.structural.max_arm_length * 0.5, 0.1);
            const double single_disk_area = kPi * r_eff * r_eff;

            const mission::MissionEvaluator mission_eval;
            const mission::MissionResult mission_result = mission_eval.evaluate(
                *context.mission_profile,
                result.powertrain,
                result.physical_model,
                single_disk_area,
                context);

            result.battery = bat_evaluator.evaluateWithMission(
                result.powertrain,
                mission_result,
                architecture.batteryMass(),
                result.physical_model.mass_properties.mass,
                context);

            result.stage1.mission_active = true;
            result.stage1.mission_total_time_s        = mission_result.total_time_s;
            result.stage1.mission_total_distance_m    = mission_result.total_distance_m;
            result.stage1.mission_cruise_distance_m   = mission_result.cruise_distance_m;
            result.stage1.mission_total_energy_wh     = mission_result.total_energy_wh;
            result.stage1.mission_energy_with_aux_wh  = mission_result.total_energy_with_aux_wh;
            result.stage1.mission_peak_power_w        = mission_result.peak_power_w;
            result.stage1.mission_hover_energy_wh     = mission_result.hover_energy_wh;
            result.stage1.mission_cruise_energy_wh    = mission_result.cruise_energy_wh;
            result.stage1.mission_energy_reserve_fraction =
                result.battery.energy_reserve_fraction;
        } else {
            result.battery = bat_evaluator.evaluate(
                result.powertrain,
                architecture.batteryMass(),
                result.physical_model.mass_properties.mass,
                context);
        }

        result.stage1.pt_total_power_nominal_w  = result.powertrain.total_power_nominal_w;
        result.stage1.pt_total_power_faulted_w  = result.powertrain.total_power_faulted_w;
        result.stage1.pt_worst_thrust_utilization = result.powertrain.worst_thrust_utilization;
        result.stage1.pt_worst_power_utilization  = result.powertrain.worst_power_utilization;

        result.stage1.bat_available_energy_wh     = result.battery.available_energy_wh;
        result.stage1.bat_required_energy_wh      = result.battery.required_energy_total_wh;
        result.stage1.bat_energy_reserve_fraction = result.battery.energy_reserve_fraction;
        result.stage1.bat_c_rate                  = result.battery.c_rate;
        result.stage1.bat_mass_fraction           = result.battery.mass_fraction;

        // Achievable hover endurance at nominal power (including auxiliary draw).
        // Answers: how long could the vehicle fly if it hovered continuously at nominal trim?
        const double p_nom_total_w = result.powertrain.total_power_nominal_w + context.power_auxiliary_w;
        result.stage1.bat_achievable_endurance_nom_min =
            (nominal_trim.trim_feasible && p_nom_total_w > 1e-9)
            ? result.battery.available_energy_wh / p_nom_total_w * 60.0
            : 0.0;
    }

    // --- Phase 3: Structural network (multi-load-case von Mises analysis) ---
    // Runs after ACS so fault trim thrusts are available. Replaces the old StructuralAnalyzer
    // and populates model.arm_structural (backwards compat) and model.structural fields.
    {
        std::vector<std::array<double, physics::kNumRotors>> fault_thrusts;
        fault_thrusts.reserve(physics::kNumRotors);
        for (int fi = 0; fi < physics::kNumRotors; ++fi) {
            fault_thrusts.push_back(result.acs.faulted.at(static_cast<std::size_t>(fi)).trim_thrust);
        }
        physics::StructuralNetworkAnalyzer{}.analyze(
            result.physical_model, architecture, context,
            result.acs.nominal.trim_thrust,
            fault_thrusts);

        result.stage1.struct_net_min_safety_factor    = result.physical_model.structural.network_min_safety_factor;
        result.stage1.struct_net_max_tip_deflection_m = result.physical_model.structural.network_max_tip_deflection;
        result.stage1.struct_net_max_tip_rotation_rad = result.physical_model.structural.network_max_tip_rotation;
        result.stage1.struct_net_max_sigma_vm_pa      = result.physical_model.structural.network_max_sigma_vm;
    }

    // --- Phase 4: Packaging (rotor clearance + payload containment) ---
    physics::ArchitecturePackagingEvaluator{}.analyze(result.physical_model, architecture);
    result.stage1.pkg_rotor_clearance_m     = result.physical_model.packaging.minimum_rotor_clearance;
    result.stage1.pkg_payload_containment_m     = result.physical_model.packaging.payload_containment_violation;
    result.stage1.pkg_battery_containment_m     = result.physical_model.packaging.battery_containment_violation;
    result.stage1.pkg_battery_payload_overlap_m = result.physical_model.packaging.battery_payload_overlap;
    result.stage1.pkg_payload_internal_overlap_m = result.physical_model.packaging.payload_internal_overlap;
    result.stage1.pkg_occupant_containment_m    = result.physical_model.packaging.occupant_containment_violation;
    result.stage1.cg_y_offset_m                 = std::abs(result.physical_model.mass_properties.center_of_mass.y());
    result.stage1.pkg_rotor_keepout_m           = result.physical_model.packaging.rotor_keepout_intrusion_m;
    // Offending zone/element IDs: result.physical_model.packaging.rotor_keepout_offending_{zone,element}_id

    // --- Mass objective ---
    result.stage1.mass =
        result.physical_model.mass_properties.mass / std::max(result.physical_model.reference_mass, 1e-9);

    // --- Hover nominal power proxy ---
    result.stage1.hover_nom = nominal_trim.trim_feasible
        ? std::pow(
              (std::accumulate(nominal_trim.trim_thrust.begin(), nominal_trim.trim_thrust.end(), 0.0) / 6.0) /
                  std::max(result.physical_model.propulsion.thrust_max, 1e-9),
              2)
        : std::numeric_limits<double>::infinity();

    result.stage1.power = nominal_trim.trim_feasible
        ? result.powertrain.total_power_nominal_w / s_reference_power_w
        : std::numeric_limits<double>::infinity();

    // --- Legacy fault-tolerance proxies (kept for backwards-compatible objectives) ---
    double gamma_worst = std::numeric_limits<double>::infinity();
    double sigma_worst = std::numeric_limits<double>::infinity();
    double sigma_reference = std::numeric_limits<double>::infinity();

    for (int index = 0; index < 6; ++index) {
        std::array<double, 6> upper_bounds{};
        std::array<double, 6> upper_bounds_ref{};
        for (int motor = 0; motor < 6; ++motor) {
            const bool is_failed = (motor == index);
            upper_bounds.at(static_cast<std::size_t>(motor)) =
                is_failed ? 0.0 : result.physical_model.propulsion.thrust_max;
            upper_bounds_ref.at(static_cast<std::size_t>(motor)) =
                is_failed ? 0.0 : reference_model.propulsion.thrust_max;
        }

        const double remaining_thrust =
            std::accumulate(upper_bounds.begin(), upper_bounds.end(), 0.0);
        gamma_worst = std::min(
            gamma_worst,
            remaining_thrust / std::max(mg, 1e-9));

        sigma_worst = std::min(
            sigma_worst,
            scaledControlEffectiveness(
                result.physical_model.allocation_matrix,
                upper_bounds,
                result.physical_model));
        sigma_reference = std::min(
            sigma_reference,
            scaledControlEffectiveness(
                reference_model.allocation_matrix,
                upper_bounds_ref,
                reference_model));
    }

    result.stage1.gamma_worst = gamma_worst;
    result.stage1.sigma_worst = sigma_worst;
    result.stage1.sigma_reference = sigma_reference;
    result.stage1.fault_thrust = std::pow(std::max(0.0, context.gamma_thrust_required - gamma_worst), 2);
    result.stage1.fault_alloc = sigma_reference / std::max(sigma_worst, 1e-9);
    result.stage1.structural = result.physical_model.structural.normalized_bending_index;
    result.stage1.packaging = result.physical_model.packaging.overlap_penalty;
    {
        const double min_sf = result.physical_model.structural.min_safety_factor;
        result.stage1.structural_safety = context.minimum_arm_safety_factor / std::max(min_sf, 1e-9);
    }

    // --- Constraint evaluation ---
    result.constraint_results.clear();
    const core::ConstraintEvaluationContext constraint_context{
        architecture,
        result.physical_model,
        result.stage1,
        context
    };
    for (const auto& constraint : architecture.constraints().constraints()) {
        result.constraint_results.push_back({
            constraint.stable_id(),
            constraint.owner_id,
            constraint.name,
            constraint.hard,
            constraint.active,
            constraint.evaluate(constraint_context)
        });
    }

    // --- Feasibility ---
    const bool acs_fault_ok = !context.require_all_fault_acs_feasible || all_single_fault_hover_ok;
    result.feasible =
        nominal_trim.trim_feasible &&
        acs_fault_ok &&
        std::all_of(
            result.constraint_results.begin(),
            result.constraint_results.end(),
            [](const ConstraintResult& entry) {
                return !entry.hard || entry.evaluation.feasible;
            });

    ObjectiveAggregator aggregator;
    aggregator.aggregate(result, context);

    result.message = result.feasible
        ? "Stage 1 evaluation completed"
        : "Stage 1 evaluation detected infeasible constraints";
    return result;
}

}  // namespace hexaarch::evaluation
