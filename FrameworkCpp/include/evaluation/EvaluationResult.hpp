#pragma once

#include <string>
#include <vector>

#include "core/Constraint.hpp"
#include "evaluation/MetricRole.hpp"
#include "physics/AttainableControlSetAnalyzer.hpp"
#include "physics/PhysicsTypes.hpp"

namespace hexaarch::evaluation {

struct ObjectiveValue {
    std::string name;
    double value = 0.0;
    double weight = 0.0;
    MetricRole role = MetricRole::soft_objective;
};

struct ConstraintResult {
    std::string stable_id;
    std::string owner_id;
    std::string name;
    bool hard = true;
    bool active = true;
    core::ConstraintEvaluation evaluation;

    [[nodiscard]] MetricRole metricRole() const {
        if (!active) return MetricRole::analysis_only;
        return hard ? MetricRole::hard_constraint : MetricRole::soft_objective;
    }
};

struct Stage1Metrics {
    double mass = 0.0;
    double power = 0.0;
    double fault_thrust = 0.0;
    double fault_alloc = 0.0;
    double hover_nom = 0.0;
    double structural = 0.0;
    double packaging = 0.0;
    double structural_safety = 0.0;
    double gamma_worst = 0.0;
    double sigma_worst = 0.0;
    double sigma_reference = 0.0;
    double hover_utilization_nominal = 0.0;
    bool hover_ok_nominal = false;

    // ACS metrics (Phase 1 — G: Attainable Control Set)
    bool acs_nominal_feasible = false;
    bool acs_all_faults_feasible = false;
    double acs_nominal_min_margin = 0.0;
    double acs_worst_fault_min_margin = 0.0;
    double acs_overall_min_margin = 0.0;
    // Soft penalty: max(0, -overall_min_margin) / (m*g). Zero when ACS is healthy.
    double acs_margin_penalty = 0.0;

    // Volume-based ACS metrics (matching MATLAB eval_acs.m)
    double acs_PFWAR = 0.0;    // Probabilistic Fault-Weighted ACS Retention (maximize)
    double acs_FII = 0.0;      // Fault Isotropy Index = std/mean retention (minimize)
    double acs_WCFR = 0.0;     // Worst-Case Fault Retention (maximize)
    double acs_hover_margin = 0.0;  // T_max/T_hover_worst - 1 (hard: >= 0; soft: maximize)

    // Per-axis directional reserves at nominal hover (spec §9.8 soft controllability metrics)
    double acs_yaw_reserve   = 0.0;  // margin in yaw_pos direction [Nm]
    double acs_roll_reserve  = 0.0;  // margin in roll_pos direction [Nm]
    double acs_pitch_reserve = 0.0;  // margin in pitch_pos direction [Nm]
    // Ratio of worst-fault to nominal minimum margin (0→1); 1 = no degradation
    double acs_faulted_to_nominal_ratio = 0.0;
    // Minimum signed distance from (L=0,M=0) to the 2D hover slice boundary across fault cases.
    // Positive = all faults hover with moment margin. Negative = hover envelope excludes origin.
    double acs_hover_slice_worst_fault_margin = 0.0;

    // Phase 2: Powertrain (D)
    double pt_total_power_nominal_w = 0.0;    // total electrical power at nominal hover [W]
    double pt_total_power_faulted_w = 0.0;    // total electrical power at worst-fault hover [W]
    double pt_worst_thrust_utilization = 0.0; // max(T_i/T_max) at nominal hover ∈ [0,1]
    double pt_worst_power_utilization = 0.0;  // max(P_i/P_cont) at nominal hover ∈ [0,1]

    // Phase 2: Battery (E)
    double bat_available_energy_wh = 0.0;      // E_avail = eta_pack × DoD × m_bat × e_spec [Wh]
    double bat_required_energy_wh = 0.0;       // E_req_total = nom + emg [Wh]
    double bat_energy_reserve_fraction = 0.0;  // (E_avail - E_req) / E_avail; ≥0 = feasible
    double bat_c_rate = 0.0;                   // peak pack current / rated capacity [1/h]
    double bat_mass_fraction = 0.0;            // m_bat / m_total ∈ [0,1]
    double bat_achievable_endurance_nom_min = 0.0;  // E_avail / (P_nom + P_aux) × 60 [min]; actual flyable time at nominal hover

    // Multi-mission profile (when EvaluationContext.mission_profile is set). Mirrors the
    // hover-only fields above for the active profile so SOO/MOO objectives can use either.
    // Defaults to zero when no profile is supplied (legacy hover-only sizing is in bat_*).
    bool   mission_active = false;
    double mission_total_time_s = 0.0;
    double mission_total_distance_m = 0.0;
    double mission_cruise_distance_m = 0.0;
    double mission_total_energy_wh = 0.0;        // propulsion-only [Wh]
    double mission_energy_with_aux_wh = 0.0;     // propulsion + auxiliary [Wh]
    double mission_peak_power_w = 0.0;           // max per-segment electrical power [W]
    double mission_hover_energy_wh = 0.0;
    double mission_cruise_energy_wh = 0.0;
    double mission_energy_reserve_fraction = 0.0;  // (E_avail - E_with_aux) / E_avail

    // Phase 3: Structural network (multi-load-case von Mises analysis — A/C in spec)
    double struct_net_min_safety_factor = 0.0;    // worst SF over all members × load cases
    double struct_net_max_tip_deflection_m = 0.0; // worst arm tip deflection [m]
    double struct_net_max_tip_rotation_rad = 0.0; // worst arm tip rotation [rad]
    double struct_net_max_sigma_vm_pa = 0.0;      // worst von Mises stress [Pa]

    // Phase 6: Packaging (ArchitecturePackagingEvaluator — Step 2)
    double pkg_rotor_clearance_m = 0.0;  // minimum inter-rotor disk clearance [m]; < 0 = overlap

    // Phase 7: Placement constraints (Step 3)
    double pkg_payload_containment_m = 0.0;       // > 0 = payload protrudes outside cabin [m]
    double pkg_battery_containment_m = 0.0;       // > 0 = battery protrudes outside cabin [m]
    double pkg_battery_payload_overlap_m = 0.0;   // > 0 = battery–payload penetration [m]
    double pkg_payload_internal_overlap_m = 0.0;  // > 0 = passenger/cargo/instrument overlap [m]
    double cabin_internal_clearance_m = 0.0;      // min positive cabin packing clearance [m]
    double cabin_space_penalty = 0.0;             // soft objective; lower is roomier
    double pkg_occupant_containment_m = 0.0;      // > 0 = occupant envelope protrudes outside cabin [m]
    double cg_y_offset_m = 0.0;                   // |lateral CG offset| from centerline [m]
    double pkg_rotor_keepout_m = 0.0;             // retained; always 0 (keepout zones removed)
};

struct EvaluationResult {
    bool feasible = true;
    double combined_objective = 0.0;
    std::string message;
    Stage1Metrics stage1;
    std::vector<ObjectiveValue> objectives;
    std::vector<ConstraintResult> constraint_results;
    physics::PhysicalModel physical_model;
    physics::AcsResult acs;
    physics::PowertrainResult powertrain;  // Phase 2
    physics::BatteryResult battery;        // Phase 2
};

}  // namespace hexaarch::evaluation
