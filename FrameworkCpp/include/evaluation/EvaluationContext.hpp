#pragma once

#include <memory>
#include <string>
#include <vector>

#include "physics/Material.hpp"

namespace hexaarch::mission { struct MissionProfile; }

namespace hexaarch::evaluation {

struct ObjectiveWeight {
    std::string name;
    double weight = 0.0;
};

struct EvaluationContext {
    bool stage1_only = true;
    double gamma_thrust_required = 1.5;
    double minimum_fault_allocation_ratio = 0.05;
    double minimum_arm_length = 0.5;
    double minimum_outer_arm_delta = 0.1;
    physics::Material arm_material = physics::Materials::Al7075();
    double minimum_arm_safety_factor = 1.5;

    // ACS config (Phase 1 — G: Attainable Control Set)
    // Hard constraint: all single-rotor-fault trim cases must be feasible.
    bool require_all_fault_acs_feasible = true;
    // Minimum 4D directional ACS margin across all fault cases and all sampled directions [N or N·m].
    // Enforces a strictly positive wrench-space headroom beyond the hover trim point.
    // Calibrated at ~750 Nm for typical feasible designs; 50 Nm ≈ 7% of that, tightly positive.
    double eps_acs_fault_margin = 50.0;

    // Phase 2: Powertrain (D) — actuator-disk power model parameters.
    // Effective disk radius = max_arm_length / 2 (set by PowertrainEvaluator, not d_prop).
    double figure_of_merit = 0.65;    // hover figure of merit (ideal vs actual induced power)
    double motor_efficiency = 0.85;   // motor mechanical efficiency
    double esc_efficiency = 0.95;     // ESC electrical efficiency
    double air_density = 1.225;       // kg/m^3 at sea level ISA

    // Phase 2: Battery (E) — energy budget parameters.
    // All energies in Wh; times in minutes.
    // e_spec=250 Wh/kg: NMC811 cell-level (2025 era), pack efficiency applied separately.
    // C_rate=5.0: peak burst C-rate for hover phases; continuous during cruise would be ~1-2C.
    double battery_specific_energy_wh_per_kg = 250.0;  // NMC811 cell-level, 2025-era [Wh/kg]
    double battery_dod_usable = 0.85;                   // usable depth of discharge
    double battery_pack_efficiency = 0.95;              // pack-level Wh efficiency
    double battery_voltage_nominal = 48.0;              // nominal bus voltage [V]
    double battery_crate_limit = 5.0;                   // peak C-rate during hover [1/h]
    double mission_time_nominal_min = 30.0;             // required hover endurance [min]; UAM design mission (takeoff + transit hover)
    double mission_time_emergency_min = 1.0;            // required fault-hover endurance after failure [min]
    double power_auxiliary_w = 500.0;                   // avionics + payload auxiliary draw [W]

    // Multi-mission profile (cruise+hover). When set, BatteryEvaluator uses this instead of the
    // two fixed mission_time_*_min legs above. shared_ptr so the EvaluationContext stays
    // copyable and the optimizer can broadcast one profile across pagmo threads.
    // Calibration knob: equivalent parasite-drag area Cd·A_ref of the airframe in cruise. Sized
    // from typical UAM eVTOL flat-plate drag ~0.4-0.8 m²; the calibration module identifies it
    // from measured cruise power vs. airspeed when flight data is available.
    std::shared_ptr<const mission::MissionProfile> mission_profile;
    double parasite_drag_area_m2 = 0.6;

    // Phase 4: Stiffness / deflection constraints (B in spec)
    // Allowable limits applied to the worst-case (over all members × load cases) values.
    // Deflection limit: 10 cm is tight for UAM arms but keeps compliance finite for CMA-ES.
    // Rotation limit: 0.10 rad ≈ 5.7° gives visible indicator of excessive tip rotation.
    double arm_tip_deflection_limit_m = 0.10;   // max allowable Euler-Bernoulli tip deflection [m]
    double arm_tip_rotation_limit_rad = 0.10;   // max allowable tip rotation [rad] (~5.7°)

    // CG envelope: assembled center of mass must stay within this rectangular window so that
    // hover trim authority is retained across all fault cases. Checked as hard constraint.
    double cg_envelope_half_x = 0.40;  // |CG_x| <= this [m]  (fore/aft)
    double cg_envelope_half_y = 0.25;  // |CG_y| <= this [m]  (lateral)

    std::vector<ObjectiveWeight> objective_weights{
        {"mass", 0.20},
        {"power", 0.20},
        {"fault_thrust", 0.25},
        {"fault_alloc", 0.25},
        {"hover_nom", 0.10},
        {"structural", 0.0},
        {"packaging", 0.0},
        {"structural_safety", 0.0},
        {"acs_margin_penalty", 0.10}
    };
};

}  // namespace hexaarch::evaluation
