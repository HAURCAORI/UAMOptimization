#pragma once

#include <array>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace hexaarch::physics {

constexpr int kNumRotors = 6;
constexpr int kNumControlDOF = 4;

using AllocationMatrix = Eigen::Matrix<double, kNumControlDOF, kNumRotors>;

struct MassProperties {
    double mass = 0.0;
    Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
};

struct PropulsionProperties {
    double thrust_max = 0.0;
    double moment_to_thrust = 0.0;
    double propeller_diameter = 0.0;
    double gravity = 9.81;
};

struct ArmStructuralResult {
    std::string arm_id;
    double L_arm = 0.0;
    double M_vertical = 0.0;
    double M_horizontal = 0.0;
    double M_total = 0.0;
    double sigma_bending = 0.0;
    double safety_factor = 0.0;
    bool structural_failure = false;
};

struct StructuralProxy {
    double arm_span = 0.0;
    double max_arm_length = 0.0;
    double frame_mass = 0.0;
    double motor_mass = 0.0;
    double bending_index = 0.0;
    double normalized_bending_index = 0.0;
    double min_safety_factor = 0.0;
};

struct PackagingReport {
    bool valid = true;
    double minimum_clearance = 0.0;
    double overlap_penalty = 0.0;
};

struct PowertrainResult {
    std::array<double, kNumRotors> motor_power_nominal{};   // W per motor, nominal hover trim
    std::array<double, kNumRotors> motor_power_faulted{};   // W per motor, worst-fault hover trim
    double total_power_nominal_w = 0.0;    // total electrical power at nominal hover [W]
    double total_power_faulted_w = 0.0;    // total electrical power at worst-fault hover [W]
    double worst_thrust_utilization = 0.0; // max(T_i / T_max) at nominal hover ∈ [0,1]
    double worst_power_utilization = 0.0;  // max(P_i / P_cont_i) at nominal hover ∈ [0,1]
};

struct BatteryResult {
    double available_energy_wh = 0.0;         // E_avail = eta_pack × DoD × m_bat × e_spec [Wh]
    double required_energy_nominal_wh = 0.0;  // P_nom × t_nom [Wh]
    double required_energy_fault_wh = 0.0;    // P_fault × t_emg [Wh]
    double required_energy_total_wh = 0.0;    // nom + fault [Wh]
    double energy_reserve_fraction = 0.0;     // (E_avail - E_req) / E_avail; ≥0 = feasible
    // C-rate = P_peak [W] / E_avail [Wh]; voltage cancels: C = I/Q = (P/V)/(E/V) = P/E
    double c_rate = 0.0;                      // peak discharge rate [1/h]; limit = C_allow
    double mass_fraction = 0.0;               // m_bat / m_total ∈ [0,1]
};

struct PhysicalModel {
    MassProperties mass_properties;
    PropulsionProperties propulsion;
    AllocationMatrix allocation_matrix = AllocationMatrix::Zero();
    std::array<AllocationMatrix, kNumRotors> faulted_allocation{};
    StructuralProxy structural;
    PackagingReport packaging;
    double reference_mass = 0.0;
    double reference_power = 0.0;
    double payload_mass = 0.0;
    double motor_mass = 0.0;
    double frame_mass = 0.0;
    double body_inertia_x = 0.0;
    double body_inertia_y = 0.0;
    std::vector<ArmStructuralResult> arm_structural;
};

}  // namespace hexaarch::physics
