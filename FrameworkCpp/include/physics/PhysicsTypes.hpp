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
