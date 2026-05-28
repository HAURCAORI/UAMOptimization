#include "physics/VehicleScalingModel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "core/ElementCapabilities.hpp"
#include "physics/AllocationMatrixBuilder.hpp"

namespace hexaarch::physics {
namespace {

double hoverPowerProxy(const std::array<double, 6>& thrust, const double propeller_diameter) {
    constexpr double pi = 3.14159265358979323846;
    const double area = pi * std::pow(0.5 * propeller_diameter, 2);
    double power = 0.0;
    for (const double thrust_i : thrust) {
        power += std::pow(std::max(thrust_i, 0.0), 1.5);
    }
    return power / std::max(std::sqrt(area), 1e-9);
}

Eigen::Vector3d worldCOM(const core::AssembledElement& assembled) {
    return assembled.world_pose * assembled.element->localCOM();
}

}  // namespace

PhysicalModel VehicleScalingModel::evaluate(const core::HexacopterArchitecture& architecture) const {
    constexpr double baseline_mass = 2240.73;
    constexpr double baseline_tmax = 7327.0;
    constexpr double baseline_Lyo = 5.50;
    constexpr double baseline_Ix = 12000.0;
    constexpr double baseline_Iy = 9400.0;

    PhysicalModel model;
    model.propulsion.thrust_max = architecture.Tmax();
    model.propulsion.propeller_diameter = architecture.propellerDiameter();
    model.propulsion.gravity = architecture.gravity();
    model.propulsion.moment_to_thrust = architecture.cT();
    model.payload_mass = architecture.payloadMass();
    model.reference_mass = baseline_mass;

    std::array<Eigen::Vector3d, 6> rotor_positions{};
    rotor_positions.fill(Eigen::Vector3d::Zero());
    std::array<double, 6> rotor_yaw_signs{};
    rotor_yaw_signs.fill(0.0);
    double total_arm_length = 0.0;

    std::vector<std::pair<Eigen::Vector3d, double>> motor_contribs;

    Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    for (const auto& assembled : architecture.assemblyState().elements) {
        const double mass = assembled.element->mass();
        const Eigen::Vector3d com = worldCOM(assembled);
        weighted_sum += mass * com;
        inertia += assembled.element->localInertiaAtLocalCOM();
        inertia += mass * ((com.squaredNorm() * Eigen::Matrix3d::Identity()) - (com * com.transpose()));

        if (const auto* rotor = dynamic_cast<const core::IPropulsionRotor*>(assembled.element)) {
            const auto rotor_index = static_cast<std::size_t>(rotor->rotorIndex());
            rotor_positions.at(rotor_index) = assembled.world_pose.translation();
            rotor_yaw_signs.at(rotor_index) = rotor->yawMomentSign();
        }
        if (const auto* structural_member = dynamic_cast<const core::IStructuralMember*>(assembled.element)) {
            total_arm_length += structural_member->structuralSpanContribution();
            if (structural_member->contributesToFrameMass()) {
                model.frame_mass += mass;
            }
        }
        if (dynamic_cast<const core::IMotorMassContributor*>(assembled.element) != nullptr) {
            model.motor_mass += mass;
            motor_contribs.emplace_back(assembled.world_pose.translation(), mass);
        }
        if (dynamic_cast<const core::IPayloadMassContributor*>(assembled.element) != nullptr) {
            model.payload_mass = mass;
        }
    }

    model.mass_properties.mass = 0.0;
    for (const auto& assembled : architecture.assemblyState().elements) {
        model.mass_properties.mass += assembled.element->mass();
    }
    model.mass_properties.center_of_mass = weighted_sum / std::max(model.mass_properties.mass, 1e-9);
    model.mass_properties.inertia = inertia;

    double actual_motor_Ix = 0.0;
    double actual_motor_Iy = 0.0;
    for (const auto& [pos, m] : motor_contribs) {
        actual_motor_Ix += m * (pos.y() * pos.y() + pos.z() * pos.z());
        actual_motor_Iy += m * (pos.x() * pos.x() + pos.z() * pos.z());
    }
    model.body_inertia_x = baseline_Ix - actual_motor_Ix;
    model.body_inertia_y = baseline_Iy - actual_motor_Iy;
    model.mass_properties.inertia(0, 0) += model.body_inertia_x;
    model.mass_properties.inertia(1, 1) += model.body_inertia_y;
    model.mass_properties.inertia(2, 2) += model.body_inertia_x + model.body_inertia_y;

    for (std::size_t i = 0; i < rotor_positions.size(); ++i) {
        for (std::size_t j = i + 1; j < rotor_positions.size(); ++j) {
            if ((rotor_positions.at(i) - rotor_positions.at(j)).norm() < 1e-6) {
                std::cerr << "[VehicleScalingModel] rotors " << i << " and " << j
                          << " have coincident positions\n";
            }
        }
    }

    model.allocation_matrix = AllocationMatrixBuilder::build(rotor_positions, rotor_yaw_signs, model.propulsion.moment_to_thrust);
    for (int index = 0; index < 6; ++index) {
        model.faulted_allocation.at(index) = AllocationMatrixBuilder::withFailedRotor(
            rotor_positions,
            rotor_yaw_signs,
            model.propulsion.moment_to_thrust,
            index);
    }

    model.structural.arm_span = total_arm_length;
    model.structural.max_arm_length = std::max({architecture.Lx(), architecture.Lyi(), architecture.Lyo()});
    model.structural.frame_mass = model.frame_mass;
    model.structural.bending_index = architecture.Tmax() * std::max(total_arm_length / 6.0, 1e-9);
    model.structural.normalized_bending_index =
        model.structural.bending_index / std::max(baseline_tmax * baseline_Lyo, 1e-9);

    // Packaging analysis moved to ArchitecturePackagingEvaluator (called from Stage1Evaluator).

    const std::array<double, 6> nominal_hover_thrust{
        model.mass_properties.mass * model.propulsion.gravity / 6.0,
        model.mass_properties.mass * model.propulsion.gravity / 6.0,
        model.mass_properties.mass * model.propulsion.gravity / 6.0,
        model.mass_properties.mass * model.propulsion.gravity / 6.0,
        model.mass_properties.mass * model.propulsion.gravity / 6.0,
        model.mass_properties.mass * model.propulsion.gravity / 6.0
    };
    model.reference_power = hoverPowerProxy(nominal_hover_thrust, architecture.propellerDiameter());
    return model;
}

}  // namespace hexaarch::physics
