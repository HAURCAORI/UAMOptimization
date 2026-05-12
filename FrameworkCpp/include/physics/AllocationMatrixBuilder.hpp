#pragma once

#include <array>

#include "eigen3/Eigen/Dense"

#include "physics/PhysicsTypes.hpp"

namespace hexaarch::physics {

class AllocationMatrixBuilder {
public:
    [[nodiscard]] static AllocationMatrix build(double Lx, double Lyi, double Lyo, double cT);
    [[nodiscard]] static AllocationMatrix build(
        const std::array<Eigen::Vector3d, 6>& rotor_positions,
        const std::array<double, 6>& yaw_moment_signs,
        double cT);
    [[nodiscard]] static AllocationMatrix withFailedRotor(
        double Lx,
        double Lyi,
        double Lyo,
        double cT,
        int failed_rotor_index);
    [[nodiscard]] static AllocationMatrix withFailedRotor(
        const std::array<Eigen::Vector3d, 6>& rotor_positions,
        const std::array<double, 6>& yaw_moment_signs,
        double cT,
        int failed_rotor_index);
};

}  // namespace hexaarch::physics
