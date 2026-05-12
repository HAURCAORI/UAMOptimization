#include "physics/AllocationMatrixBuilder.hpp"

namespace hexaarch::physics {

AllocationMatrix AllocationMatrixBuilder::build(
    const double Lx,
    const double Lyi,
    const double Lyo,
    const double cT) {
    AllocationMatrix matrix;
    matrix <<
        -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
        -Lyi, Lyi, -Lyo, Lyo, -Lyi, Lyi,
        Lx, Lx, 0.0, 0.0, -Lx, -Lx,
        -cT, -cT, cT, cT, -cT, cT;
    return matrix;
}

AllocationMatrix AllocationMatrixBuilder::build(
    const std::array<Eigen::Vector3d, 6>& rotor_positions,
    const std::array<double, 6>& yaw_moment_signs,
    const double cT) {
    AllocationMatrix matrix = AllocationMatrix::Zero();
    for (int index = 0; index < 6; ++index) {
        matrix(0, index) = -1.0;
        matrix(1, index) = rotor_positions.at(static_cast<std::size_t>(index)).y();
        matrix(2, index) = rotor_positions.at(static_cast<std::size_t>(index)).x();
        matrix(3, index) = yaw_moment_signs.at(static_cast<std::size_t>(index)) * cT;
    }
    return matrix;
}

AllocationMatrix AllocationMatrixBuilder::withFailedRotor(
    const double Lx,
    const double Lyi,
    const double Lyo,
    const double cT,
    const int failed_rotor_index) {
    AllocationMatrix matrix = build(Lx, Lyi, Lyo, cT);
    if (failed_rotor_index >= 0 && failed_rotor_index < matrix.cols()) {
        matrix.col(failed_rotor_index).setZero();
    }
    return matrix;
}

AllocationMatrix AllocationMatrixBuilder::withFailedRotor(
    const std::array<Eigen::Vector3d, 6>& rotor_positions,
    const std::array<double, 6>& yaw_moment_signs,
    const double cT,
    const int failed_rotor_index) {
    AllocationMatrix matrix = build(rotor_positions, yaw_moment_signs, cT);
    if (failed_rotor_index >= 0 && failed_rotor_index < matrix.cols()) {
        matrix.col(failed_rotor_index).setZero();
    }
    return matrix;
}

}  // namespace hexaarch::physics
