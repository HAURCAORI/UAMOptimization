#include "physics/AllocationMatrixBuilder.hpp"

namespace hexaarch::physics {

Eigen::Matrix<double, 4, 6> AllocationMatrixBuilder::placeholder() {
    return Eigen::Matrix<double, 4, 6>::Zero();
}

}  // namespace hexaarch::physics
