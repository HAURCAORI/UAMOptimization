#pragma once

#include <Eigen/Core>

namespace hexaarch::physics {

class AllocationMatrixBuilder {
public:
    [[nodiscard]] static Eigen::Matrix<double, 4, 6> placeholder();
};

}  // namespace hexaarch::physics
