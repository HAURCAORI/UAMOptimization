#pragma once

#include <vector>

namespace hexaarch::optimization {

struct OptimizationProblem {
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
};

}  // namespace hexaarch::optimization
