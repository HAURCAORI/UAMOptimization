#pragma once

#include <string>
#include <vector>

namespace hexaarch::optimization {

struct OptimizationProblem {
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<double> parameter_scales;
    std::vector<std::string> parameter_ids;
    std::vector<std::string> objective_names;
};

}  // namespace hexaarch::optimization
