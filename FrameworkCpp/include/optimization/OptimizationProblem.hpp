#pragma once

#include <functional>
#include <string>
#include <vector>

namespace hexaarch::optimization {

using GenerationCallback = std::function<void(unsigned gen, unsigned total_gen, const std::vector<double>& best_x)>;

struct OptimizationProblem {
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<double> parameter_scales;
    std::vector<std::string> parameter_ids;
    std::vector<std::string> parameter_units;
    std::vector<std::string> objective_names;
};

}  // namespace hexaarch::optimization
