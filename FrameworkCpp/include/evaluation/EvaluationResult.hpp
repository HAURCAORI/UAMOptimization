#pragma once

#include <string>

namespace hexaarch::evaluation {

struct EvaluationResult {
    bool feasible = true;
    double combined_objective = 0.0;
    std::string message;
};

}  // namespace hexaarch::evaluation
