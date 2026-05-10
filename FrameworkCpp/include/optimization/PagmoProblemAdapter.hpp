#pragma once

#include "optimization/OptimizationProblem.hpp"

namespace hexaarch::optimization {

class PagmoProblemAdapter {
public:
    [[nodiscard]] OptimizationProblem problem() const;
};

}  // namespace hexaarch::optimization
