#pragma once

#include <string>

#include "evaluation/EvaluationResult.hpp"

namespace hexaarch::analysis {

class ComparisonReporter {
public:
    [[nodiscard]] static std::string summarize(const evaluation::EvaluationResult& result);
};

}  // namespace hexaarch::analysis
