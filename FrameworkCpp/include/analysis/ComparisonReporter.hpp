#pragma once

#include <utility>
#include <string>
#include <vector>

#include "evaluation/EvaluationResult.hpp"
#include "optimization/MooRunner.hpp"
#include "optimization/SooRunner.hpp"

namespace hexaarch::analysis {

class ComparisonReporter {
public:
    [[nodiscard]] static std::string summarize(const evaluation::EvaluationResult& result);
    [[nodiscard]] static std::string compare(
        const evaluation::EvaluationResult& baseline,
        const evaluation::EvaluationResult& candidate);
    [[nodiscard]] static std::string summarize(const optimization::SooRunResult& result);
    [[nodiscard]] static std::string summarize(const optimization::MooRunResult& result);
    [[nodiscard]] static std::string summaryTable(
        const std::vector<std::pair<std::string, evaluation::EvaluationResult>>& labeled_results);
    [[nodiscard]] static std::string parametersTable(const optimization::SooRunResult& result);
    [[nodiscard]] static std::string parametersTable(const optimization::MooRunResult& result);
};

}  // namespace hexaarch::analysis
