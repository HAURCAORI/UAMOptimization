#pragma once

#include <optional>
#include <string>
#include <vector>

#include "core/HexacopterArchitecture.hpp"
#include "evaluation/EvaluationContext.hpp"
#include "evaluation/EvaluationResult.hpp"
#include "optimization/OptimizationProblem.hpp"

namespace hexaarch::optimization {

struct SooFeasibleSolution {
    evaluation::EvaluationResult result;
    std::vector<double> decision_vector;
};

struct SooRunConfig {
    std::string algorithm_name = "cmaes";
    std::string objective_name = "combined";
    unsigned population_size = 24U;
    unsigned generations = 40U;
    unsigned seed = 42U;
    double ftol = 1e-6;
};

struct SooRunResult {
    std::string algorithm_name;
    std::string objective_name;
    unsigned population_size = 0U;
    unsigned generations = 0U;
    unsigned seed = 0U;
    double ftol = 0.0;
    evaluation::EvaluationContext evaluation_context;
    OptimizationProblem problem;
    evaluation::EvaluationResult baseline;
    evaluation::EvaluationResult best_result;
    std::optional<SooFeasibleSolution> best_feasible;
    std::vector<double> baseline_decision_vector;
    std::vector<double> best_decision_vector;
};

class SooRunner {
public:
    [[nodiscard]] SooRunResult run(
        const core::HexacopterArchitecture& architecture,
        const evaluation::EvaluationContext& context,
        const SooRunConfig& config = {}) const;
    [[nodiscard]] std::string describe() const;
};

}  // namespace hexaarch::optimization
