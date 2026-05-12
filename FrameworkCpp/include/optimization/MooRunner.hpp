#pragma once

#include <string>
#include <vector>

#include "core/HexacopterArchitecture.hpp"
#include "evaluation/EvaluationContext.hpp"
#include "evaluation/EvaluationResult.hpp"
#include "optimization/OptimizationProblem.hpp"

namespace hexaarch::optimization {

struct MooRunConfig {
    std::string algorithm_name = "nsga2";
    std::vector<std::string> objective_names{"mass", "power", "fault_thrust", "fault_alloc", "hover_nom"};
    std::vector<bool> minimize_objectives; // empty = all minimize (default)
    unsigned population_size = 32U;
    unsigned generations = 60U;
    unsigned seed = 42U;
};

struct MooPoint {
    std::size_t population_index = 0U;
    std::vector<double> decision_vector;
    std::vector<double> objective_vector;
    evaluation::EvaluationResult evaluation;
};

struct MooRunResult {
    std::string algorithm_name;
    std::vector<std::string> objective_names;
    std::vector<bool> minimize_objectives;
    unsigned population_size = 0U;
    unsigned generations = 0U;
    unsigned seed = 0U;
    bool has_feasible_points = false;
    evaluation::EvaluationContext evaluation_context;
    OptimizationProblem problem;
    evaluation::EvaluationResult baseline;
    std::vector<double> baseline_decision_vector;
    std::vector<MooPoint> population;
    std::vector<MooPoint> feasible_population;
};

class MooRunner {
public:
    [[nodiscard]] MooRunResult run(
        const core::HexacopterArchitecture& architecture,
        const evaluation::EvaluationContext& context,
        const MooRunConfig& config = {}) const;
    [[nodiscard]] std::string describe() const;
};

}  // namespace hexaarch::optimization
