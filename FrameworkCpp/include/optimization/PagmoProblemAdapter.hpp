#pragma once

#include <string>
#include <vector>

#include "pagmo/types.hpp"

#include "core/HexacopterArchitecture.hpp"
#include "evaluation/ArchitectureEvaluator.hpp"
#include "evaluation/EvaluationContext.hpp"
#include "evaluation/EvaluationResult.hpp"
#include "optimization/BoundsBuilder.hpp"
#include "optimization/DesignVectorMapper.hpp"
#include "optimization/OptimizationProblem.hpp"

namespace hexaarch::optimization {

class PagmoProblemAdapter {
public:
    PagmoProblemAdapter() = default;
    PagmoProblemAdapter(
        core::HexacopterArchitecture architecture,
        evaluation::EvaluationContext context,
        std::vector<std::string> objective_names,
        bool use_weighted_sum);

    [[nodiscard]] pagmo::vector_double fitness(const pagmo::vector_double& x) const;
    [[nodiscard]] std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const;
    [[nodiscard]] pagmo::vector_double::size_type get_nobj() const;
    [[nodiscard]] OptimizationProblem problem() const;
    [[nodiscard]] evaluation::EvaluationResult evaluate(const pagmo::vector_double& x) const;

private:
    [[nodiscard]] static double infeasiblePenalty(const evaluation::EvaluationResult& result, double penalty);
    [[nodiscard]] double objectiveValue(const evaluation::EvaluationResult& result, const std::string& objective_name) const;
    [[nodiscard]] double constraintPenalty(const core::HexacopterArchitecture& architecture, const evaluation::EvaluationResult& result) const;

    core::HexacopterArchitecture architecture_;
    evaluation::EvaluationContext context_;
    evaluation::ArchitectureEvaluator evaluator_;
    DesignVectorMapper mapper_;
    BoundsBuilder bounds_builder_;
    std::vector<std::string> objective_names_;
    bool use_weighted_sum_ = true;
};

}  // namespace hexaarch::optimization
