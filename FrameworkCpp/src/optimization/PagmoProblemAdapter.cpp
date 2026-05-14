#include "optimization/PagmoProblemAdapter.hpp"

#include <cmath>
#include <stdexcept>

namespace hexaarch::optimization {
namespace {

constexpr double kInfeasibleBasePenalty = 1.0e6;

double sanitizeObjectiveValue(const double value) {
    if (!std::isfinite(value)) {
        return kInfeasibleBasePenalty;
    }
    return value;
}

}  // namespace

PagmoProblemAdapter::PagmoProblemAdapter(
    core::HexacopterArchitecture architecture,
    evaluation::EvaluationContext context,
    std::vector<std::string> objective_names,
    const bool use_weighted_sum)
    : architecture_(std::move(architecture)),
      context_(std::move(context)),
      objective_names_(std::move(objective_names)),
      use_weighted_sum_(use_weighted_sum) {}

pagmo::vector_double PagmoProblemAdapter::fitness(const pagmo::vector_double& x) const {
    const auto result = evaluate(x);
    const auto penalty = constraintPenalty(architecture_, result);
    const auto infeasible = infeasiblePenalty(result, penalty);

    if (use_weighted_sum_) {
        return {result.feasible ? sanitizeObjectiveValue(result.combined_objective) + penalty : infeasible};
    }

    std::vector<double> values;
    values.reserve(objective_names_.size());
    for (const auto& objective_name : objective_names_) {
        values.push_back(result.feasible ? sanitizeObjectiveValue(objectiveValue(result, objective_name)) + penalty : infeasible);
    }
    return values;
}

std::pair<pagmo::vector_double, pagmo::vector_double> PagmoProblemAdapter::get_bounds() const {
    return {
        bounds_builder_.lowerBounds(architecture_),
        bounds_builder_.upperBounds(architecture_)
    };
}

pagmo::vector_double::size_type PagmoProblemAdapter::get_nobj() const {
    return static_cast<pagmo::vector_double::size_type>(use_weighted_sum_ ? 1U : objective_names_.size());
}

OptimizationProblem PagmoProblemAdapter::problem() const {
    OptimizationProblem problem;
    problem.parameter_ids = mapper_.parameterIds(architecture_);
    for (const auto* parameter : architecture_.parameters().activeParameters()) {
        problem.lower_bounds.push_back(parameter->lower_bound);
        problem.upper_bounds.push_back(parameter->upper_bound);
        problem.parameter_scales.push_back(parameter->scale);
        problem.parameter_units.push_back(parameter->unit);
    }
    problem.objective_names = use_weighted_sum_ ? std::vector<std::string>{"combined"} : objective_names_;
    return problem;
}

evaluation::EvaluationResult PagmoProblemAdapter::evaluate(const pagmo::vector_double& x) const {
    auto candidate = architecture_;
    mapper_.unpackNormalized(candidate, x);
    return evaluator_.evaluate(candidate, context_);
}

double PagmoProblemAdapter::infeasiblePenalty(const evaluation::EvaluationResult& result, const double penalty) {
    return kInfeasibleBasePenalty + penalty + (result.feasible ? 0.0 : kInfeasibleBasePenalty);
}

double PagmoProblemAdapter::objectiveValue(
    const evaluation::EvaluationResult& result,
    const std::string& objective_name) const {
    if (objective_name == "combined") {
        return result.combined_objective;
    }

    for (const auto& objective : result.objectives) {
        if (objective.name == objective_name) {
            return objective.value;
        }
    }

    throw std::invalid_argument("Unknown objective name: " + objective_name);
}

double PagmoProblemAdapter::constraintPenalty(
    const core::HexacopterArchitecture& architecture,
    const evaluation::EvaluationResult& result) const {
    double penalty = 0.0;
    for (const auto& constraint_result : result.constraint_results) {
        if (!constraint_result.active || constraint_result.evaluation.feasible) {
            continue;
        }

        double weight = 1.0;
        if (const auto* definition = architecture.constraints().find(constraint_result.stable_id)) {
            weight = definition->penalty;
        }

        penalty += weight * constraint_result.evaluation.violation;
    }
    if (!result.feasible) {
        penalty += 1.0e4;
    }
    return penalty;
}

}  // namespace hexaarch::optimization
