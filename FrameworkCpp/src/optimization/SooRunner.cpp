#include "optimization/SooRunner.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "pagmo/algorithm.hpp"
#include "pagmo/algorithms/cmaes.hpp"
#include "pagmo/population.hpp"
#include "pagmo/problem.hpp"

#include "optimization/PagmoProblemAdapter.hpp"

namespace hexaarch::optimization {

SooRunResult SooRunner::run(
    const core::HexacopterArchitecture& architecture,
    const evaluation::EvaluationContext& context,
    const SooRunConfig& config) const {
    PagmoProblemAdapter adapter{
        architecture,
        context,
        {config.objective_name},
        config.objective_name == "combined"
    };
    pagmo::problem problem{adapter};

    pagmo::algorithm algorithm = [&]() {
        if (config.algorithm_name == "cmaes") {
            return pagmo::algorithm{
                pagmo::cmaes{config.generations, -1.0, -1.0, -1.0, -1.0, 0.5, config.ftol, config.ftol, false, true, config.seed}
            };
        }
        throw std::invalid_argument("Unsupported SOO algorithm: " + config.algorithm_name);
    }();
    algorithm.set_verbosity(std::max(1U, config.generations / 10U));

    pagmo::population population{problem, static_cast<pagmo::population::size_type>(config.population_size), config.seed};
    const auto evolved = algorithm.evolve(population);
    const auto best_x = evolved.champion_x();

    SooRunResult result;
    result.algorithm_name = config.algorithm_name;
    result.objective_name = config.objective_name;
    result.population_size = config.population_size;
    result.generations = config.generations;
    result.seed = config.seed;
    result.ftol = config.ftol;
    result.evaluation_context = context;
    result.problem = adapter.problem();
    result.baseline_decision_vector = DesignVectorMapper{}.pack(architecture);
    result.baseline = adapter.evaluate(result.baseline_decision_vector);
    result.best_result = adapter.evaluate(best_x);
    result.best_decision_vector.assign(best_x.begin(), best_x.end());

    const auto xs = evolved.get_x();
    double best_feasible_objective = std::numeric_limits<double>::infinity();
    for (const auto& x : xs) {
        const auto evaluation = adapter.evaluate(x);
        if (!evaluation.feasible) {
            continue;
        }

        const double objective = config.objective_name == "combined"
            ? evaluation.combined_objective
            : adapter.fitness(x).front();
        if (objective < best_feasible_objective) {
            best_feasible_objective = objective;
            result.best_feasible = SooFeasibleSolution{evaluation, std::vector<double>(x.begin(), x.end())};
        }
    }

    if (!result.best_feasible.has_value() && result.best_result.feasible) {
        result.best_feasible = SooFeasibleSolution{result.best_result, result.best_decision_vector};
    }

    return result;
}

std::string SooRunner::describe() const {
    return "pagmo2-backed single-objective Stage 1 runner";
}

}  // namespace hexaarch::optimization
