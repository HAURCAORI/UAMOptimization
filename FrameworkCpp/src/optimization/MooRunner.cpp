#include "optimization/MooRunner.hpp"

#include <algorithm>
#include <stdexcept>

#include "pagmo/algorithm.hpp"
#include "pagmo/algorithms/nsga2.hpp"
#include "pagmo/population.hpp"
#include "pagmo/problem.hpp"

#include "optimization/PagmoProblemAdapter.hpp"

namespace hexaarch::optimization {

MooRunResult MooRunner::run(
    const core::HexacopterArchitecture& architecture,
    const evaluation::EvaluationContext& context,
    const MooRunConfig& config) const {
    PagmoProblemAdapter adapter{architecture, context, config.objective_names, false};
    pagmo::problem problem{adapter};

    if (config.population_size < 5U) {
        throw std::invalid_argument("MOO population_size must be at least 5.");
    }
    if (config.algorithm_name == "nsga2" && (config.population_size % 4U) != 0U) {
        throw std::invalid_argument("NSGA-II population_size must be a multiple of 4.");
    }

    pagmo::algorithm algorithm = [&]() {
        if (config.algorithm_name == "nsga2") {
            const auto variable_count = std::max<std::size_t>(DesignVectorMapper{}.pack(architecture).size(), 1U);
            const auto mutation_rate = 1.0 / static_cast<double>(variable_count);
            return pagmo::algorithm{
                pagmo::nsga2{config.generations, 0.95, 10.0, mutation_rate, 50.0, config.seed}
            };
        }
        throw std::invalid_argument("Unsupported MOO algorithm: " + config.algorithm_name);
    }();
    algorithm.set_verbosity(std::max(1U, config.generations / 10U));

    pagmo::population population{problem, static_cast<pagmo::population::size_type>(config.population_size), config.seed};
    const auto evolved = algorithm.evolve(population);

    MooRunResult result;
    result.algorithm_name = config.algorithm_name;
    result.objective_names = config.objective_names;
    result.minimize_objectives = config.minimize_objectives.empty()
        ? std::vector<bool>(config.objective_names.size(), true)
        : config.minimize_objectives;
    result.population_size = config.population_size;
    result.generations = config.generations;
    result.seed = config.seed;
    result.evaluation_context = context;
    result.problem = adapter.problem();
    result.baseline_decision_vector = DesignVectorMapper{}.pack(architecture);
    result.baseline = adapter.evaluate(result.baseline_decision_vector);

    const auto xs = evolved.get_x();
    const auto fs = evolved.get_f();
    result.population.reserve(xs.size());
    for (std::size_t index = 0; index < xs.size(); ++index) {
        auto point = MooPoint{
            index,
            std::vector<double>(xs.at(index).begin(), xs.at(index).end()),
            std::vector<double>(fs.at(index).begin(), fs.at(index).end()),
            adapter.evaluate(xs.at(index))
        };
        if (point.evaluation.feasible) {
            result.has_feasible_points = true;
            result.feasible_population.push_back(point);
        }
        result.population.push_back(std::move(point));
    }

    return result;
}

std::string MooRunner::describe() const {
    return "pagmo2-backed multi-objective Stage 1 runner";
}

}  // namespace hexaarch::optimization
