#include "optimization/MooRunner.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include "pagmo/algorithm.hpp"
#include "pagmo/algorithms/nsga2.hpp"
#include "pagmo/population.hpp"
#include "pagmo/problem.hpp"

#include "optimization/PagmoProblemAdapter.hpp"

namespace hexaarch::optimization {
namespace {

double rawObjectiveValue(
    const evaluation::EvaluationResult& evaluation,
    const std::string& objective_name) {
    if (objective_name == "combined") {
        return evaluation.combined_objective;
    }

    for (const auto& objective : evaluation.objectives) {
        if (objective.name == objective_name) {
            return objective.value;
        }
    }

    throw std::invalid_argument("Unknown objective name: " + objective_name);
}

std::vector<double> rawObjectiveVector(
    const evaluation::EvaluationResult& evaluation,
    const std::vector<std::string>& objective_names) {
    std::vector<double> values;
    values.reserve(objective_names.size());
    for (const auto& name : objective_names) {
        values.push_back(rawObjectiveValue(evaluation, name));
    }
    return values;
}

}  // namespace

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

    const bool use_callback = static_cast<bool>(config.on_generation);
    const auto variable_count = std::max<std::size_t>(DesignVectorMapper{}.pack(architecture).size(), 1U);
    const auto mutation_rate = 1.0 / static_cast<double>(variable_count);

    auto makeAlgorithm = [&](unsigned gen_count) {
        if (config.algorithm_name == "nsga2") {
            return pagmo::algorithm{
                pagmo::nsga2{gen_count, 0.95, 10.0, mutation_rate, 50.0, config.seed}
            };
        }
        throw std::invalid_argument("Unsupported MOO algorithm: " + config.algorithm_name);
    };

    pagmo::population pop{problem, static_cast<pagmo::population::size_type>(config.population_size), config.seed};

    if (use_callback) {
        pagmo::algorithm alg = makeAlgorithm(1U);
        const unsigned log_interval = std::max(1U, config.generations / 10U);
        for (unsigned gen = 0U; gen < config.generations; ++gen) {
            pop = alg.evolve(pop);
            const auto& fs = pop.get_f();
            const auto& xs = pop.get_x();
            std::size_t best_idx = 0U;
            double best_sum = std::numeric_limits<double>::max();
            for (std::size_t i = 0U; i < fs.size(); ++i) {
                double sum = 0.0;
                for (const double f : fs[i]) { sum += f; }
                if (sum < best_sum) { best_sum = sum; best_idx = i; }
            }
            if ((gen + 1U) % log_interval == 0U || gen + 1U == config.generations) {
                std::cout << "Gen:\t" << (gen + 1U) << "\tBest obj sum:\t" << best_sum << '\n';
            }
            config.on_generation(gen + 1U, config.generations, xs[best_idx]);
        }
    } else {
        pagmo::algorithm alg = makeAlgorithm(config.generations);
        alg.set_verbosity(std::max(1U, config.generations / 10U));
        pop = alg.evolve(pop);
    }

    const auto evolved = pop;

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
    result.population.reserve(xs.size());
    for (std::size_t index = 0; index < xs.size(); ++index) {
        auto evaluation = adapter.evaluate(xs.at(index));
        auto point = MooPoint{
            index,
            std::vector<double>(xs.at(index).begin(), xs.at(index).end()),
            rawObjectiveVector(evaluation, config.objective_names),
            std::move(evaluation)
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
