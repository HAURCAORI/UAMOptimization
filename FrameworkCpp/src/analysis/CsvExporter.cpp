#include "analysis/CsvExporter.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>

#include "analysis/ComparisonReporter.hpp"
#include "analysis/ParetoAnalyzer.hpp"
#include "core/Timestamp.hpp"
#include "nlohmann/json.hpp"

namespace hexaarch::analysis {
namespace {

using hexaarch::core::currentTimestamp;

std::string shortParamName(const std::string& stable_id) {
    const auto pos = stable_id.rfind("::");
    return pos != std::string::npos ? stable_id.substr(pos + 2) : stable_id;
}

const optimization::MooPoint* representativeMooPoint(const optimization::MooRunResult& result) {
    if (result.population.empty()) {
        return nullptr;
    }

    const auto summary = ParetoAnalyzer{}.analyze(result);
    const auto knee_it = std::find_if(
        result.population.begin(), result.population.end(),
        [&](const optimization::MooPoint& p) { return p.population_index == summary.knee_index; });
    if (knee_it != result.population.end()) {
        return &*knee_it;
    }

    if (!result.feasible_population.empty()) {
        return &result.feasible_population.front();
    }

    return &result.population.front();
}

double denormalizeParameter(
    const optimization::OptimizationProblem& problem,
    const std::size_t index,
    const double normalized_value) {
    const double lower = problem.lower_bounds.at(index);
    const double upper = problem.upper_bounds.at(index);
    const double scale = index < problem.parameter_scales.size() ? problem.parameter_scales.at(index) : 1.0;
    const double span = upper - lower;
    if (span <= 0.0 || scale == 0.0) {
        return normalized_value;
    }
    return lower + span * (normalized_value / scale);
}

nlohmann::json parameterMapToJson(
    const optimization::OptimizationProblem& problem,
    const std::vector<double>& decision_vector) {
    nlohmann::json parameters = nlohmann::json::object();
    const auto count = std::min(problem.parameter_ids.size(), decision_vector.size());
    for (std::size_t index = 0; index < count; ++index) {
        parameters[problem.parameter_ids.at(index)] =
            denormalizeParameter(problem, index, decision_vector.at(index));
    }
    return parameters;
}

nlohmann::json objectiveListToJson(const std::vector<evaluation::ObjectiveValue>& objectives) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& obj : objectives) {
        arr.push_back({{"name", obj.name}, {"value", obj.value}, {"weight", obj.weight}});
    }
    return arr;
}

nlohmann::json constraintListToJson(const std::vector<evaluation::ConstraintResult>& constraints) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& cr : constraints) {
        arr.push_back({
            {"id", cr.stable_id},
            {"hard", cr.hard},
            {"active", cr.active},
            {"feasible", cr.evaluation.feasible},
            {"violation", cr.evaluation.violation}
        });
    }
    return arr;
}

nlohmann::json stage1ToJson(const evaluation::Stage1Metrics& m) {
    return {
        {"mass", m.mass}, {"power", m.power},
        {"fault_thrust", m.fault_thrust}, {"fault_alloc", m.fault_alloc},
        {"hover_nom", m.hover_nom}, {"structural", m.structural}, {"packaging", m.packaging},
        {"gamma_worst", m.gamma_worst}, {"sigma_worst", m.sigma_worst},
        {"sigma_reference", m.sigma_reference},
        {"hover_utilization_nominal", m.hover_utilization_nominal},
        {"hover_ok_nominal", m.hover_ok_nominal}
    };
}

nlohmann::json physicalModelToJson(const physics::PhysicalModel& model) {
    const auto& com = model.mass_properties.center_of_mass;
    const auto& I = model.mass_properties.inertia;
    nlohmann::json alloc = nlohmann::json::array();
    for (int r = 0; r < 4; ++r) {
        nlohmann::json row = nlohmann::json::array();
        for (int c = 0; c < 6; ++c) {
            row.push_back(model.allocation_matrix(r, c));
        }
        alloc.push_back(row);
    }
    return {
        {"mass", model.mass_properties.mass},
        {"center_of_mass", {com.x(), com.y(), com.z()}},
        {"inertia_diagonal", {I(0, 0), I(1, 1), I(2, 2)}},
        {"minimum_clearance", model.packaging.minimum_clearance},
        {"overlap_penalty", model.packaging.overlap_penalty},
        {"arm_span", model.structural.arm_span},
        {"normalized_bending_index", model.structural.normalized_bending_index},
        {"allocation_matrix", alloc}
    };
}

nlohmann::json evaluationToJson(const evaluation::EvaluationResult& result) {
    return {
        {"feasible", result.feasible},
        {"combined_objective", result.combined_objective},
        {"message", result.message},
        {"stage1", stage1ToJson(result.stage1)},
        {"objectives", objectiveListToJson(result.objectives)},
        {"constraints", constraintListToJson(result.constraint_results)},
        {"physical_model", physicalModelToJson(result.physical_model)}
    };
}

nlohmann::json evaluationContextToJson(const evaluation::EvaluationContext& context) {
    nlohmann::json weights = nlohmann::json::array();
    for (const auto& w : context.objective_weights) {
        weights.push_back({{"name", w.name}, {"weight", w.weight}});
    }
    return {
        {"gamma_thrust_required", context.gamma_thrust_required},
        {"minimum_fault_allocation_ratio", context.minimum_fault_allocation_ratio},
        {"minimum_arm_length", context.minimum_arm_length},
        {"minimum_outer_arm_delta", context.minimum_outer_arm_delta},
        {"objective_weights", weights}
    };
}

nlohmann::json problemToJson(const optimization::OptimizationProblem& problem) {
    return {
        {"parameter_ids", problem.parameter_ids},
        {"lower_bounds", problem.lower_bounds},
        {"upper_bounds", problem.upper_bounds},
        {"parameter_scales", problem.parameter_scales},
        {"objective_names", problem.objective_names}
    };
}

bool writeJsonFile(const std::filesystem::path& path, const nlohmann::json& document) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        std::cerr << "[CsvExporter] failed to open for writing: " << path << '\n';
        return false;
    }
    stream << document.dump(2);
    if (!stream) {
        std::cerr << "[CsvExporter] write error: " << path << '\n';
        return false;
    }
    return true;
}

}  // namespace

bool CsvExporter::writeComparisonCsv(
    const std::filesystem::path& path,
    const std::vector<std::pair<std::string, evaluation::EvaluationResult>>& labeled_results) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        std::cerr << "[CsvExporter] failed to open for writing: " << path << '\n';
        return false;
    }
    stream << ComparisonReporter::summaryTable(labeled_results);
    return static_cast<bool>(stream);
}

bool CsvExporter::writeSooComparisonCsv(
    const std::filesystem::path& path,
    const optimization::SooRunResult& result) {
    std::vector<std::pair<std::string, evaluation::EvaluationResult>> rows{
        {"baseline", result.baseline},
        {"raw_best", result.best_result}
    };
    if (result.best_feasible.has_value()) {
        rows.push_back({"best_feasible", result.best_feasible->result});
    }
    return writeComparisonCsv(path, rows);
}

bool CsvExporter::writeCompareComparisonCsv(
    const std::filesystem::path& path,
    const evaluation::EvaluationResult& baseline,
    const optimization::SooRunResult& soo_result,
    const optimization::MooRunResult& moo_result) {
    std::vector<std::pair<std::string, evaluation::EvaluationResult>> rows{
        {"baseline", baseline},
        {"soo_raw_best", soo_result.best_result}
    };
    if (soo_result.best_feasible.has_value()) {
        rows.push_back({"soo_best_feasible", soo_result.best_feasible->result});
    }
    if (const auto* moo_point = representativeMooPoint(moo_result)) {
        rows.push_back({"moo_knee", moo_point->evaluation});
    }
    return writeComparisonCsv(path, rows);
}

bool CsvExporter::writeSooParametersCsv(
    const std::filesystem::path& path,
    const optimization::SooRunResult& result) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        std::cerr << "[CsvExporter] failed to open for writing: " << path << '\n';
        return false;
    }

    stream << "# algorithm=" << result.algorithm_name
           << " seed=" << result.seed
           << " gen=" << result.generations
           << " generated=" << currentTimestamp() << '\n';

    stream << "label";
    for (const auto& parameter_id : result.problem.parameter_ids) {
        stream << ',' << shortParamName(parameter_id);
    }
    stream << '\n';

    const auto write_row = [&](const std::string& label, const std::vector<double>& dv) {
        stream << label;
        for (std::size_t i = 0; i < result.problem.parameter_ids.size(); ++i) {
            stream << ',' << (i < dv.size() ? denormalizeParameter(result.problem, i, dv.at(i)) : 0.0);
        }
        stream << '\n';
    };

    write_row("baseline", result.baseline_decision_vector);
    write_row("raw_best", result.best_decision_vector);
    if (result.best_feasible.has_value()) {
        write_row("best_feasible", result.best_feasible->decision_vector);
    }

    return static_cast<bool>(stream);
}

bool CsvExporter::writeParetoCsv(
    const std::filesystem::path& path,
    const optimization::MooRunResult& result) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        std::cerr << "[CsvExporter] failed to open for writing: " << path << '\n';
        return false;
    }

    stream << "# algorithm=" << result.algorithm_name
           << " seed=" << result.seed
           << " gen=" << result.generations
           << " generated=" << currentTimestamp() << '\n';

    const auto summary = ParetoAnalyzer{}.analyze(result);

    stream << "population_index,is_nondominated,is_knee,feasible,combined_objective,gamma_worst,sigma_worst";
    for (const auto& name : result.objective_names) {
        stream << ",obj_" << name;
    }
    stream << '\n';

    for (const auto& point : result.population) {
        const auto is_nondominated =
            std::find(summary.nondominated_indices.begin(), summary.nondominated_indices.end(),
                      point.population_index) != summary.nondominated_indices.end();

        stream << point.population_index << ','
               << (is_nondominated ? "true" : "false") << ','
               << (summary.knee_index == point.population_index ? "true" : "false") << ','
               << (point.evaluation.feasible ? "true" : "false") << ','
               << point.evaluation.combined_objective << ','
               << point.evaluation.stage1.gamma_worst << ','
               << point.evaluation.stage1.sigma_worst;

        for (const auto value : point.objective_vector) {
            stream << ',' << value;
        }
        stream << '\n';
    }

    return static_cast<bool>(stream);
}

bool CsvExporter::writeParetoParametersCsv(
    const std::filesystem::path& path,
    const optimization::MooRunResult& result) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        std::cerr << "[CsvExporter] failed to open for writing: " << path << '\n';
        return false;
    }

    stream << "# algorithm=" << result.algorithm_name
           << " seed=" << result.seed
           << " gen=" << result.generations
           << " generated=" << currentTimestamp() << '\n';

    stream << "population_index,feasible";
    for (const auto& parameter_id : result.problem.parameter_ids) {
        stream << ',' << shortParamName(parameter_id);
    }
    stream << '\n';

    stream << "baseline,true";
    for (std::size_t i = 0; i < result.problem.parameter_ids.size(); ++i) {
        stream << ',' << (i < result.baseline_decision_vector.size()
            ? denormalizeParameter(result.problem, i, result.baseline_decision_vector.at(i))
            : 0.0);
    }
    stream << '\n';

    for (const auto& point : result.population) {
        stream << point.population_index << ',' << (point.evaluation.feasible ? "true" : "false");
        for (std::size_t i = 0; i < result.problem.parameter_ids.size(); ++i) {
            stream << ',' << (i < point.decision_vector.size()
                ? denormalizeParameter(result.problem, i, point.decision_vector.at(i))
                : 0.0);
        }
        stream << '\n';
    }

    return static_cast<bool>(stream);
}

bool CsvExporter::writeSooJson(
    const std::filesystem::path& path,
    const optimization::SooRunResult& result) {
    const nlohmann::json best_feasible_dv = result.best_feasible.has_value()
        ? nlohmann::json(result.best_feasible->decision_vector)
        : nlohmann::json(nullptr);
    const nlohmann::json best_feasible_params = result.best_feasible.has_value()
        ? parameterMapToJson(result.problem, result.best_feasible->decision_vector)
        : nlohmann::json(nullptr);
    const nlohmann::json best_feasible_eval = result.best_feasible.has_value()
        ? evaluationToJson(result.best_feasible->result)
        : nlohmann::json(nullptr);

    return writeJsonFile(path, {
        {"algorithm_name", result.algorithm_name},
        {"objective_name", result.objective_name},
        {"population_size", result.population_size},
        {"generations", result.generations},
        {"seed", result.seed},
        {"ftol", result.ftol},
        {"has_feasible_solution", result.best_feasible.has_value()},
        {"generated", currentTimestamp()},
        {"evaluation_context", evaluationContextToJson(result.evaluation_context)},
        {"problem", problemToJson(result.problem)},
        {"baseline_decision_vector", result.baseline_decision_vector},
        {"best_decision_vector", result.best_decision_vector},
        {"best_feasible_decision_vector", best_feasible_dv},
        {"baseline_parameters", parameterMapToJson(result.problem, result.baseline_decision_vector)},
        {"best_parameters", parameterMapToJson(result.problem, result.best_decision_vector)},
        {"best_feasible_parameters", best_feasible_params},
        {"baseline", evaluationToJson(result.baseline)},
        {"best_result", evaluationToJson(result.best_result)},
        {"best_feasible_result", best_feasible_eval}
    });
}

bool CsvExporter::writeMooJson(
    const std::filesystem::path& path,
    const optimization::MooRunResult& result) {
    nlohmann::json population = nlohmann::json::array();
    for (const auto& point : result.population) {
        population.push_back({
            {"population_index", point.population_index},
            {"decision_vector", point.decision_vector},
            {"parameters", parameterMapToJson(result.problem, point.decision_vector)},
            {"objective_vector", point.objective_vector},
            {"evaluation", evaluationToJson(point.evaluation)}
        });
    }

    nlohmann::json knee = nullptr;
    if (const auto* point = representativeMooPoint(result)) {
        knee = {
            {"population_index", point->population_index},
            {"decision_vector", point->decision_vector},
            {"parameters", parameterMapToJson(result.problem, point->decision_vector)},
            {"objective_vector", point->objective_vector},
            {"evaluation", evaluationToJson(point->evaluation)}
        };
    }

    return writeJsonFile(path, {
        {"algorithm_name", result.algorithm_name},
        {"objective_names", result.objective_names},
        {"population_size", result.population_size},
        {"generations", result.generations},
        {"seed", result.seed},
        {"has_feasible_points", result.has_feasible_points},
        {"generated", currentTimestamp()},
        {"evaluation_context", evaluationContextToJson(result.evaluation_context)},
        {"problem", problemToJson(result.problem)},
        {"baseline_decision_vector", result.baseline_decision_vector},
        {"baseline_parameters", parameterMapToJson(result.problem, result.baseline_decision_vector)},
        {"baseline", evaluationToJson(result.baseline)},
        {"knee_point", knee},
        {"population", population}
    });
}

bool CsvExporter::writeCompareJson(
    const std::filesystem::path& path,
    const evaluation::EvaluationResult& baseline,
    const optimization::SooRunResult& soo_result,
    const optimization::MooRunResult& moo_result) {
    nlohmann::json moo_knee = nullptr;
    if (const auto* point = representativeMooPoint(moo_result)) {
        moo_knee = {
            {"population_index", point->population_index},
            {"parameters", parameterMapToJson(moo_result.problem, point->decision_vector)},
            {"objective_vector", point->objective_vector},
            {"evaluation", evaluationToJson(point->evaluation)}
        };
    }

    const nlohmann::json soo_best_feasible_params = soo_result.best_feasible.has_value()
        ? parameterMapToJson(soo_result.problem, soo_result.best_feasible->decision_vector)
        : nlohmann::json(nullptr);
    const nlohmann::json soo_best_feasible_eval = soo_result.best_feasible.has_value()
        ? evaluationToJson(soo_result.best_feasible->result)
        : nlohmann::json(nullptr);

    return writeJsonFile(path, {
        {"generated", currentTimestamp()},
        {"evaluation_context", evaluationContextToJson(soo_result.evaluation_context)},
        {"baseline", evaluationToJson(baseline)},
        {"soo", {
            {"algorithm_name", soo_result.algorithm_name},
            {"seed", soo_result.seed},
            {"population_size", soo_result.population_size},
            {"generations", soo_result.generations},
            {"has_feasible_solution", soo_result.best_feasible.has_value()},
            {"best_result", evaluationToJson(soo_result.best_result)},
            {"best_feasible_result", soo_best_feasible_eval},
            {"best_parameters", parameterMapToJson(soo_result.problem, soo_result.best_decision_vector)},
            {"best_feasible_parameters", soo_best_feasible_params}
        }},
        {"moo", {
            {"algorithm_name", moo_result.algorithm_name},
            {"seed", moo_result.seed},
            {"population_size", moo_result.population.size()},
            {"generations", moo_result.generations},
            {"has_feasible_points", moo_result.has_feasible_points},
            {"feasible_points", moo_result.feasible_population.size()},
            {"baseline", evaluationToJson(moo_result.baseline)},
            {"knee_point", moo_knee}
        }}
    });
}

}  // namespace hexaarch::analysis
