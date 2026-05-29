#include "analysis/ComparisonReporter.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

namespace hexaarch::analysis {
namespace {

std::string shortParamName(const std::string& stable_id) {
    const auto pos = stable_id.rfind("::");
    return pos != std::string::npos ? stable_id.substr(pos + 2) : stable_id;
}

double denormParam(const optimization::OptimizationProblem& problem, std::size_t i, double norm) {
    const double lo = problem.lower_bounds.at(i);
    const double hi = problem.upper_bounds.at(i);
    const double sc = i < problem.parameter_scales.size() ? problem.parameter_scales.at(i) : 1.0;
    const double span = hi - lo;
    if (span <= 0.0 || sc == 0.0) { return norm; }
    return lo + span * (norm / sc);
}

}  // namespace

std::string ComparisonReporter::summarize(const evaluation::EvaluationResult& result) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    stream << (result.feasible ? "feasible" : "infeasible")
           << ", J=" << result.combined_objective
           << ", mass=" << result.stage1.mass
           << ", power=" << result.stage1.power
           << ", fault_thrust=" << result.stage1.fault_thrust
           << ", fault_alloc=" << result.stage1.fault_alloc
           << ", sf=" << result.physical_model.structural.min_safety_factor;

    if (!result.feasible) {
        for (const auto& cr : result.constraint_results) {
            if (cr.hard && cr.active && !cr.evaluation.feasible) {
                stream << " [violated: " << cr.name << " viol=" << cr.evaluation.violation << "]";
            }
        }
    }

    return stream.str();
}

std::string ComparisonReporter::compare(
    const evaluation::EvaluationResult& baseline,
    const evaluation::EvaluationResult& candidate) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    stream << "baseline J=" << baseline.combined_objective
           << ", candidate J=" << candidate.combined_objective
           << ", dJ=" << candidate.combined_objective - baseline.combined_objective
           << ", dMass=" << candidate.stage1.mass - baseline.stage1.mass
           << ", dPower=" << candidate.stage1.power - baseline.stage1.power
           << ", feasible=" << (candidate.feasible ? "true" : "false");
    return stream.str();
}

std::string ComparisonReporter::summarize(const optimization::SooRunResult& result) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    stream << result.algorithm_name
           << " seed=" << result.seed
           << " pop=" << result.population_size
           << " gen=" << result.generations
           << ", objective=" << result.objective_name
           << ", baseline J=" << result.baseline.combined_objective;
    if (result.best_feasible.has_value()) {
        stream << ", feasible best J=" << result.best_feasible->result.combined_objective;
    } else {
        stream << ", feasible best J=none"
               << ", raw best J=" << result.best_result.combined_objective;
    }
    return stream.str();
}

std::string ComparisonReporter::summarize(const optimization::MooRunResult& result) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    stream << result.algorithm_name
           << " seed=" << result.seed
           << " pop=" << result.population_size
           << " gen=" << result.generations
           << ", feasible points=" << result.feasible_population.size()
           << "/" << result.population.size();
    return stream.str();
}

std::string ComparisonReporter::summaryTable(
    const std::vector<std::pair<std::string, evaluation::EvaluationResult>>& labeled_results) {

    // Column name and value live in one place — adding/removing a metric here
    // updates both the CSV header and the data rows automatically.
    struct Col { std::string_view name; std::string value; };

    auto makeColumns = [](const std::string& label,
                          const evaluation::EvaluationResult& result) -> std::vector<Col> {
        bool hard_violated = false;
        std::string worst_id;
        double worst_viol = 0.0;
        for (const auto& cr : result.constraint_results) {
            if (cr.hard && cr.active && !cr.evaluation.feasible) {
                hard_violated = true;
                if (cr.evaluation.violation > worst_viol) {
                    worst_viol = cr.evaluation.violation;
                    worst_id   = cr.stable_id;
                }
            }
        }
        auto d = [](double v) {
            std::ostringstream s; s << std::fixed << std::setprecision(6) << v; return s.str();
        };
        return {
            {"label",                      label},
            {"feasible",                   result.feasible ? "true" : "false"},
            {"objective",                  d(result.combined_objective)},
            {"mass",                       d(result.stage1.mass)},
            {"power",                      d(result.stage1.power)},
            {"fault_thrust",               d(result.stage1.fault_thrust)},
            {"fault_alloc",                d(result.stage1.fault_alloc)},
            {"hover_nom",                  d(result.stage1.hover_nom)},
            {"structural",                 d(result.stage1.structural)},
            {"packaging",                  d(result.stage1.packaging)},
            {"structural_safety",          d(result.stage1.structural_safety)},
            {"min_safety_factor",          d(result.physical_model.structural.min_safety_factor)},
            {"hard_constraint_violated",   hard_violated ? "true" : "false"},
            {"worst_violation_constraint", std::move(worst_id)},
            {"worst_violation",            d(worst_viol)},
        };
    };

    std::ostringstream stream;
    const auto header = makeColumns("", {});
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (i > 0) stream << ',';
        stream << header[i].name;
    }
    stream << '\n';

    for (const auto& [lbl, result] : labeled_results) {
        const auto cols = makeColumns(lbl, result);
        for (std::size_t i = 0; i < cols.size(); ++i) {
            if (i > 0) stream << ',';
            stream << cols[i].value;
        }
        stream << '\n';
    }
    return stream.str();
}

std::string ComparisonReporter::parametersTable(const optimization::SooRunResult& result) {
    const auto& problem = result.problem;
    if (problem.parameter_ids.empty()) {
        return "  (no active parameters)\n";
    }

    const bool has_feasible = result.best_feasible.has_value();
    const auto& best_dv = has_feasible ? result.best_feasible->decision_vector : result.best_decision_vector;
    const std::string best_label = has_feasible ? "best_feasible" : "best_raw";

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6);
    stream << "  " << std::left << std::setw(28) << "param"
           << std::setw(8) << "unit"
           << std::right << std::setw(14) << "baseline"
           << std::setw(16) << best_label << '\n';
    stream << "  " << std::string(66, '-') << '\n';

    const auto count = problem.parameter_ids.size();
    for (std::size_t i = 0; i < count; ++i) {
        const std::string short_name = shortParamName(problem.parameter_ids.at(i));
        const std::string unit = i < problem.parameter_units.size() ? problem.parameter_units.at(i) : "";
        const double baseline_val = i < result.baseline_decision_vector.size()
            ? denormParam(problem, i, result.baseline_decision_vector.at(i)) : 0.0;
        const double best_val = i < best_dv.size()
            ? denormParam(problem, i, best_dv.at(i)) : 0.0;

        stream << "  " << std::left << std::setw(28) << short_name
               << std::setw(8) << unit
               << std::right << std::setw(14) << baseline_val
               << std::setw(16) << best_val << '\n';
    }
    return stream.str();
}

std::string ComparisonReporter::parametersTable(const optimization::MooRunResult& result) {
    const auto& problem = result.problem;
    if (problem.parameter_ids.empty()) {
        return "  (no active parameters)\n";
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6);
    stream << "  " << std::left << std::setw(28) << "param"
           << std::setw(8) << "unit"
           << std::right << std::setw(14) << "baseline" << '\n';
    stream << "  " << std::string(50, '-') << '\n';

    const auto count = problem.parameter_ids.size();
    for (std::size_t i = 0; i < count; ++i) {
        const std::string short_name = shortParamName(problem.parameter_ids.at(i));
        const std::string unit = i < problem.parameter_units.size() ? problem.parameter_units.at(i) : "";
        const double baseline_val = i < result.baseline_decision_vector.size()
            ? denormParam(problem, i, result.baseline_decision_vector.at(i)) : 0.0;

        stream << "  " << std::left << std::setw(28) << short_name
               << std::setw(8) << unit
               << std::right << std::setw(14) << baseline_val << '\n';
    }
    return stream.str();
}

}  // namespace hexaarch::analysis
