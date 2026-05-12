#include "analysis/ComparisonReporter.hpp"

#include <array>
#include <iomanip>
#include <sstream>
#include <string_view>

namespace hexaarch::analysis {

std::string ComparisonReporter::summarize(const evaluation::EvaluationResult& result) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    stream << (result.feasible ? "feasible" : "infeasible")
           << ", J=" << result.combined_objective
           << ", mass=" << result.stage1.mass
           << ", power=" << result.stage1.power
           << ", fault_thrust=" << result.stage1.fault_thrust
           << ", fault_alloc=" << result.stage1.fault_alloc;

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
    static constexpr std::array<std::string_view, 13> kColumns = {
        "label", "feasible", "objective", "mass", "power",
        "fault_thrust", "fault_alloc", "hover_nom", "structural", "packaging",
        "hard_constraint_violated", "worst_violation_constraint", "worst_violation"
    };

    std::ostringstream stream;
    for (std::size_t i = 0; i < kColumns.size(); ++i) {
        if (i > 0) { stream << ','; }
        stream << kColumns[i];
    }
    stream << '\n';
    stream << std::fixed << std::setprecision(6);
    for (const auto& [label, result] : labeled_results) {
        bool hard_violated = false;
        std::string worst_id;
        double worst_violation = 0.0;
        for (const auto& cr : result.constraint_results) {
            if (cr.hard && cr.active && !cr.evaluation.feasible) {
                hard_violated = true;
                if (cr.evaluation.violation > worst_violation) {
                    worst_violation = cr.evaluation.violation;
                    worst_id = cr.stable_id;
                }
            }
        }

        stream << label << ','
               << (result.feasible ? "true" : "false") << ','
               << result.combined_objective << ','
               << result.stage1.mass << ','
               << result.stage1.power << ','
               << result.stage1.fault_thrust << ','
               << result.stage1.fault_alloc << ','
               << result.stage1.hover_nom << ','
               << result.stage1.structural << ','
               << result.stage1.packaging << ','
               << (hard_violated ? "true" : "false") << ','
               << worst_id << ','
               << worst_violation << '\n';
    }
    return stream.str();
}

}  // namespace hexaarch::analysis
