#include "analysis/ParetoAnalyzer.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>

namespace hexaarch::analysis {

ParetoSummary ParetoAnalyzer::analyze(const optimization::MooRunResult& result) const {
    ParetoSummary summary;
    const auto& working_population = result.feasible_population;
    if (working_population.empty()) {
        return summary;
    }

    const std::vector<bool> senses = result.minimize_objectives.empty()
        ? std::vector<bool>(working_population.front().objective_vector.size(), true)
        : result.minimize_objectives;

    for (std::size_t candidate_index = 0; candidate_index < working_population.size(); ++candidate_index) {
        bool dominated_by_other = false;
        for (std::size_t other_index = 0; other_index < working_population.size(); ++other_index) {
            if (candidate_index == other_index) {
                continue;
            }

            if (dominates(
                    working_population.at(other_index).objective_vector,
                    working_population.at(candidate_index).objective_vector,
                    senses)) {
                dominated_by_other = true;
                break;
            }
        }

        if (!dominated_by_other) {
            summary.nondominated_indices.push_back(working_population.at(candidate_index).population_index);
        }
    }

    if (summary.nondominated_indices.empty()) {
        summary.knee_index = 0U;
        return summary;
    }

    const auto first_point = std::find_if(
        working_population.begin(),
        working_population.end(),
        [&](const optimization::MooPoint& point) {
            return point.population_index == summary.nondominated_indices.front();
        });
    if (first_point == working_population.end()) {
        summary.knee_index = summary.nondominated_indices.front();
        return summary;
    }

    const auto objective_count = first_point->objective_vector.size();
    std::vector<double> minima(objective_count, std::numeric_limits<double>::infinity());
    std::vector<double> maxima(objective_count, -std::numeric_limits<double>::infinity());
    for (const auto raw_index : summary.nondominated_indices) {
        const auto point_it = std::find_if(
            working_population.begin(),
            working_population.end(),
            [&](const optimization::MooPoint& point) {
                return point.population_index == raw_index;
            });
        if (point_it == working_population.end()) {
            continue;
        }
        const auto& point = point_it->objective_vector;
        for (std::size_t objective_index = 0; objective_index < point.size(); ++objective_index) {
            minima.at(objective_index) = std::min(minima.at(objective_index), point.at(objective_index));
            maxima.at(objective_index) = std::max(maxima.at(objective_index), point.at(objective_index));
        }
    }

    summary.knee_index = summary.nondominated_indices.front();
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto raw_index : summary.nondominated_indices) {
        const auto point_it = std::find_if(
            working_population.begin(),
            working_population.end(),
            [&](const optimization::MooPoint& point) {
                return point.population_index == raw_index;
            });
        if (point_it == working_population.end()) {
            continue;
        }
        const auto score = normalizedKneeScore(point_it->objective_vector, minima, maxima);
        if (score < best_score) {
            best_score = score;
            summary.knee_index = raw_index;
        }
    }

    return summary;
}

std::string ParetoAnalyzer::summarize(const optimization::MooRunResult& result) const {
    const auto summary = analyze(result);

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    stream << result.algorithm_name
           << " seed=" << result.seed
           << " pop=" << result.population_size
           << " gen=" << result.generations
           << ", feasible points=" << result.feasible_population.size()
           << "/" << result.population.size()
           << ", nondominated=" << summary.nondominated_indices.size();

    if (summary.nondominated_indices.empty()) {
        return stream.str();
    }

    const auto knee_it = std::find_if(
        result.population.begin(), result.population.end(),
        [&](const optimization::MooPoint& p) { return p.population_index == summary.knee_index; });
    if (knee_it != result.population.end()) {
        stream << ", knee_index=" << summary.knee_index
               << ", knee_mass=" << knee_it->evaluation.stage1.mass
               << ", knee_power=" << knee_it->evaluation.stage1.power;
    }

    return stream.str();
}

bool ParetoAnalyzer::dominates(
    const std::vector<double>& lhs,
    const std::vector<double>& rhs,
    const std::vector<bool>& minimize) {
    if (lhs.size() != rhs.size()) {
        return false;
    }

    bool strictly_better = false;
    for (std::size_t index = 0; index < lhs.size(); ++index) {
        const bool is_minimize = index >= minimize.size() || minimize.at(index);
        const double better = is_minimize ? lhs.at(index) : -lhs.at(index);
        const double other  = is_minimize ? rhs.at(index) : -rhs.at(index);
        if (better > other) {
            return false;
        }
        if (better < other) {
            strictly_better = true;
        }
    }
    return strictly_better;
}

double ParetoAnalyzer::normalizedKneeScore(
    const std::vector<double>& point,
    const std::vector<double>& minima,
    const std::vector<double>& maxima) {
    double score = 0.0;
    for (std::size_t index = 0; index < point.size(); ++index) {
        const auto span = maxima.at(index) - minima.at(index);
        if (span <= 0.0) {
            continue;
        }
        score += (point.at(index) - minima.at(index)) / span;
    }
    return score;
}

}  // namespace hexaarch::analysis
