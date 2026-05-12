#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "optimization/MooRunner.hpp"

namespace hexaarch::analysis {

struct ParetoSummary {
    std::vector<std::size_t> nondominated_indices;
    std::size_t knee_index = 0U;
};

class ParetoAnalyzer {
public:
    [[nodiscard]] ParetoSummary analyze(const optimization::MooRunResult& result) const;
    [[nodiscard]] std::string summarize(const optimization::MooRunResult& result) const;

private:
    [[nodiscard]] static bool dominates(
        const std::vector<double>& lhs,
        const std::vector<double>& rhs,
        const std::vector<bool>& minimize);
    [[nodiscard]] static double normalizedKneeScore(
        const std::vector<double>& point,
        const std::vector<double>& minima,
        const std::vector<double>& maxima);
};

}  // namespace hexaarch::analysis
