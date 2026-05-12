#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "evaluation/EvaluationResult.hpp"
#include "optimization/MooRunner.hpp"
#include "optimization/SooRunner.hpp"

namespace hexaarch::analysis {

class CsvExporter {
public:
    [[nodiscard]] static bool writeComparisonCsv(
        const std::filesystem::path& path,
        const std::vector<std::pair<std::string, evaluation::EvaluationResult>>& labeled_results);
    [[nodiscard]] static bool writeSooComparisonCsv(
        const std::filesystem::path& path,
        const optimization::SooRunResult& result);
    [[nodiscard]] static bool writeCompareComparisonCsv(
        const std::filesystem::path& path,
        const evaluation::EvaluationResult& baseline,
        const optimization::SooRunResult& soo_result,
        const optimization::MooRunResult& moo_result);
    [[nodiscard]] static bool writeSooParametersCsv(
        const std::filesystem::path& path,
        const optimization::SooRunResult& result);
    [[nodiscard]] static bool writeParetoCsv(
        const std::filesystem::path& path,
        const optimization::MooRunResult& result);
    [[nodiscard]] static bool writeParetoParametersCsv(
        const std::filesystem::path& path,
        const optimization::MooRunResult& result);
    [[nodiscard]] static bool writeSooJson(
        const std::filesystem::path& path,
        const optimization::SooRunResult& result);
    [[nodiscard]] static bool writeMooJson(
        const std::filesystem::path& path,
        const optimization::MooRunResult& result);
    [[nodiscard]] static bool writeCompareJson(
        const std::filesystem::path& path,
        const evaluation::EvaluationResult& baseline,
        const optimization::SooRunResult& soo_result,
        const optimization::MooRunResult& moo_result);
};

}  // namespace hexaarch::analysis
