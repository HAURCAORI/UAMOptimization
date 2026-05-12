#include <filesystem>
#include <iostream>

#include "analysis/ComparisonReporter.hpp"
#include "analysis/CsvExporter.hpp"
#include "analysis/ParetoAnalyzer.hpp"
#include "core/HexacopterArchitecture.hpp"
#include "evaluation/ArchitectureEvaluator.hpp"
#include "optimization/MooRunner.hpp"
#include "optimization/SooRunner.hpp"

int main() {
    namespace fs = std::filesystem;

    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::EvaluationContext context;
    hexaarch::evaluation::ArchitectureEvaluator evaluator;

    const auto baseline = evaluator.evaluate(architecture, context);

    hexaarch::optimization::SooRunConfig soo_config;
    soo_config.population_size = 12U;
    soo_config.generations = 5U;
    const auto soo_result = hexaarch::optimization::SooRunner{}.run(architecture, context, soo_config);

    hexaarch::optimization::MooRunConfig moo_config;
    moo_config.population_size = 24U;
    moo_config.generations = 5U;
    moo_config.objective_names = {"mass", "power", "fault_alloc"};
    const auto moo_result = hexaarch::optimization::MooRunner{}.run(architecture, context, moo_config);

    const fs::path output_dir = fs::current_path() / "output";
    fs::create_directories(output_dir);

    const auto comparison_written = hexaarch::analysis::CsvExporter::writeComparisonCsv(
        output_dir / "comparison.csv",
        {
            {"baseline", baseline},
            {"soo_best", soo_result.best_result}
        });
    const auto soo_written = hexaarch::analysis::CsvExporter::writeSooJson(output_dir / "soo_run.json", soo_result);
    const auto moo_written = hexaarch::analysis::CsvExporter::writeParetoCsv(output_dir / "pareto_front.csv", moo_result);
    const auto moo_json_written = hexaarch::analysis::CsvExporter::writeMooJson(output_dir / "moo_run.json", moo_result);

    std::cout << "Baseline: " << hexaarch::analysis::ComparisonReporter::summarize(baseline) << '\n';
    std::cout << "SOO: " << hexaarch::analysis::ComparisonReporter::summarize(soo_result) << '\n';
    std::cout << "Delta: " << hexaarch::analysis::ComparisonReporter::compare(baseline, soo_result.best_result) << '\n';
    std::cout << "MOO: " << hexaarch::analysis::ParetoAnalyzer{}.summarize(moo_result) << '\n';
    std::cout << "Exports: comparison=" << comparison_written
              << ", soo_json=" << soo_written
              << ", pareto_csv=" << moo_written
              << ", moo_json=" << moo_json_written << '\n';
    std::cout << "Output directory: " << output_dir.string() << '\n';
    return 0;
}
