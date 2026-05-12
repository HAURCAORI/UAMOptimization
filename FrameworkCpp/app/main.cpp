#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "analysis/ComparisonReporter.hpp"
#include "analysis/CsvExporter.hpp"
#include "analysis/ParetoAnalyzer.hpp"
#include "core/HexacopterArchitecture.hpp"
#include "core/Timestamp.hpp"
#include "evaluation/ArchitectureEvaluator.hpp"
#include "optimization/MooRunner.hpp"
#include "optimization/SooRunner.hpp"
#include "visualization/ArchitectureViewerApp.hpp"

namespace {

using hexaarch::core::currentTimestamp;

bool ensureOutputDirectory(const std::filesystem::path& dir) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (!std::filesystem::exists(dir)) {
        std::cerr << "[" << currentTimestamp() << "] ERROR: cannot create output directory: "
                  << dir << ": " << ec.message() << '\n';
        return false;
    }
    return true;
}

enum class CliMode {
    evaluate,
    soo,
    moo,
    compare,
    visualize
};

struct CliOptions {
    CliMode mode = CliMode::evaluate;
    std::filesystem::path output_dir = std::filesystem::current_path() / "output";
    unsigned soo_population_size = 24U;
    unsigned soo_generations = 40U;
    unsigned moo_population_size = 48U;
    unsigned moo_generations = 60U;
};

void printUsage() {
    std::cout
        << "HexaArch CLI\n"
        << "Usage:\n"
        << "  FrameworkCpp.exe [eval|soo|moo|compare|visualize] [--output-dir <path>] [--soo-pop <n>] [--soo-gen <n>] [--moo-pop <n>] [--moo-gen <n>]\n"
        << "Modes:\n"
        << "  eval     Evaluate the baseline Stage 1 design.\n"
        << "  soo      Run single-objective optimization.\n"
        << "  moo      Run multi-objective optimization.\n"
        << "  compare  Run baseline + SOO + MOO and export reports.\n"
        << "  visualize  Launch the architecture visualization utility.\n";
}

std::optional<CliMode> parseMode(const std::string_view token) {
    if (token == "eval") {
        return CliMode::evaluate;
    }
    if (token == "soo") {
        return CliMode::soo;
    }
    if (token == "moo") {
        return CliMode::moo;
    }
    if (token == "compare") {
        return CliMode::compare;
    }
    if (token == "visualize") {
        return CliMode::visualize;
    }
    return std::nullopt;
}

std::optional<CliOptions> parseArgs(const std::vector<std::string>& args) {
    CliOptions options;

    for (std::size_t index = 0; index < args.size(); ++index) {
        const auto& argument = args.at(index);
        if (argument == "--help" || argument == "-h") {
            printUsage();
            return std::nullopt;
        }

        if (argument == "--output-dir") {
            if (index + 1 >= args.size()) {
                std::cerr << "Missing value for --output-dir\n";
                return std::nullopt;
            }
            options.output_dir = args.at(++index);
            continue;
        }
        if (argument == "--soo-pop" && index + 1 < args.size()) {
            options.soo_population_size = static_cast<unsigned>(std::stoul(args.at(++index)));
            continue;
        }
        if (argument == "--soo-gen" && index + 1 < args.size()) {
            options.soo_generations = static_cast<unsigned>(std::stoul(args.at(++index)));
            continue;
        }
        if (argument == "--moo-pop" && index + 1 < args.size()) {
            options.moo_population_size = static_cast<unsigned>(std::stoul(args.at(++index)));
            continue;
        }
        if (argument == "--moo-gen" && index + 1 < args.size()) {
            options.moo_generations = static_cast<unsigned>(std::stoul(args.at(++index)));
            continue;
        }

        if (const auto mode = parseMode(argument)) {
            options.mode = *mode;
            continue;
        }

        std::cerr << "Unknown argument: " << argument << '\n';
        printUsage();
        return std::nullopt;
    }

    return options;
}

CliOptions promptInteractive() {
    CliOptions options;
    std::cout
        << "Select mode:\n"
        << "  1. eval\n"
        << "  2. soo\n"
        << "  3. moo\n"
        << "  4. compare\n"
        << "  5. visualize\n"
        << "> ";

    int selection = 1;
    std::cin >> selection;
    switch (selection) {
        case 2:
            options.mode = CliMode::soo;
            break;
        case 3:
            options.mode = CliMode::moo;
            break;
        case 4:
            options.mode = CliMode::compare;
            break;
        case 5:
            options.mode = CliMode::visualize;
            break;
        case 1:
        default:
            options.mode = CliMode::evaluate;
            break;
    }

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (options.mode == CliMode::soo || options.mode == CliMode::moo || options.mode == CliMode::compare) {
        std::cout << "Output directory [" << options.output_dir.string() << "]: ";
        std::string output_override;
        std::getline(std::cin, output_override);
        if (!output_override.empty()) {
            options.output_dir = output_override;
        }
    }

    return options;
}

void printBaseline(
    const hexaarch::core::HexacopterArchitecture& architecture,
    const hexaarch::evaluation::EvaluationContext& context) {
    const auto baseline = hexaarch::evaluation::ArchitectureEvaluator{}.evaluate(architecture, context);
    std::cout << "[" << currentTimestamp() << "] Architecture id: " << architecture.id() << '\n';
    std::cout << "[" << currentTimestamp() << "] Baseline: "
              << hexaarch::analysis::ComparisonReporter::summarize(baseline) << '\n';
}

void runSoo(
    const hexaarch::core::HexacopterArchitecture& architecture,
    const hexaarch::evaluation::EvaluationContext& context,
    const CliOptions& options) {
    hexaarch::optimization::SooRunConfig config;
    config.population_size = options.soo_population_size;
    config.generations = options.soo_generations;

    if (!ensureOutputDirectory(options.output_dir)) {
        return;
    }
    std::cout << "[" << currentTimestamp() << "] Running SOO with pop=" << config.population_size
              << ", gen=" << config.generations
              << ", output=" << options.output_dir.string() << '\n';
    const auto result = hexaarch::optimization::SooRunner{}.run(architecture, context, config);
    const auto comparison_written =
        hexaarch::analysis::CsvExporter::writeSooComparisonCsv(options.output_dir / "comparison.csv", result);
    const auto parameter_written =
        hexaarch::analysis::CsvExporter::writeSooParametersCsv(options.output_dir / "soo_parameters.csv", result);
    const auto soo_written = hexaarch::analysis::CsvExporter::writeSooJson(options.output_dir / "soo_run.json", result);

    std::cout << "[" << currentTimestamp() << "] SOO: "
              << hexaarch::analysis::ComparisonReporter::summarize(result) << '\n';
    if (result.best_feasible.has_value()) {
        std::cout << "[" << currentTimestamp() << "] Best feasible: "
                  << hexaarch::analysis::ComparisonReporter::summarize(result.best_feasible->result) << '\n';
        std::cout << "[" << currentTimestamp() << "] Delta: "
                  << hexaarch::analysis::ComparisonReporter::compare(result.baseline, result.best_feasible->result) << '\n';
    } else {
        std::cout << "[" << currentTimestamp() << "] Best feasible: none found\n";
        std::cout << "[" << currentTimestamp() << "] Raw best: "
                  << hexaarch::analysis::ComparisonReporter::summarize(result.best_result) << '\n';
    }
    std::cout << "[" << currentTimestamp() << "] Exports: comparison=" << comparison_written
              << ", parameters=" << parameter_written
              << ", soo_json=" << soo_written << '\n';
    std::cout << "[" << currentTimestamp() << "] Output directory: " << options.output_dir.string() << '\n';
}

void runMoo(
    const hexaarch::core::HexacopterArchitecture& architecture,
    const hexaarch::evaluation::EvaluationContext& context,
    const CliOptions& options) {
    hexaarch::optimization::MooRunConfig config;
    config.population_size = options.moo_population_size;
    config.generations = options.moo_generations;
    config.objective_names = {"mass", "power", "fault_alloc"};

    if (!ensureOutputDirectory(options.output_dir)) {
        return;
    }
    std::cout << "[" << currentTimestamp() << "] Running MOO with pop=" << config.population_size
              << ", gen=" << config.generations
              << ", output=" << options.output_dir.string() << '\n';
    const auto result = hexaarch::optimization::MooRunner{}.run(architecture, context, config);
    const auto pareto_written = hexaarch::analysis::CsvExporter::writeParetoCsv(options.output_dir / "pareto_front.csv", result);
    const auto parameter_written =
        hexaarch::analysis::CsvExporter::writeParetoParametersCsv(options.output_dir / "pareto_parameters.csv", result);
    const auto moo_written = hexaarch::analysis::CsvExporter::writeMooJson(options.output_dir / "moo_run.json", result);

    std::cout << "[" << currentTimestamp() << "] MOO: "
              << hexaarch::analysis::ComparisonReporter::summarize(result) << '\n';
    std::cout << "[" << currentTimestamp() << "] Pareto: "
              << hexaarch::analysis::ParetoAnalyzer{}.summarize(result) << '\n';
    std::cout << "[" << currentTimestamp() << "] Exports: pareto_csv=" << pareto_written
              << ", parameters_csv=" << parameter_written
              << ", moo_json=" << moo_written << '\n';
    std::cout << "[" << currentTimestamp() << "] Output directory: " << options.output_dir.string() << '\n';
}

void runCompare(
    const hexaarch::core::HexacopterArchitecture& architecture,
    const hexaarch::evaluation::EvaluationContext& context,
    const CliOptions& options) {
    if (!ensureOutputDirectory(options.output_dir)) {
        return;
    }

    const auto baseline = hexaarch::evaluation::ArchitectureEvaluator{}.evaluate(architecture, context);
    std::cout << "[" << currentTimestamp() << "] Baseline: "
              << hexaarch::analysis::ComparisonReporter::summarize(baseline) << '\n';

    hexaarch::optimization::SooRunConfig soo_config;
    soo_config.population_size = options.soo_population_size;
    soo_config.generations = options.soo_generations;
    std::cout << "[" << currentTimestamp() << "] Running SOO with pop=" << soo_config.population_size
              << ", gen=" << soo_config.generations << '\n';
    const auto soo_result = hexaarch::optimization::SooRunner{}.run(architecture, context, soo_config);
    std::cout << "[" << currentTimestamp() << "] SOO: "
              << hexaarch::analysis::ComparisonReporter::summarize(soo_result) << '\n';
    if (soo_result.best_feasible.has_value()) {
        std::cout << "[" << currentTimestamp() << "] Delta: "
                  << hexaarch::analysis::ComparisonReporter::compare(baseline, soo_result.best_feasible->result) << '\n';
    } else {
        std::cout << "[" << currentTimestamp() << "] Delta: feasible SOO result not found\n";
    }

    hexaarch::optimization::MooRunConfig moo_config;
    moo_config.population_size = options.moo_population_size;
    moo_config.generations = options.moo_generations;
    moo_config.objective_names = {"mass", "power", "fault_alloc"};
    std::cout << "[" << currentTimestamp() << "] Running MOO with pop=" << moo_config.population_size
              << ", gen=" << moo_config.generations << '\n';
    const auto moo_result = hexaarch::optimization::MooRunner{}.run(architecture, context, moo_config);
    std::cout << "[" << currentTimestamp() << "] MOO: "
              << hexaarch::analysis::ComparisonReporter::summarize(moo_result) << '\n';
    std::cout << "[" << currentTimestamp() << "] Pareto: "
              << hexaarch::analysis::ParetoAnalyzer{}.summarize(moo_result) << '\n';

    const auto comparison_written =
        hexaarch::analysis::CsvExporter::writeCompareComparisonCsv(
            options.output_dir / "comparison.csv", baseline, soo_result, moo_result);
    const auto soo_parameter_written =
        hexaarch::analysis::CsvExporter::writeSooParametersCsv(options.output_dir / "soo_parameters.csv", soo_result);
    const auto soo_written =
        hexaarch::analysis::CsvExporter::writeSooJson(options.output_dir / "soo_run.json", soo_result);
    const auto pareto_written =
        hexaarch::analysis::CsvExporter::writeParetoCsv(options.output_dir / "pareto_front.csv", moo_result);
    const auto pareto_parameter_written =
        hexaarch::analysis::CsvExporter::writeParetoParametersCsv(options.output_dir / "pareto_parameters.csv", moo_result);
    const auto moo_written =
        hexaarch::analysis::CsvExporter::writeMooJson(options.output_dir / "moo_run.json", moo_result);
    const auto compare_written =
        hexaarch::analysis::CsvExporter::writeCompareJson(options.output_dir / "compare_run.json", baseline, soo_result, moo_result);

    std::cout << "[" << currentTimestamp() << "] Exports: comparison=" << comparison_written
              << ", soo_parameters=" << soo_parameter_written
              << ", soo_json=" << soo_written
              << ", pareto_csv=" << pareto_written
              << ", pareto_parameters=" << pareto_parameter_written
              << ", moo_json=" << moo_written
              << ", compare_json=" << compare_written << '\n';
    std::cout << "[" << currentTimestamp() << "] Output directory: " << options.output_dir.string() << '\n';
}

int runVisualizer(hexaarch::core::HexacopterArchitecture& architecture) {
    architecture.rebuildAssembly();
    hexaarch::visualization::ArchitectureViewerApp app;
    app.setArchitecture(architecture);
    return app.run();
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> args;
    args.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
    for (int index = 1; index < argc; ++index) {
        args.emplace_back(argv[index]);
    }

    const auto parsed = args.empty() ? std::optional<CliOptions>{promptInteractive()} : parseArgs(args);
    if (!parsed.has_value()) {
        return args.empty() ? 0 : 1;
    }

    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::EvaluationContext context;

    switch (parsed->mode) {
        case CliMode::evaluate:
            printBaseline(architecture, context);
            break;
        case CliMode::soo:
            runSoo(architecture, context, *parsed);
            break;
        case CliMode::moo:
            runMoo(architecture, context, *parsed);
            break;
        case CliMode::compare:
            runCompare(architecture, context, *parsed);
            break;
        case CliMode::visualize:
            return runVisualizer(architecture);
    }

    return 0;
}
