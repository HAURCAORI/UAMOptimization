#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "analysis/AcsPlotter.hpp"
#include "analysis/ComparisonReporter.hpp"
#include "analysis/CsvExporter.hpp"
#include "analysis/ParetoAnalyzer.hpp"
#include "calibration/Calibrator.hpp"
#include "calibration/FlightData.hpp"
#include "core/HexacopterArchitecture.hpp"
#include "core/Timestamp.hpp"
#include "evaluation/ArchitectureEvaluator.hpp"
#include "mission/MissionEvaluator.hpp"
#include "mission/MissionProfile.hpp"
#include "nlohmann/json.hpp"
#include "optimization/DesignVectorMapper.hpp"
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
    visualize,
    calibrate,
    mission
};

struct CliOptions {
    CliMode mode = CliMode::evaluate;
    std::filesystem::path output_dir = std::filesystem::current_path() / "output";
    unsigned soo_population_size = 24U;
    unsigned soo_generations = 40U;
    unsigned moo_population_size = 48U;
    unsigned moo_generations = 60U;
    bool visualize = false;
    bool plot_acs = false;
    std::optional<std::filesystem::path> mission_profile_path;  // --mission <profile.json>
    std::optional<std::filesystem::path> calibration_csv_path;  // calibrate <csv> | --flight-data <csv>
};

void printUsage() {
    std::cout
        << "HexaArch CLI\n"
        << "Usage:\n"
        << "  FrameworkCpp.exe [eval|soo|moo|compare|visualize|calibrate|mission] [options]\n"
        << "Modes:\n"
        << "  eval                       Evaluate the baseline Stage 1 design.\n"
        << "  soo                        Run single-objective optimization.\n"
        << "  moo                        Run multi-objective optimization.\n"
        << "  compare                    Run baseline + SOO + MOO and export reports.\n"
        << "  visualize                  Browse and render exported result files.\n"
        << "  calibrate <flight.csv>     Identify physics params from flight-data CSV.\n"
        << "  mission   <profile.json>   Evaluate baseline with a multi-segment mission profile.\n"
        << "Flags:\n"
        << "  --output-dir <path>        Output directory (default: ./output).\n"
        << "  --soo-pop/--soo-gen <n>    SOO population / generation count.\n"
        << "  --moo-pop/--moo-gen <n>    MOO population / generation count.\n"
        << "  --visualize                Show real-time viewer during soo/moo optimization.\n"
        << "  --plot-acs                 Generate SVG ACS plots in <output-dir>/acs/.\n"
        << "  --mission <profile.json>   Apply a mission profile to eval/soo/moo/compare/mission modes.\n"
        << "  --flight-data <flight.csv> Used by 'calibrate' (alternative to positional arg).\n";
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
    if (token == "calibrate") {
        return CliMode::calibrate;
    }
    if (token == "mission") {
        return CliMode::mission;
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
        if (argument == "--visualize") {
            options.visualize = true;
            continue;
        }
        if (argument == "--plot-acs") {
            options.plot_acs = true;
            continue;
        }
        if (argument == "--mission" && index + 1 < args.size()) {
            options.mission_profile_path = args.at(++index);
            continue;
        }
        if (argument == "--flight-data" && index + 1 < args.size()) {
            options.calibration_csv_path = args.at(++index);
            continue;
        }

        if (const auto mode = parseMode(argument)) {
            options.mode = *mode;
            continue;
        }

        // Positional path argument for calibrate / mission modes.
        if (options.mode == CliMode::calibrate && !options.calibration_csv_path.has_value()) {
            options.calibration_csv_path = argument;
            continue;
        }
        if (options.mode == CliMode::mission && !options.mission_profile_path.has_value()) {
            options.mission_profile_path = argument;
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
        << "  6. calibrate (requires flight-data CSV)\n"
        << "  7. mission   (requires mission profile JSON)\n"
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
        case 6:
            options.mode = CliMode::calibrate;
            break;
        case 7:
            options.mode = CliMode::mission;
            break;
        case 1:
        default:
            options.mode = CliMode::evaluate;
            break;
    }

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (options.mode == CliMode::soo || options.mode == CliMode::moo ||
        options.mode == CliMode::compare) {
        std::cout << "Output directory [" << options.output_dir.string() << "]: ";
        std::string output_override;
        std::getline(std::cin, output_override);
        if (!output_override.empty()) {
            options.output_dir = output_override;
        }
    }

    if (options.mode == CliMode::soo || options.mode == CliMode::moo) {
        std::cout << "Enable real-time visualization? [y/N]: ";
        std::string answer;
        std::getline(std::cin, answer);
        options.visualize = (!answer.empty() && (answer[0] == 'y' || answer[0] == 'Y'));
    }

    if (options.mode != CliMode::moo && options.mode != CliMode::visualize &&
        options.mode != CliMode::calibrate) {
        std::cout << "Generate ACS plots (SVG, opens in any browser)? [y/N]: ";
        std::string answer;
        std::getline(std::cin, answer);
        options.plot_acs = (!answer.empty() && (answer[0] == 'y' || answer[0] == 'Y'));
    }

    if (options.mode == CliMode::calibrate) {
        std::cout << "Flight-data CSV path: ";
        std::string path;
        std::getline(std::cin, path);
        if (!path.empty()) options.calibration_csv_path = path;
    }
    if (options.mode == CliMode::mission || options.mode == CliMode::evaluate ||
        options.mode == CliMode::soo || options.mode == CliMode::moo ||
        options.mode == CliMode::compare) {
        std::cout << "Mission profile JSON (blank to skip): ";
        std::string path;
        std::getline(std::cin, path);
        if (!path.empty()) options.mission_profile_path = path;
    }

    return options;
}

// Forward declaration — definition follows runAcsPlot below.
void runAcsPlot(const hexaarch::evaluation::EvaluationResult&, const CliOptions&, const std::string&);

// Loads the mission profile from --mission <path> (if any) and attaches it to context. Returns
// true if a profile was supplied AND loaded successfully; false if no profile was requested.
// Exits the caller via std::exit on parse failure so an invalid profile never falls through to a
// silent legacy-hover evaluation.
bool attachMissionProfile(
    const CliOptions& options,
    hexaarch::evaluation::EvaluationContext& context) {
    if (!options.mission_profile_path.has_value()) return false;
    const auto profile = hexaarch::mission::loadMissionProfileJson(*options.mission_profile_path);
    if (!profile.has_value()) {
        std::cerr << "[" << currentTimestamp()
                  << "] ERROR: failed to load mission profile: "
                  << options.mission_profile_path->string() << "\n";
        std::exit(1);
    }
    context.mission_profile =
        std::make_shared<const hexaarch::mission::MissionProfile>(std::move(*profile));
    std::cout << "[" << currentTimestamp()
              << "] Mission profile loaded: "
              << (context.mission_profile->name.empty()
                  ? options.mission_profile_path->filename().string()
                  : context.mission_profile->name)
              << "  segments=" << context.mission_profile->segments.size() << '\n';
    return true;
}

void printMissionSummary(const hexaarch::evaluation::EvaluationResult& result) {
    if (!result.stage1.mission_active) return;
    std::cout << std::fixed << std::setprecision(2)
              << "[" << currentTimestamp() << "] Mission:"
              << " time=" << result.stage1.mission_total_time_s << " s"
              << " range=" << result.stage1.mission_cruise_distance_m / 1000.0 << " km"
              << " E_total=" << result.stage1.mission_energy_with_aux_wh / 1000.0 << " kWh"
              << " E_hover=" << result.stage1.mission_hover_energy_wh / 1000.0 << " kWh"
              << " E_cruise=" << result.stage1.mission_cruise_energy_wh / 1000.0 << " kWh"
              << " P_peak=" << result.stage1.mission_peak_power_w / 1000.0 << " kW"
              << " reserve=" << result.stage1.mission_energy_reserve_fraction * 100.0 << " %\n";
}

void runCalibration(const CliOptions& options) {
    if (!options.calibration_csv_path.has_value()) {
        std::cerr << "[" << currentTimestamp() << "] ERROR: 'calibrate' mode requires a flight-data CSV.\n";
        return;
    }
    auto data = hexaarch::calibration::loadFlightDataCsv(*options.calibration_csv_path);
    if (!data.has_value()) {
        std::cerr << "[" << currentTimestamp() << "] ERROR: cannot load flight data.\n";
        return;
    }

    // Effective disk geometry from the baseline architecture.
    hexaarch::core::HexacopterArchitecture arch;
    arch.rebuildAssembly();
    const double max_arm = std::max({arch.Lx(), arch.Lyi(), arch.Lyo()});
    constexpr double kPi = 3.14159265358979323846;
    const double r_eff = std::max(max_arm * 0.5, 0.1);
    const double A_single = kPi * r_eff * r_eff;

    hexaarch::calibration::CalibrationBounds bounds;
    hexaarch::calibration::CalibrationMask mask;
    hexaarch::calibration::CalibrationProblem problem(
        std::move(*data), A_single, 6, bounds, mask);

    hexaarch::evaluation::EvaluationContext context;
    hexaarch::calibration::CalibrationParameters initial;
    initial.figure_of_merit                   = context.figure_of_merit;
    initial.motor_efficiency                  = context.motor_efficiency;
    initial.esc_efficiency                    = context.esc_efficiency;
    initial.battery_specific_energy_wh_per_kg = context.battery_specific_energy_wh_per_kg;
    initial.battery_pack_efficiency           = context.battery_pack_efficiency;
    initial.parasite_drag_area_m2             = context.parasite_drag_area_m2;

    hexaarch::calibration::CalibrationOptions opt;
    opt.verbose = true;
    const auto outcome = hexaarch::calibration::Calibrator{}.fit(problem, initial, opt);

    std::cout << std::fixed << std::setprecision(6)
              << "[" << currentTimestamp() << "] Calibration: iter=" << outcome.diagnostics.iterations
              << "  cost(init)=" << outcome.diagnostics.initial_cost
              << "  cost(final)=" << outcome.diagnostics.final_cost
              << "  converged=" << (outcome.diagnostics.converged ? "yes" : "no") << '\n'
              << "[" << currentTimestamp() << "] Residual: mean=" << outcome.diagnostics.mean_residual_w
              << " W  max=" << outcome.diagnostics.max_residual_w << " W\n"
              << "[" << currentTimestamp() << "] Parameters:\n"
              << "  figure_of_merit                 = " << outcome.parameters.figure_of_merit << "\n"
              << "  motor_efficiency                = " << outcome.parameters.motor_efficiency << "\n"
              << "  esc_efficiency                  = " << outcome.parameters.esc_efficiency << "\n"
              << "  battery_specific_energy_wh/kg   = " << outcome.parameters.battery_specific_energy_wh_per_kg << "\n"
              << "  battery_pack_efficiency         = " << outcome.parameters.battery_pack_efficiency << "\n"
              << "  parasite_drag_area_m2           = " << outcome.parameters.parasite_drag_area_m2 << "\n";

    if (ensureOutputDirectory(options.output_dir)) {
        // Per-point residuals CSV: lets the user plot measured vs. predicted in Excel/Python.
        const auto resid_path = options.output_dir / "calibration_residuals.csv";
        std::ofstream resid(resid_path);
        if (resid) {
            resid << "label,mass_kg,airspeed_mps,climb_rate_mps,thrust_total_n,"
                     "power_measured_w,power_predicted_w,residual_w,relative_error_pct\n";
            for (const auto& p : problem.data()) {
                const double pred = problem.predictPower(outcome.parameters, p);
                const double res = pred - p.power_total_w;
                const double rel = res / std::max(p.power_total_w, 1.0) * 100.0;
                resid << p.label << "," << p.mass_kg << "," << p.airspeed_mps << ","
                      << p.climb_rate_mps << "," << p.thrust_total_n << ","
                      << p.power_total_w << "," << pred << "," << res << "," << rel << "\n";
            }
            std::cout << "[" << currentTimestamp() << "] Calibration residuals CSV: "
                      << resid_path.string() << '\n';
        }

        const auto out_path = options.output_dir / "calibration_result.json";
        std::ofstream out(out_path);
        if (out) {
            out << "{\n"
                << "  \"flight_data_csv\": \"" << options.calibration_csv_path->generic_string() << "\",\n"
                << "  \"iterations\": " << outcome.diagnostics.iterations << ",\n"
                << "  \"converged\": " << (outcome.diagnostics.converged ? "true" : "false") << ",\n"
                << "  \"cost_initial\": " << outcome.diagnostics.initial_cost << ",\n"
                << "  \"cost_final\": " << outcome.diagnostics.final_cost << ",\n"
                << "  \"residual_mean_w\": " << outcome.diagnostics.mean_residual_w << ",\n"
                << "  \"residual_max_w\": " << outcome.diagnostics.max_residual_w << ",\n"
                << "  \"parameters\": {\n"
                << "    \"figure_of_merit\": " << outcome.parameters.figure_of_merit << ",\n"
                << "    \"motor_efficiency\": " << outcome.parameters.motor_efficiency << ",\n"
                << "    \"esc_efficiency\": " << outcome.parameters.esc_efficiency << ",\n"
                << "    \"battery_specific_energy_wh_per_kg\": " << outcome.parameters.battery_specific_energy_wh_per_kg << ",\n"
                << "    \"battery_pack_efficiency\": " << outcome.parameters.battery_pack_efficiency << ",\n"
                << "    \"parasite_drag_area_m2\": " << outcome.parameters.parasite_drag_area_m2 << "\n"
                << "  }\n"
                << "}\n";
            std::cout << "[" << currentTimestamp() << "] Calibration written to: "
                      << out_path.string() << '\n';
        }
    }
}

// Re-runs the mission evaluator (cheap) only to grab the per-segment breakdown for CSV export.
// The Stage1 result already carries the aggregate totals; segment-level detail isn't kept there
// to avoid bloating EvaluationResult, so we recompute via MissionEvaluator directly.
void writeMissionSegmentsCsv(
    const std::filesystem::path& csv_path,
    const hexaarch::evaluation::EvaluationContext& context,
    const hexaarch::evaluation::EvaluationResult& result) {
    if (!context.mission_profile) return;
    constexpr double kPi = 3.14159265358979323846;
    const double r_eff = std::max(result.physical_model.structural.max_arm_length * 0.5, 0.1);
    const double single_disk_area = kPi * r_eff * r_eff;
    const auto mission_result = hexaarch::mission::MissionEvaluator{}.evaluate(
        *context.mission_profile, result.powertrain, result.physical_model,
        single_disk_area, context);
    std::ofstream f(csv_path);
    if (!f) return;
    f << "idx,kind,label,duration_s,airspeed_mps,climb_rate_mps,power_w,energy_wh,distance_m\n";
    for (std::size_t i = 0; i < mission_result.segments.size(); ++i) {
        const auto& s = mission_result.segments[i];
        f << i << "," << hexaarch::mission::segmentKindToString(s.kind) << ","
          << s.label << "," << s.duration_s << "," << s.airspeed_mps << ","
          << s.climb_rate_mps << "," << s.electrical_power_w << ","
          << s.energy_wh << "," << s.distance_m << "\n";
    }
}

void runMission(
    const hexaarch::core::HexacopterArchitecture& architecture,
    const hexaarch::evaluation::EvaluationContext& context,
    const CliOptions& options) {
    const auto result = hexaarch::evaluation::ArchitectureEvaluator{}.evaluate(architecture, context);
    std::cout << "[" << currentTimestamp() << "] Mission eval: "
              << hexaarch::analysis::ComparisonReporter::summarize(result) << '\n';
    printMissionSummary(result);
    if (options.plot_acs) {
        runAcsPlot(result, options, "Mission");
    }

    if (ensureOutputDirectory(options.output_dir) && result.stage1.mission_active) {
        const auto segs_path = options.output_dir / "mission_segments.csv";
        writeMissionSegmentsCsv(segs_path, context, result);
        std::cout << "[" << currentTimestamp() << "] Mission segments CSV: "
                  << segs_path.string() << '\n';
        const auto out_path = options.output_dir / "mission_result.json";
        std::ofstream out(out_path);
        if (out) {
            out << std::fixed << std::setprecision(4)
                << "{\n"
                << "  \"name\": \"" << (context.mission_profile ? context.mission_profile->name : "") << "\",\n"
                << "  \"feasible\": " << (result.feasible ? "true" : "false") << ",\n"
                << "  \"total_time_s\": " << result.stage1.mission_total_time_s << ",\n"
                << "  \"total_distance_m\": " << result.stage1.mission_total_distance_m << ",\n"
                << "  \"cruise_distance_m\": " << result.stage1.mission_cruise_distance_m << ",\n"
                << "  \"total_energy_wh\": " << result.stage1.mission_total_energy_wh << ",\n"
                << "  \"energy_with_aux_wh\": " << result.stage1.mission_energy_with_aux_wh << ",\n"
                << "  \"hover_energy_wh\": " << result.stage1.mission_hover_energy_wh << ",\n"
                << "  \"cruise_energy_wh\": " << result.stage1.mission_cruise_energy_wh << ",\n"
                << "  \"peak_power_w\": " << result.stage1.mission_peak_power_w << ",\n"
                << "  \"available_energy_wh\": " << result.stage1.bat_available_energy_wh << ",\n"
                << "  \"energy_reserve_fraction\": " << result.stage1.mission_energy_reserve_fraction << ",\n"
                << "  \"c_rate\": " << result.stage1.bat_c_rate << "\n"
                << "}\n";
            std::cout << "[" << currentTimestamp() << "] Mission summary written to: "
                      << out_path.string() << '\n';
        }
    }
}

void runAcsPlot(
    const hexaarch::evaluation::EvaluationResult& result,
    const CliOptions& options,
    const std::string& label) {
    const auto acs_dir = options.output_dir / "acs";
    if (!ensureOutputDirectory(acs_dir)) {
        return;
    }
    hexaarch::analysis::AcsPlotter::Config cfg;
    cfg.output_dir = acs_dir.string();
    cfg.label = label;
    const double mg =
        result.physical_model.mass_properties.mass * result.physical_model.propulsion.gravity;
    hexaarch::analysis::AcsPlotter{}.plot(
        result.physical_model.allocation_matrix,
        result.physical_model.propulsion.thrust_max,
        mg,
        result.acs,
        cfg);
    std::cout << "[" << currentTimestamp() << "] ACS plots written to: " << acs_dir.string() << '\n';
    std::cout << "[" << currentTimestamp() << "] ACS metrics:"
              << "  PFWAR=" << std::fixed << std::setprecision(4) << result.acs.PFWAR
              << "  FII=" << result.acs.FII
              << "  WCFR=" << result.acs.WCFR
              << "  hover_margin=" << result.acs.hover_margin << '\n';
}

void printBaseline(
    const hexaarch::core::HexacopterArchitecture& architecture,
    const hexaarch::evaluation::EvaluationContext& context,
    const CliOptions& options) {
    const auto baseline = hexaarch::evaluation::ArchitectureEvaluator{}.evaluate(architecture, context);
    std::cout << "[" << currentTimestamp() << "] Architecture id: " << architecture.id() << '\n';
    std::cout << "[" << currentTimestamp() << "] Baseline: "
              << hexaarch::analysis::ComparisonReporter::summarize(baseline) << '\n';
    printMissionSummary(baseline);
    if (options.plot_acs) {
        runAcsPlot(baseline, options, "Baseline");
    }
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
    std::cout << "[" << currentTimestamp() << "] Parameters:\n"
              << hexaarch::analysis::ComparisonReporter::parametersTable(result);
    if (result.best_feasible.has_value()) {
        std::cout << "[" << currentTimestamp() << "] Best feasible: "
                  << hexaarch::analysis::ComparisonReporter::summarize(result.best_feasible->result) << '\n';
        std::cout << "[" << currentTimestamp() << "] Delta: "
                  << hexaarch::analysis::ComparisonReporter::compare(result.baseline, result.best_feasible->result) << '\n';
        printMissionSummary(result.best_feasible->result);
    } else {
        std::cout << "[" << currentTimestamp() << "] Best feasible: none found\n";
        std::cout << "[" << currentTimestamp() << "] Raw best: "
                  << hexaarch::analysis::ComparisonReporter::summarize(result.best_result) << '\n';
    }
    std::cout << "[" << currentTimestamp() << "] Exports: comparison=" << comparison_written
              << ", parameters=" << parameter_written
              << ", soo_json=" << soo_written << '\n';
    std::cout << "[" << currentTimestamp() << "] Output directory: " << options.output_dir.string() << '\n';

    if (options.plot_acs) {
        if (result.best_feasible.has_value()) {
            runAcsPlot(result.best_feasible->result, options, "SOO-Best");
        } else {
            runAcsPlot(result.baseline, options, "SOO-Baseline");
        }
    }
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
    std::cout << "[" << currentTimestamp() << "] Parameters:\n"
              << hexaarch::analysis::ComparisonReporter::parametersTable(result);
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
    std::cout << "[" << currentTimestamp() << "] SOO Parameters:\n"
              << hexaarch::analysis::ComparisonReporter::parametersTable(soo_result);
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

    if (options.plot_acs) {
        runAcsPlot(baseline, options, "Compare-Baseline");
        if (soo_result.best_feasible.has_value()) {
            runAcsPlot(soo_result.best_feasible->result, options, "Compare-SOO-Best");
        }
    }
}

hexaarch::core::HexacopterArchitecture architectureFromDecisionVector(
    const std::vector<double>& decision_vector) {
    hexaarch::core::HexacopterArchitecture arch;
    hexaarch::optimization::DesignVectorMapper{}.unpackNormalized(arch, decision_vector);
    arch.rebuildAssembly();
    return arch;
}

int runSooWithVisualization(
    const hexaarch::core::HexacopterArchitecture& architecture,
    const hexaarch::evaluation::EvaluationContext& context,
    const CliOptions& options) {
    hexaarch::core::HexacopterArchitecture init_arch = architecture;
    init_arch.rebuildAssembly();

    hexaarch::visualization::ArchitectureViewerApp app;
    app.setArchitecture(init_arch);

    std::optional<hexaarch::optimization::SooRunResult> run_result;

    std::thread opt_thread([&]() {
        hexaarch::optimization::SooRunConfig config;
        config.population_size = options.soo_population_size;
        config.generations = options.soo_generations;
        config.on_generation = [&](unsigned gen, unsigned total, const std::vector<double>& best_x) {
            const auto candidate = architectureFromDecisionVector(best_x);
            const auto evaluation = hexaarch::evaluation::ArchitectureEvaluator{}.evaluate(candidate, context);
            if (evaluation.feasible) {
                app.postArchitecture(
                    candidate,
                    "HexaArch SOO  Feasible Gen " + std::to_string(gen) + "/" + std::to_string(total));
            }
        };
        if (!ensureOutputDirectory(options.output_dir)) { return; }
        const auto result = hexaarch::optimization::SooRunner{}.run(architecture, context, config);
        (void)hexaarch::analysis::CsvExporter::writeSooComparisonCsv(options.output_dir / "comparison.csv", result);
        (void)hexaarch::analysis::CsvExporter::writeSooParametersCsv(options.output_dir / "soo_parameters.csv", result);
        (void)hexaarch::analysis::CsvExporter::writeSooJson(options.output_dir / "soo_run.json", result);

        if (result.best_feasible.has_value()) {
            app.postArchitecture(
                architectureFromDecisionVector(result.best_feasible->decision_vector),
                "HexaArch SOO  Complete — " + hexaarch::analysis::ComparisonReporter::summarize(result));
        }
        std::cout << "[" << currentTimestamp() << "] SOO complete: "
                  << hexaarch::analysis::ComparisonReporter::summarize(result) << '\n';
        run_result = result;
    });

    const int viewer_result = app.run();
    opt_thread.join();

    if (options.plot_acs && run_result.has_value()) {
        const auto& res = *run_result;
        if (res.best_feasible.has_value()) {
            runAcsPlot(res.best_feasible->result, options, "SOO-Best");
        } else {
            runAcsPlot(res.baseline, options, "SOO-Baseline");
        }
    }

    return viewer_result;
}

int runMooWithVisualization(
    const hexaarch::core::HexacopterArchitecture& architecture,
    const hexaarch::evaluation::EvaluationContext& context,
    const CliOptions& options) {
    hexaarch::core::HexacopterArchitecture init_arch = architecture;
    init_arch.rebuildAssembly();

    hexaarch::visualization::ArchitectureViewerApp app;
    app.setArchitecture(init_arch);

    std::thread opt_thread([&]() {
        hexaarch::optimization::MooRunConfig config;
        config.population_size = options.moo_population_size;
        config.generations = options.moo_generations;
        config.objective_names = {"mass", "power", "fault_alloc"};
        config.on_generation = [&](unsigned gen, unsigned total, const std::vector<double>& best_x) {
            const auto candidate = architectureFromDecisionVector(best_x);
            const auto evaluation = hexaarch::evaluation::ArchitectureEvaluator{}.evaluate(candidate, context);
            if (evaluation.feasible) {
                app.postArchitecture(
                    candidate,
                    "HexaArch MOO  Feasible Gen " + std::to_string(gen) + "/" + std::to_string(total));
            }
        };
        if (!ensureOutputDirectory(options.output_dir)) { return; }
        const auto result = hexaarch::optimization::MooRunner{}.run(architecture, context, config);
        (void)hexaarch::analysis::CsvExporter::writeParetoCsv(options.output_dir / "pareto_front.csv", result);
        (void)hexaarch::analysis::CsvExporter::writeParetoParametersCsv(options.output_dir / "pareto_parameters.csv", result);
        (void)hexaarch::analysis::CsvExporter::writeMooJson(options.output_dir / "moo_run.json", result);

        if (result.has_feasible_points && !result.feasible_population.empty()) {
            const auto& knee = result.feasible_population.front();
            app.postArchitecture(
                architectureFromDecisionVector(knee.decision_vector),
                "HexaArch MOO  Complete — " + hexaarch::analysis::ComparisonReporter::summarize(result));
        }
        std::cout << "[" << currentTimestamp() << "] MOO complete: "
                  << hexaarch::analysis::ComparisonReporter::summarize(result) << '\n';
    });

    const int viewer_result = app.run();
    opt_thread.join();
    return viewer_result;
}

int runVisualizerFromSooJson(const std::filesystem::path& json_path) {
    std::ifstream file(json_path);
    if (!file) {
        std::cerr << "Cannot open: " << json_path << '\n';
        return 1;
    }
    const auto doc = nlohmann::json::parse(file, nullptr, false);
    if (doc.is_discarded()) {
        std::cerr << "Failed to parse: " << json_path << '\n';
        return 1;
    }

    std::vector<double> dv;
    if (doc.contains("best_feasible_decision_vector") && !doc["best_feasible_decision_vector"].is_null()) {
        dv = doc["best_feasible_decision_vector"].get<std::vector<double>>();
    } else {
        std::cerr << "No feasible SOO solution in: " << json_path << '\n';
        return 1;
    }

    auto arch = architectureFromDecisionVector(dv);
    hexaarch::visualization::ArchitectureViewerApp app;
    app.setArchitecture(arch);
    return app.run();
}

int runVisualizerFromMooJson(const std::filesystem::path& json_path) {
    std::ifstream file(json_path);
    if (!file) {
        std::cerr << "Cannot open: " << json_path << '\n';
        return 1;
    }
    const auto doc = nlohmann::json::parse(file, nullptr, false);
    if (doc.is_discarded()) {
        std::cerr << "Failed to parse: " << json_path << '\n';
        return 1;
    }

    std::vector<std::string> obj_names;
    if (doc.contains("objective_names")) {
        obj_names = doc["objective_names"].get<std::vector<std::string>>();
    }

    struct Point {
        std::vector<double> decision_vector;
        std::vector<double> objective_vector;
    };
    std::vector<Point> feasible;
    if (doc.contains("population")) {
        for (const auto& p : doc["population"]) {
            const bool is_feasible =
                p.contains("evaluation") &&
                p["evaluation"].contains("feasible") &&
                p["evaluation"]["feasible"].get<bool>();
            if (is_feasible) {
                feasible.push_back({
                    p["decision_vector"].get<std::vector<double>>(),
                    p["objective_vector"].get<std::vector<double>>()
                });
            }
        }
    }

    if (feasible.empty()) {
        std::cerr << "No feasible points in MOO result.\n";
        return 1;
    }

    std::cout << "Feasible Pareto front (" << feasible.size() << " points):\n";
    for (std::size_t i = 0; i < feasible.size(); ++i) {
        std::cout << "  " << (i + 1) << ".";
        for (std::size_t j = 0; j < feasible[i].objective_vector.size(); ++j) {
            const std::string name = j < obj_names.size() ? obj_names[j] : ("obj" + std::to_string(j));
            std::cout << "  " << name << "=" << std::fixed << std::setprecision(3)
                      << feasible[i].objective_vector[j];
        }
        std::cout << '\n';
    }
    std::cout << "Select point [1-" << feasible.size() << "]: ";

    int sel = 1;
    std::cin >> sel;
    if (sel < 1 || sel > static_cast<int>(feasible.size())) {
        std::cerr << "Invalid selection.\n";
        return 1;
    }

    auto arch = architectureFromDecisionVector(feasible[static_cast<std::size_t>(sel - 1)].decision_vector);
    hexaarch::visualization::ArchitectureViewerApp app;
    app.setArchitecture(arch);
    return app.run();
}

int runVisualizer(
    hexaarch::core::HexacopterArchitecture& architecture,
    const CliOptions& options) {
    while (true) {
        std::vector<std::filesystem::path> json_files;
        if (std::filesystem::exists(options.output_dir)) {
            for (const auto& entry : std::filesystem::directory_iterator(options.output_dir)) {
                const auto& p = entry.path();
                if (p.extension() == ".json") {
                    json_files.push_back(p);
                }
            }
            std::sort(json_files.begin(), json_files.end());
        }

        std::cout << "\nSelect model to visualize:\n";
        std::cout << "  0. Baseline architecture\n";
        for (std::size_t i = 0; i < json_files.size(); ++i) {
            std::cout << "  " << (i + 1) << ". " << json_files[i].filename().string() << '\n';
        }
        std::cout << "  q. Quit\n";
        std::cout << "Selection: ";

        std::string input;
        std::cin >> input;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        if (input == "q" || input == "Q") {
            break;
        }

        int sel = -1;
        try { sel = std::stoi(input); } catch (...) { continue; }

        if (sel == 0) {
            architecture.rebuildAssembly();
            hexaarch::visualization::ArchitectureViewerApp app;
            app.setArchitecture(architecture);
            app.run();
            continue;
        }

        if (sel < 1 || sel > static_cast<int>(json_files.size())) {
            std::cout << "Invalid selection.\n";
            continue;
        }

        const auto& chosen = json_files[static_cast<std::size_t>(sel - 1)];
        if (chosen.filename().string().find("moo") != std::string::npos) {
            runVisualizerFromMooJson(chosen);
        } else {
            runVisualizerFromSooJson(chosen);
        }
    }
    return 0;
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

    // Attach mission profile early so every downstream mode sees the multi-segment energy budget.
    attachMissionProfile(*parsed, context);

    switch (parsed->mode) {
        case CliMode::evaluate:
            printBaseline(architecture, context, *parsed);
            break;
        case CliMode::soo:
            if (parsed->visualize) {
                return runSooWithVisualization(architecture, context, *parsed);
            }
            runSoo(architecture, context, *parsed);
            break;
        case CliMode::moo:
            if (parsed->visualize) {
                return runMooWithVisualization(architecture, context, *parsed);
            }
            runMoo(architecture, context, *parsed);
            break;
        case CliMode::compare:
            runCompare(architecture, context, *parsed);
            break;
        case CliMode::visualize:
            return runVisualizer(architecture, *parsed);
        case CliMode::calibrate:
            runCalibration(*parsed);
            break;
        case CliMode::mission:
            if (!context.mission_profile) {
                std::cerr << "[" << currentTimestamp()
                          << "] 'mission' mode requires --mission <profile.json> or a positional arg.\n";
                return 1;
            }
            runMission(architecture, context, *parsed);
            break;
    }

    return 0;
}
