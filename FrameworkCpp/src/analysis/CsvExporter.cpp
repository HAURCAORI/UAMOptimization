#include "analysis/CsvExporter.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>

#include "analysis/ComparisonReporter.hpp"
#include "analysis/ParetoAnalyzer.hpp"
#include "core/Timestamp.hpp"
#include "evaluation/MetricRole.hpp"
#include "nlohmann/json.hpp"

namespace hexaarch::analysis {
namespace {

using hexaarch::core::currentTimestamp;

std::string shortParamName(const std::string& stable_id) {
    const auto pos = stable_id.rfind("::");
    return pos != std::string::npos ? stable_id.substr(pos + 2) : stable_id;
}

const optimization::MooPoint* representativeMooPoint(const optimization::MooRunResult& result) {
    if (result.feasible_population.empty()) {
        return nullptr;
    }

    const auto summary = ParetoAnalyzer{}.analyze(result);
    const auto knee_it = std::find_if(
        result.feasible_population.begin(), result.feasible_population.end(),
        [&](const optimization::MooPoint& p) { return p.population_index == summary.knee_index; });
    if (knee_it != result.feasible_population.end()) {
        return &*knee_it;
    }

    return &result.feasible_population.front();
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
        arr.push_back({
            {"name", obj.name},
            {"value", obj.value},
            {"weight", obj.weight},
            {"role", evaluation::metricRoleName(obj.role)}
        });
    }
    return arr;
}

nlohmann::json constraintListToJson(const std::vector<evaluation::ConstraintResult>& constraints) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& cr : constraints) {
        arr.push_back({
            {"id", cr.stable_id},
            {"role", evaluation::metricRoleName(cr.metricRole())},
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
        {"structural_safety", m.structural_safety},
        {"gamma_worst", m.gamma_worst}, {"sigma_worst", m.sigma_worst},
        {"sigma_reference", m.sigma_reference},
        {"hover_utilization_nominal", m.hover_utilization_nominal},
        {"hover_ok_nominal", m.hover_ok_nominal},
        {"acs_nominal_feasible", m.acs_nominal_feasible},
        {"acs_all_faults_feasible", m.acs_all_faults_feasible},
        {"acs_nominal_min_margin", m.acs_nominal_min_margin},
        {"acs_worst_fault_min_margin", m.acs_worst_fault_min_margin},
        {"acs_overall_min_margin", m.acs_overall_min_margin},
        {"acs_margin_penalty", m.acs_margin_penalty},
        {"acs_PFWAR", m.acs_PFWAR},
        {"acs_FII", m.acs_FII},
        {"acs_WCFR", m.acs_WCFR},
        {"acs_hover_margin", m.acs_hover_margin},
        {"acs_hover_slice_worst_fault_margin", m.acs_hover_slice_worst_fault_margin},
        {"pt_total_power_nominal_w", m.pt_total_power_nominal_w},
        {"pt_total_power_faulted_w", m.pt_total_power_faulted_w},
        {"pt_worst_thrust_utilization", m.pt_worst_thrust_utilization},
        {"pt_worst_power_utilization", m.pt_worst_power_utilization},
        {"bat_available_energy_wh", m.bat_available_energy_wh},
        {"bat_required_energy_wh", m.bat_required_energy_wh},
        {"bat_energy_reserve_fraction", m.bat_energy_reserve_fraction},
        {"bat_c_rate", m.bat_c_rate},
        {"bat_mass_fraction", m.bat_mass_fraction},
        {"bat_achievable_endurance_nom_min", m.bat_achievable_endurance_nom_min},
        {"struct_net_min_safety_factor", m.struct_net_min_safety_factor},
        {"struct_net_max_tip_deflection_m", m.struct_net_max_tip_deflection_m},
        {"struct_net_max_tip_rotation_rad", m.struct_net_max_tip_rotation_rad},
        {"struct_net_max_sigma_vm_pa", m.struct_net_max_sigma_vm_pa},
        {"pkg_rotor_clearance_m",           m.pkg_rotor_clearance_m},
        {"pkg_payload_containment_m",       m.pkg_payload_containment_m},
        {"pkg_battery_containment_m",       m.pkg_battery_containment_m},
        {"pkg_battery_payload_overlap_m",   m.pkg_battery_payload_overlap_m},
        {"pkg_payload_internal_overlap_m",  m.pkg_payload_internal_overlap_m},
        {"cabin_internal_clearance_m",      m.cabin_internal_clearance_m},
        {"cabin_space_penalty",             m.cabin_space_penalty},
        {"pkg_occupant_containment_m",      m.pkg_occupant_containment_m},
        {"cg_y_offset_m",                   m.cg_y_offset_m},
        {"pkg_rotor_keepout_m",             m.pkg_rotor_keepout_m}
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
    nlohmann::json arm_structural = nlohmann::json::array();
    for (const auto& arm : model.arm_structural) {
        arm_structural.push_back({
            {"arm_id", arm.arm_id},
            {"L_arm", arm.L_arm},
            {"M_vertical", arm.M_vertical},
            {"M_horizontal", arm.M_horizontal},
            {"M_total", arm.M_total},
            {"sigma_bending", arm.sigma_bending},
            {"safety_factor", arm.safety_factor},
            {"structural_failure", arm.structural_failure}
        });
    }
    return {
        {"mass", model.mass_properties.mass},
        {"center_of_mass", {com.x(), com.y(), com.z()}},
        {"inertia_diagonal", {I(0, 0), I(1, 1), I(2, 2)}},
        {"minimum_rotor_clearance", model.packaging.minimum_rotor_clearance},
        {"overlap_penalty", model.packaging.overlap_penalty},
        {"arm_span", model.structural.arm_span},
        {"normalized_bending_index", model.structural.normalized_bending_index},
        {"min_safety_factor", model.structural.min_safety_factor},
        {"arm_structural", arm_structural},
        {"allocation_matrix", alloc}
    };
}

nlohmann::json metricDescriptorsToJson() {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& d : evaluation::stage1MetricDescriptors()) {
        arr.push_back({
            {"name", d.name},
            {"role", evaluation::metricRoleName(d.role)},
            {"unit", d.unit}
        });
    }
    return arr;
}

nlohmann::json evaluationToJson(const evaluation::EvaluationResult& result) {
    return {
        {"feasible", result.feasible},
        {"combined_objective", result.combined_objective},
        {"message", result.message},
        {"stage1", stage1ToJson(result.stage1)},
        {"objectives", objectiveListToJson(result.objectives)},
        {"constraints", constraintListToJson(result.constraint_results)},
        {"metric_descriptors", metricDescriptorsToJson()},
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
        {"minimum_arm_safety_factor", context.minimum_arm_safety_factor},
        {"arm_material", {
            {"name", context.arm_material.name},
            {"density", context.arm_material.density},
            {"yield_strength", context.arm_material.yield_strength},
            {"elastic_modulus", context.arm_material.elastic_modulus}
        }},
        {"figure_of_merit", context.figure_of_merit},
        {"motor_efficiency", context.motor_efficiency},
        {"esc_efficiency", context.esc_efficiency},
        {"air_density", context.air_density},
        {"battery_specific_energy_wh_per_kg", context.battery_specific_energy_wh_per_kg},
        {"battery_dod_usable", context.battery_dod_usable},
        {"battery_pack_efficiency", context.battery_pack_efficiency},
        {"battery_voltage_nominal", context.battery_voltage_nominal},
        {"battery_crate_limit", context.battery_crate_limit},
        {"mission_time_nominal_min", context.mission_time_nominal_min},
        {"mission_time_emergency_min", context.mission_time_emergency_min},
        {"power_auxiliary_w", context.power_auxiliary_w},
        {"arm_tip_deflection_limit_m", context.arm_tip_deflection_limit_m},
        {"arm_tip_rotation_limit_rad", context.arm_tip_rotation_limit_rad},
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
    const bool has_knee = !summary.nondominated_indices.empty();

    stream << "population_index,is_nondominated,is_knee,feasible,combined_objective,gamma_worst,sigma_worst,min_safety_factor,acs_PFWAR,acs_WCFR,acs_FII,acs_hover_margin,acs_hover_slice_worst_fault_margin";
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
               << (has_knee && summary.knee_index == point.population_index ? "true" : "false") << ','
               << (point.evaluation.feasible ? "true" : "false") << ','
               << point.evaluation.combined_objective << ','
               << point.evaluation.stage1.gamma_worst << ','
               << point.evaluation.stage1.sigma_worst << ','
               << point.evaluation.physical_model.structural.min_safety_factor << ','
               << point.evaluation.stage1.acs_PFWAR << ','
               << point.evaluation.stage1.acs_WCFR << ','
               << point.evaluation.stage1.acs_FII << ','
               << point.evaluation.stage1.acs_hover_margin << ','
               << point.evaluation.stage1.acs_hover_slice_worst_fault_margin;

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
