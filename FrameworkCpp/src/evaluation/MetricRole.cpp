#include "evaluation/MetricRole.hpp"

namespace hexaarch::evaluation {

std::string metricRoleName(const MetricRole role) {
    switch (role) {
    case MetricRole::hard_constraint: return "hard_constraint";
    case MetricRole::soft_objective:  return "soft_objective";
    case MetricRole::analysis_only:   return "analysis_only";
    }
    return "unknown";
}

const std::vector<MetricDescriptor>& stage1MetricDescriptors() {
    static const std::vector<MetricDescriptor> table = {
        // --- Soft objectives (weight > 0 in default EvaluationContext) ---
        {"mass",                               MetricRole::soft_objective,   ""},
        {"power",                              MetricRole::soft_objective,   ""},
        {"fault_thrust",                       MetricRole::soft_objective,   ""},
        {"fault_alloc",                        MetricRole::soft_objective,   ""},
        {"hover_nom",                          MetricRole::soft_objective,   ""},
        {"acs_margin_penalty",                 MetricRole::soft_objective,   ""},

        // --- Zero-weight objectives (analysis role in practice) ---
        {"structural",                         MetricRole::analysis_only,    ""},
        {"packaging",                          MetricRole::analysis_only,    ""},
        {"structural_safety",                  MetricRole::analysis_only,    ""},

        // --- Legacy fault-tolerance proxies ---
        {"gamma_worst",                        MetricRole::analysis_only,    ""},
        {"sigma_worst",                        MetricRole::analysis_only,    ""},
        {"sigma_reference",                    MetricRole::analysis_only,    ""},

        // --- Hover feasibility (hard: gates result.feasible) ---
        {"hover_ok_nominal",                   MetricRole::hard_constraint,  "bool"},
        {"hover_utilization_nominal",          MetricRole::analysis_only,    ""},

        // --- ACS hard feasibility flags ---
        {"acs_nominal_feasible",               MetricRole::hard_constraint,  "bool"},
        {"acs_all_faults_feasible",            MetricRole::hard_constraint,  "bool"},

        // --- ACS directional margins ---
        {"acs_nominal_min_margin",             MetricRole::analysis_only,    "N or Nm"},
        {"acs_worst_fault_min_margin",         MetricRole::hard_constraint,  "N or Nm"},
        {"acs_overall_min_margin",             MetricRole::analysis_only,    "N or Nm"},

        // --- ACS volume metrics ---
        {"acs_PFWAR",                          MetricRole::analysis_only,    ""},
        {"acs_FII",                            MetricRole::analysis_only,    ""},
        {"acs_WCFR",                           MetricRole::analysis_only,    ""},

        // --- ACS hover margin (hard: feeds all_faults_hover_feasible) ---
        {"acs_hover_margin",                   MetricRole::hard_constraint,  ""},

        // --- ACS per-axis reserves ---
        {"acs_yaw_reserve",                    MetricRole::analysis_only,    "Nm"},
        {"acs_roll_reserve",                   MetricRole::analysis_only,    "Nm"},
        {"acs_pitch_reserve",                  MetricRole::analysis_only,    "Nm"},
        {"acs_faulted_to_nominal_ratio",       MetricRole::analysis_only,    ""},
        {"acs_hover_slice_worst_fault_margin", MetricRole::analysis_only,    "Nm"},

        // --- Powertrain metrics ---
        {"pt_total_power_nominal_w",           MetricRole::analysis_only,    "W"},
        {"pt_total_power_faulted_w",           MetricRole::analysis_only,    "W"},
        {"pt_worst_thrust_utilization",        MetricRole::analysis_only,    ""},
        {"pt_worst_power_utilization",         MetricRole::analysis_only,    ""},

        // --- Battery metrics ---
        {"bat_available_energy_wh",            MetricRole::analysis_only,    "Wh"},
        {"bat_required_energy_wh",             MetricRole::analysis_only,    "Wh"},
        {"bat_energy_reserve_fraction",        MetricRole::hard_constraint,  ""},
        {"bat_c_rate",                         MetricRole::hard_constraint,  "1/h"},
        {"bat_mass_fraction",                  MetricRole::analysis_only,    ""},

        // --- Structural network (Phase 3/4) ---
        {"struct_net_min_safety_factor",       MetricRole::hard_constraint,  ""},
        {"struct_net_max_tip_deflection_m",    MetricRole::hard_constraint,  "m"},
        {"struct_net_max_tip_rotation_rad",    MetricRole::hard_constraint,  "rad"},
        {"struct_net_max_sigma_vm_pa",         MetricRole::analysis_only,    "Pa"},
    };
    return table;
}

}  // namespace hexaarch::evaluation
