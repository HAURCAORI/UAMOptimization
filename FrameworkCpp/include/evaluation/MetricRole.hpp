#pragma once

#include <string>
#include <vector>

namespace hexaarch::evaluation {

enum class MetricRole {
    hard_constraint,  // Feasibility gate — violation makes design infeasible
    soft_objective,   // Enters combined_objective (weight > 0)
    analysis_only     // Computed and exported; no optimization role
};

[[nodiscard]] std::string metricRoleName(MetricRole role);

struct MetricDescriptor {
    std::string name;
    MetricRole role = MetricRole::analysis_only;
    std::string unit;
};

// Full descriptor table for all Stage1Metrics fields, ordered as in the struct.
// Use this to look up the role of any named metric in JSON/CSV exports.
[[nodiscard]] const std::vector<MetricDescriptor>& stage1MetricDescriptors();

}  // namespace hexaarch::evaluation
