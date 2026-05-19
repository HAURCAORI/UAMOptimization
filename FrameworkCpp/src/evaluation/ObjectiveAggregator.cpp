#include "evaluation/ObjectiveAggregator.hpp"

namespace hexaarch::evaluation {

double ObjectiveAggregator::aggregate(EvaluationResult& result, const EvaluationContext& context) const {
    result.objectives.clear();

    const auto append = [&](const std::string& name, const double value) {
        for (const auto& weight : context.objective_weights) {
            if (weight.weight > 0.0 && weight.name == name) {
                result.objectives.push_back({name, value, weight.weight});
                return;
            }
        }
    };

    append("mass", result.stage1.mass);
    append("power", result.stage1.power);
    append("fault_thrust", result.stage1.fault_thrust);
    append("fault_alloc", result.stage1.fault_alloc);
    append("hover_nom", result.stage1.hover_nom);
    append("structural", result.stage1.structural);
    append("packaging", result.stage1.packaging);
    append("structural_safety", result.stage1.structural_safety);
    append("acs_margin", result.stage1.acs_margin_penalty);

    double numerator = 0.0;
    double denominator = 0.0;
    for (const auto& objective : result.objectives) {
        numerator += objective.weight * objective.value;
        denominator += objective.weight;
    }

    result.combined_objective = denominator > 0.0 ? numerator / denominator : 0.0;
    return result.combined_objective;
}

}  // namespace hexaarch::evaluation
