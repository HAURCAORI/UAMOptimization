#pragma once

#include <functional>
#include <string>

namespace hexaarch::core {
class HexacopterArchitecture;
}

namespace hexaarch::physics {
struct PhysicalModel;
}

namespace hexaarch::evaluation {
struct EvaluationContext;
struct Stage1Metrics;
}

namespace hexaarch::core {

enum class ConstraintSense {
    less_equal,
    greater_equal,
    equal
};

struct ConstraintEvaluation {
    double value = 0.0;
    double violation = 0.0;
    bool feasible = true;
};

struct ConstraintEvaluationContext {
    const HexacopterArchitecture& architecture;
    const physics::PhysicalModel& physical_model;
    const evaluation::Stage1Metrics& stage1_metrics;
    const evaluation::EvaluationContext& evaluation_context;
};

struct Constraint {
    std::string name;
    std::string owner_id;
    ConstraintSense sense = ConstraintSense::greater_equal;
    double threshold = 0.0;
    bool hard = true;
    bool active = true;
    double penalty = 0.0;
    std::function<ConstraintEvaluation(const ConstraintEvaluationContext&)> evaluator;

    [[nodiscard]] std::string stable_id() const;
    [[nodiscard]] ConstraintEvaluation evaluate(double value) const;
    [[nodiscard]] ConstraintEvaluation evaluate(const ConstraintEvaluationContext& context) const;
};

}  // namespace hexaarch::core
