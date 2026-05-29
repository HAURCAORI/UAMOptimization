#include "core/Constraint.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace hexaarch::core {

std::string Constraint::stable_id() const {
    return owner_id + "::" + name;
}

ConstraintEvaluation Constraint::evaluate(const double value) const {
    ConstraintEvaluation result;
    result.value = value;

    switch (sense) {
    case ConstraintSense::less_equal:
        result.violation = value <= threshold ? 0.0 : value - threshold;
        break;
    case ConstraintSense::greater_equal:
        result.violation = value >= threshold ? 0.0 : threshold - value;
        break;
    case ConstraintSense::equal:
        result.violation = std::abs(value - threshold);
        break;
    }

    result.feasible = result.violation <= 1e-9;
    return result;
}

ConstraintEvaluation Constraint::evaluate(const ConstraintEvaluationContext& context) const {
    if (!evaluator) {
        throw std::logic_error("Constraint '" + stable_id() + "' has no evaluator set");
    }
    try {
        return evaluator(context);
    } catch (const std::exception& ex) {
        std::cerr << "[Constraint] exception in '" << stable_id() << "': " << ex.what() << '\n';
        ConstraintEvaluation fallback;
        fallback.value     = (sense == ConstraintSense::greater_equal) ? threshold - 1.0 : threshold + 1.0;
        fallback.violation = 1.0;
        fallback.feasible  = false;
        return fallback;
    } catch (...) {
        std::cerr << "[Constraint] unknown exception in '" << stable_id() << "'\n";
        ConstraintEvaluation fallback;
        fallback.value     = (sense == ConstraintSense::greater_equal) ? threshold - 1.0 : threshold + 1.0;
        fallback.violation = 1.0;
        fallback.feasible  = false;
        return fallback;
    }
}

}  // namespace hexaarch::core
