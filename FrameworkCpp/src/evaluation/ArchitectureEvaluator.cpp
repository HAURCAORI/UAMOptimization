#include "evaluation/ArchitectureEvaluator.hpp"

#include "evaluation/Stage1Evaluator.hpp"

namespace hexaarch::evaluation {

EvaluationResult ArchitectureEvaluator::evaluate(
    const core::HexacopterArchitecture& architecture,
    const EvaluationContext& context) const {
    return Stage1Evaluator{}.evaluate(architecture, context);
}

}  // namespace hexaarch::evaluation
