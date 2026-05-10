#include "evaluation/Stage1Evaluator.hpp"

namespace hexaarch::evaluation {

EvaluationResult Stage1Evaluator::evaluate(
    const core::HexacopterArchitecture&,
    const EvaluationContext&) const {
    return EvaluationResult{
        .feasible = true,
        .combined_objective = 0.0,
        .message = "Phase 0 placeholder Stage 1 evaluation"
    };
}

}  // namespace hexaarch::evaluation
